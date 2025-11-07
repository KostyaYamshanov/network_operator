#include "GANOP.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>

GANOP::GANOP(const GAConfig& config)
    : config_(config), rng_(config.seed) {
    
    if (!config.fitness_evaluator) {
        throw std::runtime_error("fitness_evaluator must be provided");
    }
    if (!config.solution_factory) {
        throw std::runtime_error("solution_factory must be provided");
    }
    
    int num_objectives = config.fitness_evaluator->getNumObjectives();
    if (num_objectives <= 0) {
        throw std::invalid_argument("num_objectives must be > 0");
    }
    
    // Инициализация популяции
    int total_bits = config.num_params * (config.int_bits + config.frac_bits);
    
    population_params_.assign(config.population_size, 
                              std::vector<int>(total_bits));
    population_struct_.assign(config.population_size,
                              std::vector<std::vector<int>>(config.num_struct_variations));
    
    fitness_population_.assign(config.population_size,
                               std::vector<float>(num_objectives));
    pareto_ranks_.assign(config.population_size, 0);
}

void GANOP::run() {
    std::cout << "Initializing population..." << std::endl;
    initializePopulation();
    
    std::cout << "Evaluating initial population..." << std::endl;
    evaluatePopulation();
    updateParetoRanks();
    
    // Вызов колбэка для поколения 0
    if (config_.on_generation_end) {
        float sum_fitness = 0.0f;
        int num_obj = config_.fitness_evaluator->getNumObjectives();
        for (int i = 0; i < config_.population_size; ++i) {
            if (num_obj > 0) {
                sum_fitness += fitness_population_[i][num_obj - 1];
            }
        }
        float avg_fitness = config_.population_size > 0 
            ? sum_fitness / static_cast<float>(config_.population_size) 
            : 0.0f;
        config_.on_generation_end(0, avg_fitness);
    }
    
    // Главный цикл эволюции
    for (int generation = 1; generation <= config_.num_generations; ++generation) {
        std::cout << generation << " / " << config_.num_generations << std::endl;
        
        for (int crossover_idx = 0; crossover_idx < config_.num_crossovers_per_gen; ++crossover_idx) {
            // Выбор двух родителей
            int parent1, parent2;
            selectParents(parent1, parent2);
            
            // Проверка вероятности кроссовера
            std::uniform_real_distribution<float> dist_real(0.0f, 1.0f);
            float ksi = dist_real(rng_);
            
            int num_obj = config_.fitness_evaluator->getNumObjectives();
            float prob1 = (1.0f + config_.selection_alpha * pareto_ranks_[parent1]) / 
                          (1.0f + pareto_ranks_[parent1]);
            float prob2 = (1.0f + config_.selection_alpha * pareto_ranks_[parent2]) / 
                          (1.0f + pareto_ranks_[parent2]);
            
            if (ksi < prob1 || ksi < prob2) {
                // Кроссовер
                std::vector<std::vector<int>> offspring_params(4);
                std::vector<std::vector<std::vector<int>>> offspring_struct(4);
                
                crossover(parent1, parent2, offspring_params, offspring_struct);
                
                // Мутация и оценка каждого потомка
                for (int offspring = 0; offspring < 4; ++offspring) {
                    if (dist_real(rng_) < config_.mutation_prob) {
                        mutate(offspring_params[offspring], offspring_struct[offspring]);
                    }
                    
                    // Оценка потомка
                    auto solution = config_.solution_factory();
                    solution->decode(offspring_params[offspring], offspring_struct[offspring]);
                    auto fitness = config_.fitness_evaluator->evaluate(*solution);
                    
                    if (fitness.size() != static_cast<size_t>(num_obj)) {
                        throw std::runtime_error(
                            "Fitness function returned wrong number of objectives: " +
                            std::to_string(fitness.size()) + " (expected " + 
                            std::to_string(num_obj) + ")"
                        );
                    }
                    
                    // Поиск особи с максимальным рангом для замены
                    int worst_idx = 0;
                    int max_rank = pareto_ranks_[0];
                    for (int i = 1; i < config_.population_size; ++i) {
                        if (pareto_ranks_[i] > max_rank) {
                            max_rank = pareto_ranks_[i];
                            worst_idx = i;
                        }
                    }
                    
                    // Замена, если потомок лучше худшей особи
                    int offspring_rank = computeRank(fitness);
                    if (offspring_rank < max_rank) {
                        population_params_[worst_idx] = offspring_params[offspring];
                        population_struct_[worst_idx] = offspring_struct[offspring];
                        fitness_population_[worst_idx] = fitness;
                        
                        // Локальное обновление рангов (только для худшей и соседей)
                        pareto_ranks_[worst_idx] = offspring_rank;
                        
                        std::uniform_int_distribution<int> dist_idx(0, config_.population_size - 1);
                        for (int k = 0; k < 10; ++k) {
                            int random_idx = dist_idx(rng_);
                            if (random_idx != worst_idx) {
                                pareto_ranks_[random_idx] = computeRank(fitness_population_[random_idx]);
                            }
                        }
                    }
                }
            }
        }
        
        // Обновление всех рангов Парето
        updateParetoRanks();
        
        // Вызов колбэка поколения
        if (config_.on_generation_end) {
            float sum_fitness = 0.0f;
            int num_obj = config_.fitness_evaluator->getNumObjectives();
            for (int i = 0; i < config_.population_size; ++i) {
                if (num_obj > 0) {
                    sum_fitness += fitness_population_[i][num_obj - 1];
                }
            }
            float avg_fitness = config_.population_size > 0 
                ? sum_fitness / static_cast<float>(config_.population_size) 
                : 0.0f;
            config_.on_generation_end(generation, avg_fitness);
        }
    }
    
    // Выбор Парето-оптимальных решений
    updateParetoRanks();
    
    // Выбор лучшего Парето решения
    int best_idx = getBestParetoIndex();
    
    std::cout << "Best Pareto solution found at index: " << best_idx << std::endl;
    std::cout << "Fitness values: ";
    for (float f : fitness_population_[best_idx]) {
        std::cout << f << " ";
    }
    std::cout << std::endl;
    
    // Вызов финального колбэка
    if (config_.on_algorithm_end) {
        auto best_solution = config_.solution_factory();
        best_solution->decode(population_params_[best_idx], population_struct_[best_idx]);
        config_.on_algorithm_end(*best_solution);
    }
}

// src/GANOP.cpp
void GANOP::initializePopulation() {
    std::uniform_int_distribution<int> dist_bit(0, 1);
    
    auto temp_solution = config_.solution_factory();
    auto robot_sol = dynamic_cast<RobotSolution*>(temp_solution.get());
    
    if (!robot_sol) {
        throw std::runtime_error(
            "initializePopulation error: solution must be RobotSolution"
        );
    }

    NetOper& nop = robot_sol->getNetOper();
    
    // Инициализируем NetOper
    nop.setCs(qc);
    nop.setPsi(NopPsiN);
    auto robot_config = robot_sol->get_config();
    nop.setNodesForVars(robot_config.nodes_for_vars);
    nop.setNodesForParams(robot_config.nodes_for_params);
    nop.setNodesForOutput(robot_config.nodes_for_output);
    
    // === Первая особь ===
    // После setCs(qc), параметры NOP уже содержат qc
    // Теперь преобразуем их в Grey код для хромосомы
    vectorToGrey(population_params_[0], nop);  // ← Вот здесь!
    
    // Генерируем вариации структуры
    // for (int j = 0; j < config_.num_struct_variations; ++j) {
    //     nop.GenVar(population_struct_[0][j]);
    // }
    
    // === Остальная популяция ===
    for (int i = 1; i < config_.population_size; ++i) {
        // Генерируем вариации структуры
        for (int j = 0; j < config_.num_struct_variations; ++j) {
            nop.GenVar(population_struct_[i][j]);
        }
        
        // Генерируем случайные параметры (как в оригинале)
        for (int j = 0; j < static_cast<int>(population_params_[i].size()); ++j) {
            population_params_[i][j] = dist_bit(rng_);
        }
    }
}


// src/GANOP.cpp
void GANOP::vectorToGrey(std::vector<int>& grey_code, NetOper& nop) {
    std::vector<int> binary_code;           
    int bits_per_param = config_.int_bits + config_.frac_bits;
    
    // Получаем параметры из NOP (они уже установлены через setCs(qc))
    auto& params = nop.get_parameters();
    
    if (grey_code.size() < params.size() * bits_per_param) {
        grey_code.resize(params.size() * bits_per_param);
    }
    
    if (binary_code.size() < grey_code.size()) {
        binary_code.resize(grey_code.size());
    }
    
    std::fill(binary_code.begin(), binary_code.end(), 0);
    
    // Для каждого параметра преобразуем Float -> Binary
    for (size_t j = 0; j < params.size(); ++j) {
        float param = params[j];
        
        if (param < 0.0f) {
            param = std::abs(param);
        }
        
        int x = static_cast<int>(std::floor(param));  // целая часть
        double r = static_cast<double>(param - static_cast<float>(x));  // дробная часть
        
        // Целая часть (int_bits бит)
        int k = config_.int_bits + j * bits_per_param - 1;
        while (k >= static_cast<int>(j * bits_per_param)) {
            binary_code[k] = x % 2;
            x /= 2;
            k--;
        }
        
        // Дробная часть (frac_bits бит)
        k = config_.int_bits + j * bits_per_param;
        while (k < static_cast<int>((config_.int_bits + config_.frac_bits) * (j + 1))) {
            r *= 2.0;
            x = static_cast<int>(std::floor(r));
            binary_code[k] = x;
            r -= static_cast<float>(x);
            k++;
        }
        
        // Binary -> Grey код для этого параметра
        grey_code[j * bits_per_param] = binary_code[j * bits_per_param];
        for (int i = j * bits_per_param + 1; i < (j + 1) * bits_per_param; i++) {
            grey_code[i] = binary_code[i] ^ binary_code[i - 1];
        }
    }
}



// src/GANOP.cpp - новый метод
void GANOP::greyToVector(const std::vector<int>& grey_code, NetOper& nop) {
    if (grey_code.empty()) return;
    
    std::vector<int> binary_code(grey_code.size(), 0);
    int bits_per_param = config_.int_bits + config_.frac_bits;
    
    // Grey -> Binary
    for (size_t i = 0; i < grey_code.size(); ++i) {
        if (i % bits_per_param == 0) {
            binary_code[i] = grey_code[i];
        } else {
            binary_code[i] = binary_code[i - 1] ^ grey_code[i];
        }
    }
    
    // Binary -> Float
    auto& params = nop.get_parameters();
    params.clear();
    
    double g1 = std::pow(2.0, config_.int_bits - 1);
    
    for (int param_idx = 0; param_idx < config_.num_params; ++param_idx) {
        double value = 0.0;
        double g = g1;
        
        int start_bit = param_idx * bits_per_param;
        int end_bit = start_bit + config_.int_bits;
        
        if (end_bit > static_cast<int>(binary_code.size())) {
            break;
        }
        
        // Целая часть
        for (int i = start_bit; i < end_bit; ++i) {
            value += g * binary_code[i];
            g /= 2.0;
        }
        
        // Дробная часть
        int frac_end = std::min(start_bit + bits_per_param,
                                static_cast<int>(binary_code.size()));
        for (int i = end_bit; i < frac_end; ++i) {
            value += g * binary_code[i];
            g /= 2.0;
        }
        
        params.push_back(static_cast<float>(value));
    }
}


void GANOP::evaluatePopulation() {
    int num_obj = config_.fitness_evaluator->getNumObjectives();
    
    for (int i = 0; i < config_.population_size; ++i) {
        // Создаём решение из хромосомы
        auto solution = config_.solution_factory();
        solution->decode(population_params_[i], population_struct_[i]);
        
        // Вычисляем фитнесс
        auto fitness = config_.fitness_evaluator->evaluate(*solution);
        
        // Проверка корректности размера
        if (static_cast<int>(fitness.size()) != num_obj) {
            throw std::runtime_error(
                "Fitness function returned wrong number of objectives"
            );
        }
        
        fitness_population_[i] = fitness;
    }
}

void GANOP::updateParetoRanks() {
    // Вычисляем ранг для каждой особи (количество особей, которые её доминируют)
    for (int i = 0; i < config_.population_size; ++i) {
        pareto_ranks_[i] = computeRank(fitness_population_[i]);
    }
    
    // Выбираем Парето-оптимальные (ранг = 0)
    pareto_indices_.clear();
    for (int i = 0; i < config_.population_size; ++i) {
        if (pareto_ranks_[i] == 0) {
            pareto_indices_.push_back(i);
        }
    }
}

int GANOP::computeRank(const std::vector<float>& fitness) const {
    int num_obj = config_.fitness_evaluator->getNumObjectives();
    int domination_count = 0;
    
    for (int i = 0; i < config_.population_size; ++i) {
        // Проверяем, доминирует ли особь i над текущей (fitness)
        
        // Шаг 1: проверяем, пока текущая особь не хуже, чем i
        int j = 0;
        while (j < num_obj && fitness[j] >= fitness_population_[i][j]) {
            j++;
        }
        
        // Если все критерии прошли (j == num_obj)
        if (j >= num_obj) {
            // Шаг 2: проверяем, не все ли равны
            int k = 0;
            while (k < num_obj && fitness[k] == fitness_population_[i][k]) {
                k++;
            }
            
            // Если не все равны (k < num_obj) - особь i доминирует текущую
            if (k < num_obj) {
                domination_count++;
            }
        }
    }
    
    return domination_count;
}


void GANOP::selectParents(int& parent1_idx, int& parent2_idx) {
    std::uniform_int_distribution<int> dist_pop(0, config_.population_size - 1);
    
    // Выбираем первого родителя с поиском в соседстве
    parent1_idx = dist_pop(rng_);
    int best_rank = pareto_ranks_[parent1_idx];
    
    for (int i = 0; i < config_.search_neighbors; ++i) {
        int candidate = dist_pop(rng_);
        if (pareto_ranks_[candidate] < best_rank) {
            parent1_idx = candidate;
            best_rank = pareto_ranks_[candidate];
        }
    }
    
    // Второго родителя выбираем просто случайно
    parent2_idx = dist_pop(rng_);
}

void GANOP::crossover(int p1, int p2, 
                      std::vector<std::vector<int>>& offspring_params,
                      std::vector<std::vector<std::vector<int>>>& offspring_struct) {
    
    std::uniform_int_distribution<int> dist_bit(0, 1);
    std::uniform_int_distribution<int> dist_struct(0, config_.num_struct_variations - 1);
    std::uniform_int_distribution<int> dist_param(0, config_.num_params * 
                                                       (config_.int_bits + config_.frac_bits) - 1);
    
    int crossover_point_struct = dist_struct(rng_);
    int crossover_point_param = dist_param(rng_);
    
    int total_bits = config_.num_params * (config_.int_bits + config_.frac_bits);
    
    // Инициализация потомков
    for (int i = 0; i < 4; ++i) {
        offspring_params[i] = population_params_[i < 2 ? p1 : p2];
        offspring_struct[i].resize(config_.num_struct_variations);
        for (int j = 0; j < config_.num_struct_variations; ++j) {
            offspring_struct[i][j] = population_struct_[i < 2 ? p1 : p2][j];
        }
    }
    
    // Кроссовер параметров
    // Потомок 0 и 2: берут от p1 до точки, от p2 после
    for (int j = 0; j < crossover_point_param; ++j) {
        offspring_params[0][j] = population_params_[p1][j];
        offspring_params[2][j] = population_params_[p1][j];
    }
    for (int j = crossover_point_param; j < total_bits; ++j) {
        offspring_params[0][j] = population_params_[p2][j];
        offspring_params[2][j] = population_params_[p2][j];
    }
    
    // Потомок 1 и 3: противоположно
    for (int j = 0; j < crossover_point_param; ++j) {
        offspring_params[1][j] = population_params_[p2][j];
        offspring_params[3][j] = population_params_[p2][j];
    }
    for (int j = crossover_point_param; j < total_bits; ++j) {
        offspring_params[1][j] = population_params_[p1][j];
        offspring_params[3][j] = population_params_[p1][j];
    }
    
    // Кроссовер структур
    // Потомок 0 и 1: от p1 до точки, от p2 после
    for (int j = 0; j < crossover_point_struct; ++j) {
        offspring_struct[0][j] = population_struct_[p1][j];
        offspring_struct[1][j] = population_struct_[p1][j];
    }
    for (int j = crossover_point_struct; j < config_.num_struct_variations; ++j) {
        offspring_struct[0][j] = population_struct_[p2][j];
        offspring_struct[1][j] = population_struct_[p2][j];
    }
    
    // Потомок 2 и 3: противоположно
    for (int j = 0; j < crossover_point_struct; ++j) {
        offspring_struct[2][j] = population_struct_[p2][j];
        offspring_struct[3][j] = population_struct_[p2][j];
    }
    for (int j = crossover_point_struct; j < config_.num_struct_variations; ++j) {
        offspring_struct[2][j] = population_struct_[p1][j];
        offspring_struct[3][j] = population_struct_[p1][j];
    }
}

void GANOP::mutate(std::vector<int>& chromosome_params,
                   std::vector<std::vector<int>>& chromosome_struct) {
    
    std::uniform_int_distribution<int> dist_bit(0, 1);
    std::uniform_int_distribution<int> dist_struct(0, config_.num_struct_variations - 1);
    std::uniform_int_distribution<int> dist_param(0, static_cast<int>(chromosome_params.size()) - 1);
    
    // Мутация параметров (инвертирование случайного бита)
    int mutant_bit = dist_param(rng_);
    chromosome_params[mutant_bit] = 1 - chromosome_params[mutant_bit];
    
    // Мутация структуры (случайная регенерация одной вариации)
    int mutant_struct = dist_struct(rng_);
    for (auto& bit : chromosome_struct[mutant_struct]) {
        bit = dist_bit(rng_);
    }
}

int GANOP::getBestParetoIndex() const {
    if (pareto_indices_.empty()) {
        throw std::runtime_error("No Pareto solutions found!");
    }
    
    int num_obj = config_.fitness_evaluator->getNumObjectives();
    int best_idx = pareto_indices_[0];
    float best_value = fitness_population_[best_idx][num_obj - 1];
    
    for (int idx : pareto_indices_) {
        if (fitness_population_[idx][num_obj - 1] < best_value) {
            best_value = fitness_population_[idx][num_obj - 1];
            best_idx = idx;
        }
    }
    
    return best_idx;
}
