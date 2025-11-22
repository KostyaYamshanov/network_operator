#include "GANOP.hpp"
#include "base_solution.hpp"
#include "simple_config.hpp"
#include "simple_fitness_evaluator.hpp"     
#include <iostream>
#include <fstream>
#include "nop.hpp"


// Глобальная конфигурация для доступа в колбэке
SimpleConfig g_simple_config;
GAConfig g_ga_config;


void log_generation(int gen, float avg_fitness) {
    static std::ofstream log("evolution_log.txt", std::ios::app);
    log << "Generation " << gen << ": avg_fitness = " << avg_fitness << std::endl;
    std::cout << "Gen " << gen << " done, avg: " << avg_fitness << std::endl;
}


void saveResultsToCSV(const std::vector<float>& y_expected, 
                     const std::vector<float>& y_nop,
                     float rmse) {
    std::ofstream csv("results.csv");
    if (!csv.is_open()) {
        std::cerr << "Failed to open results.csv" << std::endl;
        return;
    }
    
    // Заголовок
    csv << "x,target,nop_output,error" << std::endl;
    
    float x = g_simple_config.x_start;
    for (size_t i = 0; i < y_expected.size(); ++i) {
        float error = y_nop[i] - y_expected[i];
        csv << x << "," 
            << y_expected[i] << "," 
            << y_nop[i] << "," 
            << error << std::endl;
        x += g_simple_config.x_step;
    }
    
    csv.close();
    std::cout << "\n=== RESULTS ===" << std::endl;
    std::cout << "Overall RMSE: " << rmse << std::endl;
    std::cout << "Results saved to results.csv" << std::endl;
}


void save_best_solution(const ISolution& solution) {
    try {
        const auto& base_sol = dynamic_cast<const BaseSolution<SimpleConfig>&>(solution);
        auto fitness_evaluator = SimpleFitnessEvaluator(g_simple_config, 1);
        const NetOper& nop = base_sol.getNetOperConst();
        
        std::cout << "\n=== BEST SOLUTION FOUND ===" << std::endl;
        nop.printMatrix();
        // Целевые значения
        auto y_expected = fitness_evaluator.getTargetValues();
        
        // Вычисляем значения NOP
        std::vector<float> y_nop;
        float x = g_simple_config.x_start;
        
        for (int i = 0; i < g_simple_config.num_samples; ++i) {
            std::vector<float> output(1, 0.0f);
            const_cast<NetOper&>(nop).calcResult({x}, output);
            
            if (!output.empty()) {
                y_nop.push_back(output[0]);
            } else {
                y_nop.push_back(0.0f);
            }
            x += g_simple_config.x_step;
        }
        
        // Вычисляем RMSE
        float rmse = fitness_evaluator.computeRMSE(y_expected, y_nop);
        
        std::cout << "Target function: sin(x) + " << g_simple_config.free_param << "*cos(x)" << std::endl;
        std::cout << "RMSE: " << rmse << std::endl;
        
        // Сохраняем в CSV
        saveResultsToCSV(y_expected, y_nop, rmse);
        
    } catch (const std::exception& e) {
        std::cerr << "Error saving best solution: " << e.what() << std::endl;
    }
}




int main() {
    // === 1. Конфигурация проблемы ===
    SimpleConfig simple_config;
    
    
    // === 2. Конфигурация GA ===
    GAConfig ga_config;
    
    // Копируем данные сети из SimpleConfig
    ga_config.nodes_for_vars = simple_config.nodes_for_vars;
    ga_config.nodes_for_params = simple_config.nodes_for_params;
    ga_config.nodes_for_output = simple_config.nodes_for_output;
    
    // Параметры GA
    ga_config.population_size = 1000;
    ga_config.num_generations = 128;
    ga_config.num_params = 2;
    ga_config.int_bits = 4;
    ga_config.frac_bits = 8;
    ga_config.num_struct_variations = 10;
    ga_config.seed = 42;
    
    // Инициализируем шаблон один раз
    ga_config.nop_template = std::make_shared<NetOper>();
    ga_config.nop_template->setNodesForVars(simple_config.nodes_for_vars);
    ga_config.nop_template->setNodesForParams(simple_config.nodes_for_params);
    ga_config.nop_template->setNodesForOutput(simple_config.nodes_for_output);
    ga_config.nop_template->setCs(simple_config.base_params);
    ga_config.nop_template->setPsi(simple_config.base_matrix);
    
    std::cout << "NetOper template created once" << std::endl;
    
    // Сохраняем конфиги глобально для доступа в колбэках
    g_simple_config = simple_config;
    g_ga_config = ga_config;
    
    // === 3. Инъекция зависимостей ===
    int num_objectives = 1; // кол-во функционалов
    ga_config.fitness_evaluator = std::make_shared<SimpleFitnessEvaluator>(simple_config, num_objectives);
    
    // Factory для создания решений с использованием BaseSolution
    ga_config.solution_factory = [simple_config, &ga_config]() -> std::unique_ptr<ISolution> {
        auto solution = std::make_unique<BaseSolution<SimpleConfig>>(simple_config);
        // Устанавливаем биты из GA конфига
        solution->setIntBits(ga_config.int_bits);
        solution->setFracBits(ga_config.frac_bits);
        return solution;
    };
    
    ga_config.on_generation_end = log_generation;
    ga_config.on_algorithm_end = save_best_solution;
    
    // === 4. Запуск GA ===
    try {
        GANOP ga(ga_config);
        ga.run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
