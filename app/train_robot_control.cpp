#include "GANOP.hpp"
#include "RobotFitnessEvaluator.hpp"
#include "base_solution.hpp"
#include "RobotProblemConfig.hpp"
#include "controller.hpp"
#include "runner.hpp"
#include "model.hpp"       
#include <iostream>
#include <fstream>


// Глобальная конфигурация для доступа в колбэке
RobotProblemConfig g_robot_config;
GAConfig g_ga_config;


void log_generation(int gen, float avg_fitness) {
    static std::ofstream log("evolution_log.txt", std::ios::app);
    log << "Generation " << gen << ": avg_fitness = " << avg_fitness << std::endl;
    std::cout << "Gen " << gen << " done, avg: " << avg_fitness << std::endl;
}


/**
 * @brief Проверить существует ли файл (без C++17)
 */
bool file_exists(const std::string& filename) {
    std::ifstream infile(filename);
    return infile.good();
}


/**
 * @brief Сохранить лучшее решение в файлы
 * Сохраняет матрицу и параметры для последующей загрузки
 */
void save_best_solution(const ISolution& solution) {
    const NetOper& net = solution.getNetOperConst();
    
    std::cout << "\n=== BEST SOLUTION FOUND ===" << std::endl;
    std::cout << "Best NOP matrix" << std::endl;
    net.printMatrix();

    // ✨ СОХРАНЯЕМ МАТРИЦУ И ПАРАМЕТРЫ
    auto& net_nonconst = const_cast<NetOper&>(net);
    
    if (!net_nonconst.saveMatrixToFile("best_matrix.txt")) {
        std::cerr << "WARNING: Failed to save best_matrix.txt" << std::endl;
    }
    
    if (!net_nonconst.saveParametersToFile("best_params.txt")) {
        std::cerr << "WARNING: Failed to save best_params.txt" << std::endl;
    }

    // Симулируем траектории и сохраняем
    std::ofstream outFile("trajectories.csv");
    if (!outFile.is_open()) {
        std::cerr << "Failed to open trajectories.csv for writing!" << std::endl;
        return;
    }
    
    outFile << "Trajectory,Time,X,Y,Theta\n";
    
    Model::State currState = {0.0f, 0.0f, 0.0f};
    Model model(currState, g_robot_config.dt, g_robot_config.model_path);
    Model::State goal = {0.0f, 0.0f, 0.0f};
    
    Controller controller(goal, net_nonconst);
    Runner runner(model, controller);
    runner.setGoal(goal);
    
    std::vector<Model::State> test_states = g_robot_config.generateTestTrajectories();
    
    std::cout << "Simulating " << test_states.size() << " test trajectories..." << std::endl;
    
    // Симулируем каждую траекторию
    for (size_t i = 0; i < test_states.size(); ++i) {
        runner.init(test_states[i]);
        float currTime = 0.0f;
        
        while (currTime < g_robot_config.time_limit) {
            currState = runner.makeStep();
            outFile << i << "," << currTime << ","
                   << currState.x << "," << currState.y << ","
                   << currState.yaw << "\n";
            
            currTime += g_robot_config.dt;
            
            if (currState.dist(goal) < g_robot_config.epsilon_term) {
                break;
            }
        }
    }
    
    outFile.close();
    std::cout << "Trajectories logged to trajectories.csv (" << test_states.size() << " trajectories)" << std::endl;
    
    // Вывод параметров сети
    std::cout << "Parameters: ";
    for (float p : net_nonconst.get_parameters()) {
        std::cout << p << " ";
    }
    std::cout << std::endl;
}


int main() {
    // === 1. Конфигурация проблемы ===
    RobotProblemConfig robot_config;
    robot_config.dt = 0.033333f;
    robot_config.time_limit = 30.0f;
    robot_config.epsilon_term = 0.1f;
    
    robot_config.num_trajectories = 64;
    robot_config.num_test_trajectories = 64;  
    
    robot_config.qyminc = {-5.5f, -5.5f, -1.31f};
    robot_config.qymaxc = {5.5f, 5.5f, 1.31f};
    robot_config.model_path = "rosbot_gazebo9_2d_model.onnx";
    
    // === 2. Конфигурация GA ===
    GAConfig ga_config;
    
    ga_config.nodes_for_vars = robot_config.nodes_for_vars;
    ga_config.nodes_for_params = robot_config.nodes_for_params;
    ga_config.nodes_for_output = robot_config.nodes_for_output;
    
    // Параметры GA
    ga_config.population_size = 500;
    ga_config.num_generations = 1;
    ga_config.num_crossovers_per_gen = 24;
    ga_config.mutation_prob = 0.5f;
    ga_config.selection_alpha = 0.5f;
    ga_config.search_neighbors = 64;
    ga_config.int_bits = 16;
    ga_config.frac_bits = 16;
    ga_config.num_params = 8;
    ga_config.num_struct_variations = 15;
    ga_config.seed = 69;
    
    // Инициализируем шаблон один раз
    ga_config.nop_template = std::make_shared<NetOper>();
    ga_config.nop_template->setNodesForVars(robot_config.nodes_for_vars);
    ga_config.nop_template->setNodesForParams(robot_config.nodes_for_params);
    ga_config.nop_template->setNodesForOutput(robot_config.nodes_for_output);
    
    // ✨ ЗАГРУЖАЕМ СОХРАНЁННУЮ МАТРИЦУ И ПАРАМЕТРЫ ИЛИ ИСПОЛЬЗУЕМ БАЗОВЫЕ
    std::cout << "\n=== LOADING NETWORK STATE ===" << std::endl;
    
    bool loaded_from_file = false;
    
    // Проверяем существуют ли файлы сохранённого состояния (без C++17)
    if (file_exists("best_matrix.txt") && file_exists("best_params.txt")) {
        std::cout << "Found saved network state files!" << std::endl;
        
        if (ga_config.nop_template->loadMatrixFromFile("best_matrix.txt") &&
            ga_config.nop_template->loadParametersFromFile("best_params.txt")) {
            std::cout << "✓ Successfully loaded network from best_matrix.txt and best_params.txt" << std::endl;
            loaded_from_file = true;
        } else {
            std::cerr << "WARNING: Failed to load network state, using base configuration" << std::endl;
        }
    } else {
        std::cout << "No saved network state found, using base configuration" << std::endl;
    }
    
    // Если не загрузили из файла, используем базовые параметры
    if (!loaded_from_file) {
        std::cout << "Loading base configuration..." << std::endl;
        ga_config.nop_template->setCs(robot_config.base_params);
        ga_config.nop_template->setPsi(robot_config.base_matrix);
        std::cout << "✓ Base network configuration loaded" << std::endl;
    }
    
    std::cout << "NetOper template initialized" << std::endl;

    // Сохраняем конфиги глобально для доступа в колбэках
    g_robot_config = robot_config;
    g_ga_config = ga_config;
    
    // === 3. Инъекция зависимостей ===
    ga_config.fitness_evaluator = std::make_shared<RobotFitnessEvaluator>(robot_config, 4);
    
    // Factory для создания решений с использованием BaseSolution
    ga_config.solution_factory = [robot_config, &ga_config]() -> std::unique_ptr<ISolution> {
        auto solution = std::make_unique<BaseSolution<RobotProblemConfig>>(robot_config);
        solution->setIntBits(ga_config.int_bits);
        solution->setFracBits(ga_config.frac_bits);
        return solution;
    };
    
    ga_config.on_generation_end = log_generation;
    ga_config.on_algorithm_end = save_best_solution;
    
    // === 4. Запуск GA ===
    std::cout << "\n=== STARTING GENETIC ALGORITHM ===" << std::endl;
    std::cout << "Population: " << ga_config.population_size << std::endl;
    std::cout << "Generations: " << ga_config.num_generations << std::endl;
    std::cout << "Training trajectories: " << robot_config.num_trajectories << std::endl;
    std::cout << "Test trajectories: " << robot_config.num_test_trajectories << std::endl;
    
    try {
        GANOP ga(ga_config);
        ga.run();
        
        std::cout << "\n=== GA COMPLETED SUCCESSFULLY ===" << std::endl;
        std::cout << "Results saved to:" << std::endl;
        std::cout << "  - best_matrix.txt" << std::endl;
        std::cout << "  - best_params.txt" << std::endl;
        std::cout << "  - trajectories.csv" << std::endl;
        std::cout << "  - evolution_log.txt" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
