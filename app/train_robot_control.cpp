#include "GANOP.hpp"
#include "RobotSolution.hpp"
#include "RobotFitnessEvaluator.hpp"
#include "controller.hpp"
#include "runner.hpp"
#include "model.hpp"       
#include <iostream>
#include <fstream>

// Глобальная конфигурация для доступа в колбэке
RobotProblemConfig g_robot_config;

void log_generation(int gen, float avg_fitness) {
    static std::ofstream log("evolution_log.txt", std::ios::app);
    log << "Generation " << gen << ": avg_fitness = " << avg_fitness << std::endl;
    std::cout << "Gen " << gen << " done, avg: " << avg_fitness << std::endl;
}

void save_best_solution(const ISolution& solution) {
    const auto& robot_sol = dynamic_cast<const RobotSolution&>(solution);
    const NetOper& net = robot_sol.getNetOperConst();
    
    std::cout<<"Best NOP matrix"<<std::endl;
    net.printMatrix();

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
    
    // const_cast если нужно
    Controller controller(goal, const_cast<NetOper&>(net));
    Runner runner(model, controller);
    runner.setGoal(goal);
    
    // Генерируем начальные состояния
    std::vector<Model::State> init_states;
    for (int i = 0; i < g_robot_config.num_trajectories; ++i) {
        init_states.push_back(Model::State{
            (i & 4) ? g_robot_config.qymaxc[0] : g_robot_config.qyminc[0],
            (i & 2) ? g_robot_config.qymaxc[1] : g_robot_config.qyminc[1],
            (i & 1) ? g_robot_config.qymaxc[2] : g_robot_config.qyminc[2]
        });
    }
    
    // Симулируем каждую траекторию
    for (int i = 0; i < g_robot_config.num_trajectories; ++i) {
        runner.init(init_states[i]);
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
    std::cout << "Trajectories logged to trajectories.csv" << std::endl;
    
    // Вывод параметров сети
    net.printMatrix();
    
    // Используем const_cast для получения параметров
    auto& net_nonconst = const_cast<NetOper&>(net);
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
    robot_config.time_limit = 8.0f;
    robot_config.epsilon_term = 0.1f;
    robot_config.num_trajectories = 8;
    robot_config.qyminc = {-2.5f, -2.5f, -1.31f};
    robot_config.qymaxc = {2.5f, 2.5f, 1.31f};
    robot_config.model_path = "rosbot_gazebo9_2d_model.onnx";
    robot_config.base_matrix = NopPsiN;
    // robot_config.base_params = {6703.02, 20833.6, 39213.1, 51080.3};

    robot_config.base_params = {4.02, 3.6, 2.1, 1.3};
    
    
    GAConfig ga_config;
    
    ga_config.nodes_for_vars = robot_config.nodes_for_vars;
    ga_config.nodes_for_params = robot_config.nodes_for_params;
    ga_config.nodes_for_output = robot_config.nodes_for_output;
    
    // Инициализируем шаблон один раз
    ga_config.nop_template = std::make_shared<NetOper>();
    ga_config.nop_template->setNodesForVars(robot_config.nodes_for_vars);
    ga_config.nop_template->setNodesForParams(robot_config.nodes_for_params);
    ga_config.nop_template->setNodesForOutput(robot_config.nodes_for_output);
    ga_config.nop_template->setCs(robot_config.base_params);
    ga_config.nop_template->setPsi(robot_config.base_matrix);
    
    std::cout << "NetOper template created once" << std::endl;

    // Сохраняем глобально для доступа в колбэке
    g_robot_config = robot_config;
    
    // === 2. Конфигурация GA ===
    // GAConfig ga_config;
    ga_config.population_size = 500;
    ga_config.num_generations = 16;
    ga_config.num_crossovers_per_gen = 24;
    ga_config.mutation_prob = 0.5f;
    ga_config.selection_alpha = 0.5f;
    ga_config.search_neighbors = 64;
    ga_config.int_bits = 16;
    ga_config.frac_bits = 16;
    ga_config.num_params = 4;
    ga_config.num_struct_variations = 15;
    ga_config.seed = 69;
    
    // === 3. Инъекция зависимостей ===
    ga_config.fitness_evaluator = std::make_shared<RobotFitnessEvaluator>(robot_config, 4);
    
    ga_config.solution_factory = [robot_config]() -> std::unique_ptr<ISolution> {
        return std::make_unique<RobotSolution>(robot_config);
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
