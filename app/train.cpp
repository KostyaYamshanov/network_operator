#include "ganop.hpp"
#include <iostream>
#include <fstream>

// Пример колбэка: логирует в файл
void my_callback(int gen, float avg_fitness) {
    std::ofstream log("evolution_log.txt", std::ios::app);
    log << "Generation " << gen << ": avg_fitness = " << avg_fitness << std::endl;
    log.close();
    std::cout << "Gen " << gen << " done, avg: " << avg_fitness << std::endl;
}


int main() {
    GANOPConfig config; 

    config.on_generation_end = my_callback;

    auto robot_fitness = [&config](TArrReal& Fu, NetOper& net) -> void {
        Model::State currState = {0.0f, 0.0f, 0.0f}; 
        Model model(currState, config.dt, config.model_path);
        
        Model::State goal = {0.0f, 0.0f, 0.0f};
        Controller controller(goal, net);
        Runner runner(model, controller); 
        runner.setGoal(goal);

        std::vector<Model::State> init_states; 
        int nGraphc = config.num_trajectories;
        
        for (int i = 0; i < nGraphc; ++i) {
            init_states.push_back(
                Model::State{   (i & 4) ? config.qymaxc[0] : config.qyminc[0], 
                                (i & 2) ? config.qymaxc[1] : config.qyminc[1], 
                                (i & 1) ? config.qymaxc[2] : config.qyminc[2] }
                );
        }

        float sumt = 0.0f;
        float sumdelt = 0.0f;
        float sum_path = 0.0f;

        for (int i = 0; i < nGraphc; ++i) {
            runner.init(init_states[i]);
            float currTime = 0.0f;
            float path_length = 0.0f;
            Model::State prevState = init_states[i];
            while (currTime < config.time_limit) {
                currState = runner.makeStep();
                float dx = currState.x - prevState.x;
                float dy = currState.y - prevState.y;
                path_length += std::sqrt(dx * dx + dy * dy);
                prevState = currState;
                currTime += config.dt;
                if (currState.dist(goal) < config.epsilon_term)
                    break; 
            }
            sumt += currTime;
            sumdelt += currState.dist(goal);
            sum_path += path_length;
        }

        Fu[0] = sumt;
        Fu[1] = sumdelt * 2.0f;
        Fu[2] = sum_path;
        Fu[3] = sum_path * 1.5f + sumt + sumdelt * 7.0f;
    };

    config.fitness_func = robot_fitness;
    GANOP ga(config);
    ga.GenAlgorithm();
    return 0;
}