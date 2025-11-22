// RobotFitnessEvaluator.hpp
#pragma once
#include "ifitness_evaluator.hpp"
#include "RobotSolution.hpp"
#include "RobotProblemConfig.hpp"
#include "controller.hpp"  // <-- Добавить
#include "runner.hpp"      // <-- Добавить
#include "model.hpp"       // <-- Добавить
#include <vector>
#include <cmath>

class RobotFitnessEvaluator : public IFitnessEvaluator {
public:
    RobotFitnessEvaluator(const RobotProblemConfig& config, int num_objectives = 4)
        : config_(config), num_objectives_(num_objectives) {}
    
    // std::vector<float> evaluate(const ISolution& solution) override {
    //     const auto& robot_sol = dynamic_cast<const RobotSolution&>(solution);
        
    //     // Используем getNetOperConst() для const контекста
    //     const NetOper& net = robot_sol.getNetOperConst();
        
    //     Model::State currState = {0.0f, 0.0f, 0.0f};
    //     Model model(currState, config_.dt, config_.model_path);
    //     Model::State goal = {0.0f, 0.0f, 0.0f};
        
    //     // const_cast нужен, если Controller требует неконст NetOper
    //     Controller controller(goal, const_cast<NetOper&>(net));
    //     Runner runner(model, controller);
    //     runner.setGoal(goal);
        
    //     // Генерация начальных состояний
    //     std::vector<Model::State> init_states;
    //     for (int i = 0; i < config_.num_trajectories; ++i) {
    //         init_states.push_back(Model::State{
    //             (i & 4) ? config_.qymaxc[0] : config_.qyminc[0],
    //             (i & 2) ? config_.qymaxc[1] : config_.qyminc[1],
    //             (i & 1) ? config_.qymaxc[2] : config_.qyminc[2]
    //         });
    //     }
        
    //     float sumt = 0.0f, sumdelt = 0.0f, sum_path = 0.0f;
        
    //     // Симуляция траекторий
    //     for (int i = 0; i < config_.num_trajectories; ++i) {
    //         runner.init(init_states[i]);
    //         float currTime = 0.0f;
    //         float path_length = 0.0f;
    //         Model::State prevState = init_states[i];
            
    //         while (currTime < config_.time_limit) {
    //             currState = runner.makeStep();
    //             float dx = currState.x - prevState.x;
    //             float dy = currState.y - prevState.y;
    //             path_length += std::sqrt(dx * dx + dy * dy);
    //             prevState = currState;
    //             currTime += config_.dt;
                
    //             if (currState.dist(goal) < config_.epsilon_term)
    //                 break;
    //         }
    //         sumt += currTime;
    //         sumdelt += currState.dist(goal);
    //         sum_path += path_length;
    //     }
        
    //     // Возвращаем вектор критериев
    //     return {
    //         sumt,
    //         sumdelt * 2.0f,
    //         sum_path,
    //         sum_path * 1.5f + sumt + sumdelt * 7.0f
    //     };
    // }


    std::vector<float> evaluate(const ISolution& solution) override {
        const auto& robot_sol = dynamic_cast<const RobotSolution&>(solution);
        const NetOper& net = robot_sol.getNetOperConst();
        
        Model::State currState = {0.0f, 0.0f, 0.0f};
        Model model(currState, config_.dt, config_.model_path);
        Model::State goal = {0.0f, 0.0f, 0.0f};
        
        Controller controller(goal, const_cast<NetOper&>(net));
        Runner runner(model, controller);
        runner.setGoal(goal);
        
        std::vector<Model::State> init_states;
        for (int i = 0; i < config_.num_trajectories; ++i) {
            init_states.push_back(Model::State{
                (i & 4) ? config_.qymaxc[0] : config_.qyminc[0],
                (i & 2) ? config_.qymaxc[1] : config_.qyminc[1],
                (i & 1) ? config_.qymaxc[2] : config_.qyminc[2]
            });
        }
        
        float total_time = 0.0f;
        float total_error = 0.0f;
        float total_path = 0.0f;
        float total_smoothness = 0.0f;
        int successes = 0;
        
        for (const auto& init_state : init_states) {
            runner.init(init_state);
            float curr_time = 0.0f;
            float path_length = 0.0f;
            
            Model::State prev_state = init_state;
            Model::State prev_vel = {0, 0, 0};
            
            while (curr_time < config_.time_limit) {
                currState = runner.makeStep();
                
                // Расстояние и путь
                float dx = currState.x - prev_state.x;
                float dy = currState.y - prev_state.y;
                path_length += std::sqrt(dx * dx + dy * dy);
                
                // Гладкость (штраф за ускорение)
                float vx = dx / config_.dt;
                float vy = dy / config_.dt;
                float ax = (vx - prev_vel.x) / config_.dt;
                float ay = (vy - prev_vel.y) / config_.dt;
                total_smoothness += std::sqrt(ax*ax + ay*ay) * 0.1f;
                
                prev_state = currState;
                prev_vel = {vx, vy, 0};
                curr_time += config_.dt;
                
                if (currState.dist(goal) < config_.epsilon_term) {
                    successes++;
                    break;
                }
            }
            
            total_time += curr_time;
            total_error += currState.dist(goal);
            total_path += path_length;
        }
        
        // Штраф за неудачи
        float failure_penalty = (config_.num_trajectories - successes) * 100.0f;
        
        return {
            total_time,
            total_error * 2.0f + failure_penalty,
            total_path,
            total_smoothness + total_path * 1.5f + total_time + total_error * 3.0f + failure_penalty
        };
    }

    
    int getNumObjectives() const override { return num_objectives_; }
    
private:
    RobotProblemConfig config_;
    int num_objectives_;
};
