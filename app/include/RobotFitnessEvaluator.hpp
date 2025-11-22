/**
 * @file RobotFitnessEvaluator.hpp
 * @brief Evaluator для задачи робота
 * 
 * Работает с любым ISolution через универсальный интерфейс
 */

#pragma once
#include "base_evaluator.hpp"
#include "RobotProblemConfig.hpp"
#include "controller.hpp"
#include "runner.hpp"
#include "model.hpp"
#include <vector>
#include <cmath>


class RobotFitnessEvaluator : public BaseFitnessEvaluator {
public:
    explicit RobotFitnessEvaluator(const RobotProblemConfig& config, int num_objectives = 4)
        : BaseFitnessEvaluator(num_objectives), config_(config) {}
    
    
    /**
     * @brief Вычисление фитнеса
     * 
     * ✨ Больше нет dynamic_cast!
     * ✨ Работает с любым ISolution
     */
    std::vector<float> evaluate(const ISolution& solution) override {
        try {
            // Используем интерфейс ISolution напрямую - никакого dynamic_cast!
            const NetOper& net = solution.getNetOperConst();
            
            Model::State currState = {0.0f, 0.0f, 0.0f};
            Model model(currState, config_.dt, config_.model_path);
            Model::State goal = {0.0f, 0.0f, 0.0f};
            
            Controller controller(goal, const_cast<NetOper&>(net));
            Runner runner(model, controller);
            runner.setGoal(goal);
            
            // ✨ Используем метод конфига для генерации траекторий
            std::vector<Model::State> init_states = config_.generateTrainTrajectories();
            
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
            
        } catch (const std::exception& e) {
            std::cerr << "Error in RobotFitnessEvaluator::evaluate(): " << e.what() << std::endl;
            return {1e9f, 1e9f, 1e9f, 1e9f};
        }
    }
    
    
private:
    RobotProblemConfig config_;
};
