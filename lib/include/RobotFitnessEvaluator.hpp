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
    
    std::vector<float> evaluate(const ISolution& solution) override {
        const auto& robot_sol = dynamic_cast<const RobotSolution&>(solution);
        
        // Используем getNetOperConst() для const контекста
        const NetOper& net = robot_sol.getNetOperConst();
        
        Model::State currState = {0.0f, 0.0f, 0.0f};
        Model model(currState, config_.dt, config_.model_path);
        Model::State goal = {0.0f, 0.0f, 0.0f};
        
        // const_cast нужен, если Controller требует неконст NetOper
        Controller controller(goal, const_cast<NetOper&>(net));
        Runner runner(model, controller);
        runner.setGoal(goal);
        
        // Генерация начальных состояний
        std::vector<Model::State> init_states;
        for (int i = 0; i < config_.num_trajectories; ++i) {
            init_states.push_back(Model::State{
                (i & 4) ? config_.qymaxc[0] : config_.qyminc[0],
                (i & 2) ? config_.qymaxc[1] : config_.qyminc[1],
                (i & 1) ? config_.qymaxc[2] : config_.qyminc[2]
            });
        }
        
        float sumt = 0.0f, sumdelt = 0.0f, sum_path = 0.0f;
        
        // Симуляция траекторий
        for (int i = 0; i < config_.num_trajectories; ++i) {
            runner.init(init_states[i]);
            float currTime = 0.0f;
            float path_length = 0.0f;
            Model::State prevState = init_states[i];
            
            while (currTime < config_.time_limit) {
                currState = runner.makeStep();
                float dx = currState.x - prevState.x;
                float dy = currState.y - prevState.y;
                path_length += std::sqrt(dx * dx + dy * dy);
                prevState = currState;
                currTime += config_.dt;
                
                if (currState.dist(goal) < config_.epsilon_term)
                    break;
            }
            sumt += currTime;
            sumdelt += currState.dist(goal);
            sum_path += path_length;
        }
        
        // Возвращаем вектор критериев
        return {
            sumt,
            sumdelt * 2.0f,
            sum_path,
            sum_path * 1.5f + sumt + sumdelt * 7.0f
        };
    }
    
    int getNumObjectives() const override { return num_objectives_; }
    
private:
    RobotProblemConfig config_;
    int num_objectives_;
};
