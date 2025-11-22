// SimpleFitnessEvaluator.hpp
#pragma once
#include "ifitness_evaluator.hpp"
#include "simple_solution.hpp"
#include "simple_config.hpp"
#include <vector>
#include <cmath>
#include <iostream>

class SimpleFitnessEvaluator : public IFitnessEvaluator {
public:
    SimpleFitnessEvaluator(const SimpleConfig& config, int num_objectives = 1)
        : config_(config), num_objectives_(num_objectives) {}
    
    std::vector<float> evaluate(const ISolution& solution) override {
        try {
            const auto& simple_sol = dynamic_cast<const SimpleSolution&>(solution);
            const NetOper& nop = simple_sol.getNetOperConst();
            
            // Получаем целевые значения
            std::vector<float> y_expected = getTargetValues();
            
            // Вычисляем текущие значения через NOP
            std::vector<float> y_current;
            float x = config_.x_start;
            
            std::cout << "Computing NOP output..." << std::endl;
            
            for (int i = 0; i < config_.num_samples; ++i) {
                std::vector<float> output(1, 0.0f);  // Один выход
                
                // Используем calcResult вместо прямого вычисления
                const_cast<NetOper&>(nop).calcResult({x}, output);
                
                if (!output.empty()) {
                    y_current.push_back(output[0]);
                } else {
                    std::cerr << "Error: NOP calcResult returned empty output at x=" << x << std::endl;
                    y_current.push_back(0.0f);
                }
                
                x += config_.x_step;
            }
            
            // Вычисляем RMSE
            float rmse = computeRMSE(y_expected, y_current);
            
            std::cout << "Evaluation complete. RMSE = " << rmse << std::endl;
            
            return {rmse};
            
        } catch (const std::bad_cast& e) {
            std::cerr << "Bad cast in SimpleFitnessEvaluator: " << e.what() << std::endl;
            return {1e9f};
        } catch (const std::exception& e) {
            std::cerr << "Error in evaluate(): " << e.what() << std::endl;
            return {1e9f};
        }
    }
    
    int getNumObjectives() const override { return num_objectives_; }
    
    static float computeRMSE(const std::vector<float>& y_out, 
                            const std::vector<float>& y_ref) {
        if (y_out.size() != y_ref.size()) {
            throw std::invalid_argument("Vectors must have the same size");
        }
        
        float sum_squares = 0.0f;
        for (size_t i = 0; i < y_out.size(); ++i) {
            float diff = y_out[i] - y_ref[i];
            sum_squares += diff * diff;
        }
        
        return std::sqrt(sum_squares / y_out.size());
    }

    
    // Целевая функция: f(x, q) = sin(x) + q*cos(x)
    static float targetFunction(float x, float q) {
        return std::sin(x) + q * std::cos(x);
    }

    // Получение целевых значений
    std::vector<float> getTargetValues() {
        std::vector<float> result;
        float x = config_.x_start;
        
        for (int i = 0; i < config_.num_samples; ++i) {
            float y_expected = targetFunction(x, config_.free_param);
            result.push_back(y_expected);
            x += config_.x_step;
        }
        
        return result;
    }

private:
    SimpleConfig config_;
    int num_objectives_;
};
