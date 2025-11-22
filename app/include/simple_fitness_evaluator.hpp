/**
 * @file SimpleFitnessEvaluator.hpp
 * @brief Evaluator для простой задачи оптимизации
 * 
 * Работает с любым ISolution через универсальный интерфейс
 */

#pragma once
#include "base_evaluator.hpp"
#include "simple_config.hpp"
#include <vector>
#include <cmath>
#include <iostream>


class SimpleFitnessEvaluator : public BaseFitnessEvaluator {
public:
    SimpleConfig config_;

    explicit SimpleFitnessEvaluator(const SimpleConfig& config, int num_objectives = 1)
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
            const NetOper& nop = solution.getNetOperConst();
            
            // Получаем целевые значения
            std::vector<float> y_expected = getTargetValues();
            
            // Вычисляем текущие значения через NOP
            std::vector<float> y_current;
            float x = config_.x_start;
            
            // std::cout << "Computing NOP output..." << std::endl;
            
            for (int i = 0; i < config_.num_samples; ++i) {
                std::vector<float> output(1, 0.0f);
                
                // Используем calcResult
                const_cast<NetOper&>(nop).calcResult({x}, output);
                
                if (!output.empty()) {
                    y_current.push_back(output[0]);
                } else {
                    std::cerr << "Error: NOP calcResult returned empty output at x=" << x << std::endl;
                    y_current.push_back(0.0f);
                }
                
                x += config_.x_step;
            }
            
            // Вычисляем RMSE - используем метод из базового класса
            float rmse = BaseFitnessEvaluator::computeRMSE(y_expected, y_current);
            
            // std::cout << "Evaluation complete. RMSE = " << rmse << std::endl;
            
            return {rmse};
            
        } catch (const std::exception& e) {
            std::cerr << "Error in SimpleFitnessEvaluator::evaluate(): " << e.what() << std::endl;
            return {1e9f};
        }
    }
        
    /// Целевая функция: f(x, q) = sin(x) + q*cos(x)
    static float targetFunction(float x, float q) {
        return std::sin(x) + q * std::cos(x);
    }

    
    /// Получение целевых значений
    std::vector<float> getTargetValues() const {
        std::vector<float> result;
        float x = config_.x_start;
        
        for (int i = 0; i < config_.num_samples; ++i) {
            float y_expected = targetFunction(x, config_.free_param);
            result.push_back(y_expected);
            x += config_.x_step;
        }
        
        return result;
    }
};