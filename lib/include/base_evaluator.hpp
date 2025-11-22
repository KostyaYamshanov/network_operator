/**
 * @file BaseFitnessEvaluator.hpp
 * @brief Базовый класс для всех evaluator'ов
 * 
 * Работает с универсальным ISolution интерфейсом,
 * не привязан к конкретным типам решений
 */


#pragma once
#include "ifitness_evaluator.hpp"
#include "base_config.hpp"
#include "nop.hpp"
#include <vector>
#include <cmath>
#include <iostream>



/**
 * @class BaseFitnessEvaluator
 * @brief Базовый evaluator для всех типов задач
 * 
 * Предоставляет:
 * - Работу с ISolution интерфейсом (без привязки к конкретным типам)
 * - Вспомогательные методы (computeRMSE и т.д.)
 * - Доступ к NetOper через универсальный интерфейс
 */
class BaseFitnessEvaluator : public IFitnessEvaluator {
public:
    explicit BaseFitnessEvaluator(int num_objectives = 1)
        : num_objectives_(num_objectives) {}
    
    virtual ~BaseFitnessEvaluator() = default;
    
    
    /**
     * @brief Вычисление критериев
     * 
     * Реализуется в наследниках для каждой конкретной задачи
     */
    virtual std::vector<float> evaluate(const ISolution& solution) = 0;
    
    
    /**
     * @brief Получить размерность пространства критериев
     */
    int getNumObjectives() const override { 
        return num_objectives_; 
    }
    
    /**
     * @brief Вычисление RMSE между двумя векторами
     */
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
    

protected:
    int num_objectives_;
    
    
    /**
     * @brief Получить NetOper из решения через const cast
     * 
     * Нужен const_cast т.к. интерфейс ISolution::evaluate() const,
     * но NetOper::calcResult() требует non-const
     */
    // static NetOper& getNetOperFromSolution(const ISolution& solution) {
    //     // Обращаемся к методам через интерфейс ISolution
    //     // Это HACK - но в реальности нужно добавить метод в ISolution
    //     // или использовать RTTI с dynamic_cast
    //     return const_cast<NetOper&>(
    //         *reinterpret_cast<const NetOper*>(nullptr)  // Placeholder
    //     );
    // }
};

