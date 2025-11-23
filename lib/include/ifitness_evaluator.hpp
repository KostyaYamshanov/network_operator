#pragma once
#include "isolution.hpp"
#include <vector>

class IFitnessEvaluator {
public:
    virtual ~IFitnessEvaluator() = default;
    
    // Вычисляет вектор критериев для данного решения
    virtual std::vector<float> evaluate(const ISolution& solution) = 0;
    
    // Размерность пространства критериев
    virtual int getNumObjectives() const = 0;
};