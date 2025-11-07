// GANOP.hpp
#pragma once
#include "GAConfig.hpp"
#include "nop.hpp"
#include "RobotSolution.hpp"
#include "isolution.hpp"
#include <vector>
#include <memory>

class GANOP {
public:
    explicit GANOP(const GAConfig& config);
    
    // Запуск алгоритма
    void run();
    
    // Получение результатов
    const std::vector<int>& getParetoIndices() const { return pareto_indices_; }
    int getBestParetoIndex() const;
    const std::vector<std::vector<float>>& getAllFitness() const { return fitness_population_; }
    
private:
    // === Внутренние методы GA ===
    void greyToVector(const std::vector<int>& grey_code, NetOper& nop);
    void vectorToGrey(std::vector<int>& grey_code, NetOper& nop);
    void initializePopulation();
    void evaluatePopulation();
    void updateParetoRanks();
    void selectParents(int& parent1_idx, int& parent2_idx);
    void crossover(int p1, int p2, std::vector<std::vector<int>>& offspring_params,
                   std::vector<std::vector<std::vector<int>>>& offspring_struct);
    void mutate(std::vector<int>& chromosome_params,
                std::vector<std::vector<int>>& chromosome_struct);
    int computeRank(const std::vector<float>& fitness) const;
    
    // === Члены класса ===
    GAConfig config_;
    
    // Популяция (хромосомы)
    std::vector<std::vector<int>> population_params_;           // [HH][bits]
    std::vector<std::vector<std::vector<int>>> population_struct_; // [HH][lchr][...]
    
    // Фитнесс и ранги
    std::vector<std::vector<float>> fitness_population_;  // [HH][num_objectives]
    std::vector<int> pareto_ranks_;                       // [HH]
    std::vector<int> pareto_indices_;                     // индексы Парето-оптимальных
    
    // Генератор случайных чисел
    std::mt19937 rng_;
};
