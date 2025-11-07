// GAConfig.hpp
#pragma once
#include <functional>
#include <memory>
#include "ifitness_evaluator.hpp"
#include "isolution.hpp"
#include <random>

struct GAConfig {
    // === Параметры GA (универсальные) ===
    int population_size = 500;
    int num_generations = 16;
    int num_crossovers_per_gen = 24;
    float mutation_prob = 0.5f;
    float selection_alpha = 0.5f;
    int search_neighbors = 8;
    uint32_t seed = std::mt19937::default_seed;
    
    // === Кодирование хромосом ===
    int num_params = 4;      // m_p
    int int_bits = 16;
    int frac_bits = 16;
    int num_struct_variations = 10;
    
    // === Интерфейсы (инъекция зависимостей) ===
    std::shared_ptr<IFitnessEvaluator> fitness_evaluator;
    std::function<std::unique_ptr<ISolution>()> solution_factory;
    
    // === Колбэки ===
    std::function<void(int gen, float avg_fitness)> on_generation_end = nullptr;
    std::function<void(const ISolution& best_solution)> on_algorithm_end = nullptr;
};
