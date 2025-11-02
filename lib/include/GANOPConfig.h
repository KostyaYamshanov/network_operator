#pragma once
#include <vector>
#include <string>
#include <functional>
#include <cstdint>
#include <random>  // Для default seed

#include "nop.hpp"

using TArrInt = std::vector<int>;
using TArrReal = std::vector<float>;

struct GANOPConfig {
    // GA params
    int num_params = 4;      
    int int_bits = 16;       
    int frac_bits = 16;      
    int num_struct_variations = 10;  
    int population_size = 500;       
    int num_objectives = 4;   // nfu       
    int num_generations = 16;        
    int num_crossovers_per_gen = 24; 
    float mutation_prob = 0.5f;      
    float selection_alpha = 0.5f;    
    int search_neighbors = 8;        
    uint32_t seed = std::mt19937::default_seed;  // Для воспроизводимости

    // Network topology
    std::vector<int> nodes_for_vars = {0, 1, 2};
    std::vector<int> nodes_for_params = {3, 4, 5, 6};
    std::vector<int> nodes_for_output = {22, 23};

    // Simulation params (for default fitness)
    float dt = 0.033333f;
    float time_limit = 15.0f;
    float epsilon_term = 0.1f;
    int num_trajectories = 8;
    std::vector<float> qyminc = {-3.5f, -3.5f, -1.31f};
    std::vector<float> qymaxc = {3.5f, 3.5f, 1.31f};
    std::string model_path = "rosbot_gazebo9_2d_model.onnx";

    // Logging
    std::string output_csv = "trajectories.csv";
    bool enable_logging = true;

    // Изменённый тип для фитнесса
    std::function<void(TArrReal&, NetOper&)> fitness_func = nullptr;

    // Колбэк (без изменений)
    std::function<void(int, float)> on_generation_end = nullptr;
};