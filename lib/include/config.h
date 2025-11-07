#pragma once
#include <vector>
#include <string>
#include <cstdint>

struct GANOPConfig {
    // GA params
    int num_params = 4;      // m_p
    int int_bits = 16;       // m_c
    int frac_bits = 16;      // m_d
    int num_struct_variations = 10;  // m_lchr
    int population_size = 500;       // m_HH
    int num_objectives = 4;          // m_nfu
    int num_generations = 16;        // m_PP
    int num_crossovers_per_gen = 24; // m_RR
    float mutation_prob = 0.5f;      // pmut
    float selection_alpha = 0.5f;    // alfa
    int search_neighbors = 8;        // ksearch

    // Simulation params (for fitness)
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

    // Callbacks (для прогресса)
    std::function<void(int gen, float avg_fitness)> on_generation_end = nullptr;
};