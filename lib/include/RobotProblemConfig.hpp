// RobotProblemConfig.hpp
#pragma once
#include <vector>
#include <string>
#include <memory>
#include "nop.hpp"

struct RobotProblemConfig {
    // Параметры симуляции
    float dt = 0.033333f;
    float time_limit = 15.0f;
    float epsilon_term = 0.1f;
    int num_trajectories = 8;
    
    // Пространство начальных состояний
    std::vector<float> qyminc = {-5.5f, -5.5f, -1.31f};
    std::vector<float> qymaxc = {5.5f, 5.5f, 1.31f};
    
    // Модель
    std::string model_path = "rosbot_gazebo9_2d_model.onnx";
    
    // Network topology
    std::vector<int> nodes_for_vars = {0, 1, 2};
    std::vector<int> nodes_for_params = {3, 4, 5, 6};
    std::vector<int> nodes_for_output = {22, 23};

    std::shared_ptr<NetOper> nop_template;
};
