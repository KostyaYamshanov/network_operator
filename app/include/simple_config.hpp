#pragma once
#include <vector>
#include <string>
#include <memory>
#include "nop.hpp"

struct SimpleConfig {   
    std::vector<std::vector<int>> base_matrix = {
    {0, 0, 0, 0, 0, 1, 1, 26, 0, 2, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 1, 0, 2, 0, 0, 0, 0, 0},
    {0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
    {0, 0, 0, 0, 4, 0, 0, 3, 0, 22, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 12},
    {0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 16, 0},
    {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 8, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 1, 14, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 0, 0, 1, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 11, 8},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 13},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 18},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}
};
    std::vector<float> base_params = {0., 0.};

    std::vector<int> nodes_for_vars = {0};
    std::vector<int> nodes_for_params = {1, 2};
    std::vector<int> nodes_for_output = {13};

    float x_start = 0.0f;      // Начало диапазона
    float x_step = 0.2f;          // Шаг
    int num_samples = 1000;       // Количество точек для оценки
    float free_param = 2.5f;         // Параметр целевой функции q
};
