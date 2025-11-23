/**
 * @file BaseConfig.hpp
 * @brief Базовый класс конфигурации для нейросетевых моделей
 */

#pragma once
#include <vector>
#include <string>
#include <memory>
#include "nop.hpp"


/**
 * @class BaseConfig
 * @brief Базовая конфигурация, содержащая общую структуру для всех моделей
 */
struct BaseConfig {
    // ===== ОБЯЗАТЕЛЬНЫЕ ПОЛЯ =====
    
    /// Матрица связей между узлами сети
    std::vector<std::vector<int>> base_matrix;
    
    /// Базовые параметры модели
    std::vector<float> base_params;
    
    // ===== РАЗБИЕНИЕ УЗЛОВ =====
    
    /// Индексы узлов, связанных с переменными состояния
    std::vector<int> nodes_for_vars;
    
    /// Индексы узлов, связанных с параметрами модели
    std::vector<int> nodes_for_params;
    
    /// Индексы узлов, связанных с выходом
    std::vector<int> nodes_for_output;
    
    
    // ===== ВИРТУАЛЬНЫЙ ДЕСТРУКТОР =====
    virtual ~BaseConfig() = default;
    
    
    // ===== МЕТОДЫ ВАЛИДАЦИИ =====
    
    /**
     * @brief Проверить корректность конфигурации
     * @return true если конфиг валиден, false иначе
     */
    virtual bool validate() const {
        // Проверка матрицы
        if (base_matrix.empty()) {
            return false;
        }
        
        size_t matrix_size = base_matrix.size();
        for (const auto& row : base_matrix) {
            if (row.size() != matrix_size) {
                return false;  // Матрица должна быть квадратной
            }
        }
        
        // Проверка узлов
        for (int node : nodes_for_vars) {
            if (node < 0 || node >= static_cast<int>(matrix_size)) {
                return false;
            }
        }
        
        for (int node : nodes_for_params) {
            if (node < 0 || node >= static_cast<int>(matrix_size)) {
                return false;
            }
        }
        
        for (int node : nodes_for_output) {
            if (node < 0 || node >= static_cast<int>(matrix_size)) {
                return false;
            }
        }
        
        // Проверка на пересечение наборов узлов
        std::vector<int> all_nodes;
        all_nodes.insert(all_nodes.end(), nodes_for_vars.begin(), nodes_for_vars.end());
        all_nodes.insert(all_nodes.end(), nodes_for_params.begin(), nodes_for_params.end());
        all_nodes.insert(all_nodes.end(), nodes_for_output.begin(), nodes_for_output.end());
        
        std::sort(all_nodes.begin(), all_nodes.end());
        auto last = std::unique(all_nodes.begin(), all_nodes.end());
        
        // Если был дубликат - это ошибка конфигурации
        if (last != all_nodes.end()) {
            return false;
        }
        
        return true;
    }
    
    
    /**
     * @brief Получить размер матрицы
     * @return Размер квадратной матрицы
     */
    size_t getMatrixSize() const {
        return base_matrix.size();
    }
    
    
    /**
     * @brief Получить количество параметров
     * @return Количество параметров
     */
    size_t getNumParams() const {
        return base_params.size();
    }
    
    
    /**
     * @brief Получить количество переменных
     * @return Количество узлов переменных
     */
    size_t getNumVars() const {
        return nodes_for_vars.size();
    }
    
    
    /**
     * @brief Получить количество выходов
     * @return Количество узлов выходов
     */
    size_t getNumOutputs() const {
        return nodes_for_output.size();
    }
};
