#include "RobotSolution.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

RobotSolution::RobotSolution(const RobotProblemConfig& config)
    : config_(config), int_bits_(16), frac_bits_(16), num_params_(4) {
    
    net_oper_ = NetOper();// *config.nop_template;
    net_oper_.setNodesForVars(config_.nodes_for_vars);
    net_oper_.setNodesForParams(config_.nodes_for_params);
    net_oper_.setNodesForOutput(config_.nodes_for_output);
    net_oper_.setCs(config_.base_params);
    net_oper_.setPsi(config_.base_matrix);
}

RobotProblemConfig RobotSolution::get_config()
{
    return config_;
}

void RobotSolution::decode(const std::vector<int>& chromosome_params,
                           const std::vector<std::vector<int>>& chromosome_struct) {
    try {        
        net_oper_.setPsi(config_.base_matrix);
        
        for (size_t i = 0; i < chromosome_struct.size(); ++i) {
            net_oper_.Variations(chromosome_struct[i]);
        }
        
        greyToVector(chromosome_params);
        
    } catch (const std::exception& e) {
        std::cerr << "Error decoding solution: " << e.what() << std::endl;
        throw;
    }
}

std::unique_ptr<ISolution> RobotSolution::clone() const {
    return std::make_unique<RobotSolution>(config_);
}

std::vector<float> RobotSolution::getParameters() const {
    auto& net = const_cast<NetOper&>(net_oper_);
    auto params = net.get_parameters();
    
    if (params.empty()) {
        std::cerr << "Warning: NetOper parameters are empty" << std::endl;
    }
    
    return params;
}

void RobotSolution::greyToVector(const std::vector<int>& grey_code) {
    try {
        if (grey_code.empty()) {
            std::cerr << "Warning: grey_code is empty" << std::endl;
            return;
        }
        
        std::vector<int> binary_code(grey_code.size(), 0);
        int bits_per_param = int_bits_ + frac_bits_;
        
        // Преобразование из кода Грея в бинарный
        for (size_t i = 0; i < grey_code.size(); ++i) {
            if (i % bits_per_param == 0) {
                binary_code[i] = grey_code[i];
            } else {
                binary_code[i] = binary_code[i - 1] ^ grey_code[i];
            }
        }
        
        // Преобразование бинарных блоков в числа
        auto& params = const_cast<NetOper&>(net_oper_).get_parameters();
        params.clear();
        
        double g1 = std::pow(2.0, int_bits_ - 1);  // 2^(int_bits - 1)
        
        for (int param_idx = 0; param_idx < num_params_; ++param_idx) {
            double value = 0.0;
            double g = g1;
            
            int start_bit = param_idx * bits_per_param;
            int end_bit = start_bit + int_bits_;
            
            if (end_bit > static_cast<int>(binary_code.size())) {
                break;
            }
            
            // Целая часть
            for (int i = start_bit; i < end_bit; ++i) {
                value += g * binary_code[i];
                g /= 2.0;
            }
            
            // Дробная часть
            int frac_end = std::min(start_bit + bits_per_param, 
                                    static_cast<int>(binary_code.size()));
            for (int i = end_bit; i < frac_end; ++i) {
                value += g * binary_code[i];
                g /= 2.0;
            }
            
            params.push_back(static_cast<float>(value));
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error in greyToVector: " << e.what() << std::endl;
        throw;
    }
}
