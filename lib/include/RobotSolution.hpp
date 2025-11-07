// include/net_oper_my/lib/include/RobotSolution.hpp
#pragma once
#include "isolution.hpp"
#include "RobotProblemConfig.hpp"
#include "nop.hpp"
#include <memory>

class RobotSolution : public ISolution {
public:
    explicit RobotSolution(const RobotProblemConfig& config);
    
    void decode(const std::vector<int>& chromosome_params,
                const std::vector<std::vector<int>>& chromosome_struct) override;
    
    std::unique_ptr<ISolution> clone() const override;
    
    std::vector<float> getParameters() const override;
    
    // Доступ к NetOper для внутреннего использования
    NetOper& getNetOper() { return net_oper_; }
    const NetOper& getNetOperConst() const { return net_oper_; }
    
    // Генерирование вариации (вызывает NOP.GenVar)
    void generateVariation(std::vector<int>& variation) {
        net_oper_.GenVar(variation);
    }

    RobotProblemConfig get_config();

    
private:
    NetOper net_oper_;
    RobotProblemConfig config_;
    int int_bits_, frac_bits_, num_params_;
    
    void greyToVector(const std::vector<int>& grey_code);
};
