#pragma once
#include <vector>
#include <memory>


// Forward declaration
class NetOper;


class ISolution {
public:
    virtual ~ISolution() = default;

    
    // ===== ОСНОВНЫЕ МЕТОДЫ =====
    
    /// Декодирование из хромосомы
    virtual void decode(const std::vector<int>& chromosome_params,
                        const std::vector<std::vector<int>>& chromosome_struct) = 0;

    /// Клонирование
    virtual std::unique_ptr<ISolution> clone() const = 0;

    /// Получение параметров
    virtual std::vector<float> getParameters() const = 0;
    
    
    // ===== ДОСТУП К NetOper (НОВОЕ!) =====
    
    /// Получить неконстантную ссылку на NetOper
    /// @note Evaluator'ы могут использовать это для вычислений
    virtual NetOper& getNetOper() = 0;
    
    /// Получить константную ссылку на NetOper
    virtual const NetOper& getNetOperConst() const = 0;
};