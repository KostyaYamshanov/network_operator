#pragma once
#include <vector>
#include <memory>

class ISolution
{
public:
    virtual ~ISolution() = default;

    // Декодирование из хромосомы
    virtual void decode(const std::vector<int> &chromosome_params,
                        const std::vector<std::vector<int>> &chromosome_struct) = 0;

    // Клонирование
    virtual std::unique_ptr<ISolution> clone() const = 0;

    // Опционально: сериализация
    virtual std::vector<float> getParameters() const = 0;
};