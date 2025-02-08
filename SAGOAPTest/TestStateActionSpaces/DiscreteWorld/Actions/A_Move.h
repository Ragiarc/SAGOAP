#pragma once
#include "../../../../SAGOAP/SAGOAP.cpp"
class A_Move : public BaseAction
{
public:
    bool IsRelevant(const std::unique_ptr<StateComponent>& goal) const override;
    std::vector<std::unique_ptr<StateComponent>>
    GenerateRequirements(const std::unique_ptr<StateComponent>& goal) override;
    std::vector<std::unique_ptr<StateComponent>> GenerateResults(const std::unique_ptr<StateComponent>& goal) override;
};
