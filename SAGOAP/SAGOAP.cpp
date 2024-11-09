#include <vector>
#include <any>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <typeinfo>

class StateComponent {
public:
    virtual ~StateComponent() = default;
    // Add or update a property of any type
    void SetProperty(const std::string& key, std::any value) {
        properties[key] = std::move(value);
    }

    // Retrieve a property by key with type checking
    template <typename T>
    T GetProperty(const std::string& key) const {
        if (auto it = properties.find(key); it != properties.end()) {
            return std::any_cast<T>(it->second);
        }
        throw std::runtime_error("Property not found or type mismatch");
    }

    virtual void AddValues(std::unique_ptr<StateComponent> otherComponent) const = 0;
    virtual void SubtractValues(std::unique_ptr<StateComponent> otherComponent) const = 0;
    virtual std::unique_ptr<StateComponent> Clone() const = 0;

private:
    std::unordered_map<std::string, std::any> properties;
};

class BaseAction {
public:
    virtual bool IsRelevant(const StateComponent& goal) const = 0;
    virtual std::vector<StateComponent> GenerateRequirements(const StateComponent& goal) = 0;
    virtual std::vector<StateComponent> GenerateResults(const StateComponent& goal) = 0;
    std::unordered_map<std::vector<StateComponent>, std::vector<StateComponent>> requirementsToResultsMap;
    virtual ~BaseAction() = default;
};

template <typename... ActionTypes>
class ActionGenerator {
public:
    std::vector<std::unique_ptr<BaseAction>> GenerateActions(const StateComponent& goal) {
        std::vector<std::unique_ptr<BaseAction>> actions;

        // For each action type, instantiate and add if relevant
        (CreateActionIfRelevant<ActionTypes>(goal, actions), ...);

        return actions;
    }

private:
    template <typename ActionType>
    void CreateActionIfRelevant(const StateComponent& goal, 
                                std::vector<std::unique_ptr<BaseAction>>& actions) {
        std::unique_ptr<ActionType> action = std::make_unique<ActionType>();
        
        if (action->IsRelevant(goal)) {
            std::vector<StateComponent> requirements = action->GenerateRequirements(goal); 
            std::vector<StateComponent> results = action->GenerateResults(goal);

            action->reqResMap[requirements] = results;
            actions.push_back(std::move(action)); // Add configured action to list
        }
    }
};

namespace GoalUtils {

    std::vector<std::unique_ptr<StateComponent>> ApplyResults(
        const std::vector<std::unique_ptr<StateComponent>>& goal,
        const std::vector<std::unique_ptr<StateComponent>>& results) 
    {
        std::vector<std::unique_ptr<StateComponent>> updatedGoal;

        for (const auto& resultComponent : results) {
            bool toAdd = true;
            for (const auto& goalComponent : goal) {
                if (typeid(goalComponent) == typeid(resultComponent)) {
                    goalComponent->AddValues(resultComponent->Clone());
                    toAdd = false;
                    break;
                }
            }
            if (toAdd) {
                updatedGoal.push_back(resultComponent->Clone()); 
            }
        }

        for (const auto& goalComponent : goal) {
            updatedGoal.push_back(goalComponent->Clone());
        }

        return updatedGoal;
    }

    std::vector<std::unique_ptr<StateComponent>> ApplyRequirements(
        const std::vector<std::unique_ptr<StateComponent>>& goal,
        const std::vector<std::unique_ptr<StateComponent>>& requirements) 
    {
        std::vector<std::unique_ptr<StateComponent>> updatedGoal;

        // Copy the current goal components
        for (const auto& goalComponent : goal) {
            updatedGoal.push_back(goalComponent->Clone());
        }

        // Add each requirement to the goal
        for (const auto& requirement : requirements) {
            updatedGoal.push_back(requirement->Clone()); // Cloning keeps pure function property
        }

        return updatedGoal;
    }

} // namespace GoalUtils

