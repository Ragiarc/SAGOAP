#include <vector>
#include <any>
#include <iostream>
#include <stdexcept>
#include <string>
#include <typeindex>
#include <unordered_map>
#include <typeinfo>
#include <unordered_set>

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
    virtual bool IsEmpty() const = 0;

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

namespace StateComponentUtils {

    std::vector<std::unique_ptr<StateComponent>> CombineComponents(
    const std::vector<std::unique_ptr<StateComponent>>& baseComponents,
    const std::vector<std::unique_ptr<StateComponent>>& addComponents) 
    {
        // Create a copy of the goal to work with
        std::vector<std::unique_ptr<StateComponent>> updatedGoal;

        // Track which goal components have been updated by results
        std::unordered_set<std::type_index> updatedTypes;

        // Apply results to the goal
        for (const auto& baseComponent : baseComponents) {
            std::unique_ptr<StateComponent> combinedComponent = baseComponent->Clone();
            for (const auto& addComponent : addComponents) {
                if (typeid(*combinedComponent) == typeid(*addComponent)) {
                    combinedComponent->AddValues(addComponent->Clone());
                    updatedTypes.insert(typeid(*baseComponent));
                    break;
                }
            }
            updatedGoal.push_back(std::move(combinedComponent));
        }

        // Add remaining results that didn't match any goal components
        for (const auto& resultComponent : addComponents) {
            if (!updatedTypes.contains(typeid(resultComponent))) {
                updatedGoal.push_back(resultComponent->Clone());
            }
        }

        return updatedGoal;
    }

    std::vector<std::unique_ptr<StateComponent>> RemoveComponentList(
    const std::vector<std::unique_ptr<StateComponent>>& baseComponents,
    const std::vector<std::unique_ptr<StateComponent>>& componentsToRemove)
    {
        std::vector<std::unique_ptr<StateComponent>> updatedComponents;

        for (const auto& baseComponent : baseComponents) {
            bool toRemove = false;
            for (const auto& removeComponent : componentsToRemove) {
                if (typeid(baseComponent) == typeid(removeComponent)) {
                    auto clonedBase = baseComponent->Clone();
                    clonedBase->SubtractValues(removeComponent->Clone());
                    if (!clonedBase->IsEmpty()) { // Assuming `IsEmpty` checks if the component is effectively null
                        updatedComponents.push_back(std::move(clonedBase));
                    }
                    toRemove = true;
                    break;
                }
            }
            if (!toRemove) {
                updatedComponents.push_back(baseComponent->Clone());
            }
        }

        return updatedComponents;
    }

} // namespace GoalUtils

