#include <vector>
#include <any>
#include <functional>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <typeindex>
#include <unordered_map>
#include <typeinfo>
#include <unordered_set>



class StateComponent {
public:
    virtual ~StateComponent() = default;
    
    virtual void AddValues(const StateComponent& otherComponent) = 0;
    virtual void SubtractValues(const StateComponent& otherComponent) = 0;
    virtual std::unique_ptr<StateComponent> Clone() const = 0;
    virtual bool IsEmpty() const = 0;
    virtual size_t GetHash() const = 0;
};

class BaseAction {
public:
    virtual ~BaseAction() = default;
    
    std::vector<std::unique_ptr<StateComponent>> requirements;
    std::vector<std::unique_ptr<StateComponent>> results;
    
    virtual bool IsRelevant(const std::unique_ptr<StateComponent>& goal) const = 0;
    virtual std::vector<std::unique_ptr<StateComponent>> GenerateRequirements(const std::unique_ptr<StateComponent>& goal) = 0;
    virtual std::vector<std::unique_ptr<StateComponent>> GenerateResults(const std::unique_ptr<StateComponent>& goal) = 0;
    void Configure(const std::unique_ptr<StateComponent>& goal) {
        requirements = GenerateRequirements(goal);
        results = GenerateResults(goal);
    }
};

template <typename... ActionTypes>
class ActionGenerator {
public:
    std::vector<std::unique_ptr<BaseAction>> GenerateActions(const std::unique_ptr<StateComponent>& goal) {
        std::vector<std::unique_ptr<BaseAction>> actions;

        // For each action type, instantiate and add if relevant
        (CreateActionIfRelevant<ActionTypes>(goal, actions), ...);

        return actions;
    }

private:
    template <typename ActionType>
    void CreateActionIfRelevant(const std::unique_ptr<StateComponent>& goal, 
                                std::vector<std::unique_ptr<BaseAction>>& actions) {
        std::unique_ptr<ActionType> action = std::make_unique<ActionType>();
        
        if (action->IsRelevant(goal)) {
            action->Configure(goal); // Populate requirements and results
            actions.push_back(std::move(action)); // Add configured action to list
        }
    }
};

namespace StateComponentUtils {

    std::vector<std::unique_ptr<StateComponent>> CombineComponentLists(
        const std::vector<std::unique_ptr<StateComponent>>& baseComponents,
        const std::vector<std::unique_ptr<StateComponent>>& addComponents)
    {
        std::map<std::type_index, std::unique_ptr<StateComponent>> componentTypeToUpdatedValue;

        // Add base components to the map
        for (const auto& baseComponent : baseComponents) {
            std::type_index typeIndex = typeid(*baseComponent);
            componentTypeToUpdatedValue[typeIndex] = baseComponent->Clone();
        }

        // Add or update components based on addComponents
        for (const auto& addComponent : addComponents) {
            std::type_index typeIndex = typeid(*addComponent);
            if (componentTypeToUpdatedValue.count(typeIndex)) {
                // If component of this type already exists, add to it
                componentTypeToUpdatedValue[typeIndex]->AddValues(*addComponent);
            }
            else {
                // Otherwise, clone and add the new component
                componentTypeToUpdatedValue[typeIndex] = addComponent->Clone();
            }
        }

        // Move unique_ptr values from the map to the vector
        std::vector<std::unique_ptr<StateComponent>> combinedComponents;
        combinedComponents.reserve(componentTypeToUpdatedValue.size());
        for (auto& [typeIndex, component] : componentTypeToUpdatedValue)
        {
            combinedComponents.push_back(std::move(component));
        }

        return combinedComponents;
    }

    std::vector<std::unique_ptr<StateComponent>> RemoveComponentList(
        const std::vector<std::unique_ptr<StateComponent>>& baseComponents,
        const std::vector<std::unique_ptr<StateComponent>>& componentsToRemove)
    {
        std::vector<std::unique_ptr<StateComponent>> updatedComponents;

        for (const auto& baseComponent : baseComponents) {
            bool toRemove = false;
            std::unique_ptr<StateComponent> clonedBase = baseComponent->Clone(); // Deep copy

            for (const auto& removeComponent : componentsToRemove) {
                if (typeid(*baseComponent) == typeid(*removeComponent)) {
                    clonedBase->SubtractValues(*removeComponent);
                    toRemove = true;
                    break;
                }
            }

            if (!toRemove || !clonedBase->IsEmpty()) {
                updatedComponents.push_back(std::move(clonedBase));
            }
        }

        return updatedComponents;
    }

} // namespace GoalUtils

class GoapPlanner {
private:
    // Map to store hash functions for each StateComponent type
    std::unordered_set<std::type_index> registeredComponents;

    // Set to store registered actions (consider using a vector if order matters)
    std::unordered_set<std::type_index> registeredActions;

public:
    // Registers a StateComponent type and its associated hash function
    template <typename T>
    void RegisterStateComponent() {
        static_assert(std::is_base_of_v<StateComponent, T>, "T must derive from StateComponent");

        registeredComponents.insert(std::type_index(typeid(T)));
    }

    // Registers an Action type
    template <typename T>
    void RegisterAction() {
        static_assert(std::is_base_of_v<BaseAction, T>, "T must derive from BaseAction");

        registeredActions.insert(std::type_index(typeid(T)));
    }

    // Plans a sequence of actions to achieve the goal state from the current state
    std::vector<std::unique_ptr<BaseAction>> Plan(
        const std::vector<std::unique_ptr<StateComponent>>& currentState,
        const std::vector<std::unique_ptr<StateComponent>>& goalState
    ) {
        // 1. Generate relevant actions using ActionGenerator and registered actions
        // 2. Use A* search (or another planning algorithm) to find the optimal action sequence
        // 3. Return the action sequence

        // Placeholder for the planning logic (needs implementation)
        std::vector<std::unique_ptr<BaseAction>> plan;
        return plan;
    }
};

