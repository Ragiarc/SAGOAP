#include <vector>
#include <any>
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

    std::vector<std::unique_ptr<StateComponent>> CombineComponentLists(
    const std::vector<std::unique_ptr<StateComponent>>& baseComponents,
    const std::vector<std::unique_ptr<StateComponent>>& addComponents) 
    {
        std::vector<std::unique_ptr<StateComponent>> combinedComponents;
        std::map<const char*, std::unique_ptr<StateComponent>> componentTypeToUpdatedValue;

        for (const auto& baseComponent : baseComponents) {
            std::unique_ptr<StateComponent> combinedComponent = baseComponent->Clone();
            auto baseComponentType = typeid(*combinedComponent).name();

            componentTypeToUpdatedValue[baseComponentType] = std::move(combinedComponent);
            
            for (const auto& addComponent : addComponents) {
                auto addComponentType = typeid(*addComponent).name();
                
                if (baseComponentType == typeid(*addComponent).name()) {
                    auto baseComp = std::move(componentTypeToUpdatedValue[baseComponentType]);
                    baseComp->AddValues(*addComponent);
                    componentTypeToUpdatedValue[baseComponentType] = std::move(baseComp);
                    break;
                } else
                {
                    auto addComponentClone = addComponent->Clone();
                    if (componentTypeToUpdatedValue.contains(addComponentType)) { addComponentClone->AddValues(*componentTypeToUpdatedValue[addComponentType].get());}
                    componentTypeToUpdatedValue[addComponentType] = std::move(addComponentClone);
                }
            }
        }

        for(auto it = componentTypeToUpdatedValue.begin(); it != componentTypeToUpdatedValue.end(); ++it ) {
            combinedComponents.push_back( std::move(it->second) );
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
            for (const auto& removeComponent : componentsToRemove) {
                if (typeid(*baseComponent).name() == typeid(*removeComponent).name()) {
                    auto clonedBase = baseComponent->Clone();
                    clonedBase->SubtractValues(*removeComponent);
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

