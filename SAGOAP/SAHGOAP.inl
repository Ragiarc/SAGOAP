#pragma once
#include <queue>
#include <unordered_set>
#include <algorithm>

namespace SAHGOAP
{
    template <typename T>
    void StateTypeRegistry::RegisterType()
    {
        std::type_index type = std::type_index(typeid(T));
        StateTypeFunctions functions;
        functions.add = [](const StateProperty& base, const StateProperty& delta) -> StateProperty {
            T new_value = *std::any_cast<const T>(&base);
            const T& delta_value = *std::any_cast<const T>(&delta);
            new_value.AddValues(delta_value);
            return new_value;
        };
        functions.subtract = [](const StateProperty& base, const StateProperty& delta) -> StateProperty {
            T new_value = *std::any_cast<const T>(&base);
            const T& delta_value = *std::any_cast<const T>(&delta);
            new_value.SubtractValues(delta_value);
            return new_value;
        };
        functions.get_hash = [](const StateProperty& prop) -> size_t {
            return std::any_cast<const T>(&prop)->GetHash();
        };
        functions.is_empty = [](const StateProperty& prop) -> bool {
            return std::any_cast<const T>(&prop)->IsEmpty();
        };
        functions.to_string = [](const StateProperty& prop) -> std::string {
            return std::any_cast<const T>(&prop)->ToString();
        };
        registry[type] = functions;
    }

    template <typename... ActionTypes>
    std::vector<std::unique_ptr<BaseAction>> ActionGenerator<ActionTypes...>::GenerateActions(
        const AgentState& currentState, const AgentState& goal) const
    {
        std::vector<std::unique_ptr<BaseAction>> actions;
        (CreateActionIfRelevant<ActionTypes>(currentState, goal, actions), ...);
        return actions;
    }

    template <typename... ActionTypes>
    template <typename ActionType>
    void ActionGenerator<ActionTypes...>::CreateActionIfRelevant(const AgentState& currentState,
        const AgentState& goal,
        std::vector<std::unique_ptr<BaseAction>>& actions) const
    {
        auto new_actions = ActionType::GenerateInstances(currentState, goal);
        if (!new_actions.empty()) {
            actions.insert(actions.end(),
                std::make_move_iterator(new_actions.begin()),
                std::make_move_iterator(new_actions.end()));
        }
    }

    template <typename ActionGeneratorType>
    std::vector<std::unique_ptr<BaseAction>> HierarchicalPlanner::Plan(
        const AgentState& initialState,
        Goal initialGoal,
        const StateTypeRegistry& registry,
        const ActionGeneratorType& actionGenerator,
        HeuristicFunction heuristic) const
    {
        std::priority_queue<std::shared_ptr<PlannerNode>, std::vector<std::shared_ptr<PlannerNode>>, ComparePlannerNodes> openSet;
        std::unordered_set<size_t> closedSet;

        auto clone_plan = [](const std::vector<std::unique_ptr<BaseAction>>& plan) {
            std::vector<std::unique_ptr<BaseAction>> new_plan;
            new_plan.reserve(plan.size());
            for (const auto& action : plan) {
                new_plan.push_back(action->Clone());
            }
            return new_plan;
        };
        
        auto startNode = std::make_shared<PlannerNode>(
            AgentState(initialState),
            std::move(initialGoal),
            std::vector<std::unique_ptr<BaseAction>>{}
        );
        startNode->gCost = 0.0f;
        startNode->hCost = heuristic(startNode->currentState, startNode->tasksRemaining);
        startNode->CalculateFCost();
        openSet.push(startNode);

        while (!openSet.empty()) {
            std::shared_ptr<PlannerNode> currentNode = openSet.top();
            openSet.pop();

            if (currentNode->tasksRemaining.empty()) {
                return std::move(currentNode->planSoFar);
            }
            
            size_t currentHash = Utils::GetStateHash(currentNode->currentState, registry) ^ (Utils::GetGoalHash(currentNode->tasksRemaining) << 1);
            if (closedSet.count(currentHash)) {
                continue;
            }
            closedSet.insert(currentHash);

            auto currentTask = currentNode->tasksRemaining.front()->Clone();
            Goal remainingTasks;
            remainingTasks.reserve(currentNode->tasksRemaining.size() - 1);
            for (size_t i = 1; i < currentNode->tasksRemaining.size(); ++i) {
                remainingTasks.push_back(currentNode->tasksRemaining[i]->Clone());
            }

            std::vector<std::unique_ptr<BaseTask>> subTasks;
            std::unique_ptr<BaseAction> primitiveAction;

            if (auto* achieveTask = dynamic_cast<AchieveStateTask*>(currentTask.get())) {
                AgentState remainingGoal = Utils::SubtractStates(achieveTask->targetState, currentNode->currentState, registry);
                if (remainingGoal.properties.empty()) {
                    // This task is already complete, create a new node with the remaining tasks and continue.
                    auto neighborNode = std::make_shared<PlannerNode>(AgentState(currentNode->currentState), std::move(remainingTasks), clone_plan(currentNode->planSoFar));
                    neighborNode->parent = currentNode;
                    neighborNode->gCost = currentNode->gCost;
                    neighborNode->hCost = heuristic(neighborNode->currentState, neighborNode->tasksRemaining);
                    neighborNode->CalculateFCost();
                    openSet.push(neighborNode);
                    continue;
                }

                auto potentialActions = actionGenerator.GenerateActions(currentNode->currentState, remainingGoal);

                if (potentialActions.empty()) {
                    continue; // Dead end
                }

                // Create a new branch in the search space for EACH possible action.
                while (!potentialActions.empty()) {
                    std::unique_ptr<BaseAction> action = std::move(potentialActions.back());
                    potentialActions.pop_back();

                    if (!action) continue;

                    Goal decompTasks;
                    AgentState requirements = action->GetRequirements(currentNode->currentState);

                    // add an AchieveStateTask if there are actual requirements to achieve.
                    if (!requirements.properties.empty()) {
                        decompTasks.push_back(std::make_unique<AchieveStateTask>(std::move(requirements)));
                    }
                    // ALWAYS add the task to execute the action itself.
                    decompTasks.push_back(std::make_unique<ExecuteActionTask>(std::move(action))); 
    
                    // Add the rest of the original tasks to the end of our new sub-plan
                    for(auto& task : remainingTasks) decompTasks.push_back(task->Clone());

                    // Create a neighbor node that will attempt this decomposition.
                    auto neighborNode = std::make_shared<PlannerNode>(
                        AgentState(currentNode->currentState), 
                        std::move(decompTasks), 
                        clone_plan(currentNode->planSoFar) // Plan hasn't advanced yet
                    );
                    neighborNode->parent = currentNode;
                    neighborNode->gCost = currentNode->gCost; // Cost doesn't increase until an action is executed
                    neighborNode->hCost = heuristic(neighborNode->currentState, neighborNode->tasksRemaining);
                    neighborNode->CalculateFCost();
                    openSet.push(neighborNode);
                }
            }
            else if (currentTask->Decompose(currentNode->currentState, registry, subTasks, primitiveAction)) {
                if (primitiveAction) {
                    AgentState requirements = primitiveAction->GetRequirements(currentNode->currentState);
                    if (!Utils::IsStateSatisfyingGoal(currentNode->currentState, requirements, registry)) {
                        continue; // Invalid decomposition, preconditions not met.
                    }
                    
                    AgentState nextState = Utils::CombineStates(currentNode->currentState, primitiveAction->GetEffects(currentNode->currentState), registry);
                    auto newPlan = clone_plan(currentNode->planSoFar);
                    newPlan.push_back(primitiveAction->Clone());

                    auto neighborNode = std::make_shared<PlannerNode>(std::move(nextState), std::move(remainingTasks), std::move(newPlan));
                    neighborNode->parent = currentNode;
                    neighborNode->gCost = currentNode->gCost + primitiveAction->GetCost(currentNode->currentState);
                    neighborNode->hCost = heuristic(neighborNode->currentState, neighborNode->tasksRemaining);
                    neighborNode->CalculateFCost();
                    openSet.push(neighborNode);
                } else {
                    for (auto& task : remainingTasks) subTasks.push_back(task->Clone());
                    auto neighborNode = std::make_shared<PlannerNode>(AgentState(currentNode->currentState), std::move(subTasks), clone_plan(currentNode->planSoFar));
                    neighborNode->parent = currentNode;
                    neighborNode->gCost = currentNode->gCost;
                    neighborNode->hCost = heuristic(neighborNode->currentState, neighborNode->tasksRemaining);
                    neighborNode->CalculateFCost();
                    openSet.push(neighborNode);
                }
            }
        }
        return {};
    }

    namespace Utils
    {
        template<typename T>
        void Set(AgentState& state, T value)
        {
            state.properties[std::type_index(typeid(T))] = std::move(value);
        }
        template<typename T>
        const T* Get(const AgentState& state)
        {
            auto it = state.properties.find(std::type_index(typeid(T)));
            if (it != state.properties.end()) return std::any_cast<T>(&(it->second));
            return nullptr;
        }
        template<typename T>
        T* GetMutable(AgentState& state)
        {
            auto it = state.properties.find(std::type_index(typeid(T)));
            if (it != state.properties.end()) return std::any_cast<T>(&(it->second));
            return nullptr;
        }
    }
}
