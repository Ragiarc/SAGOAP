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
        const AgentState& goalState,
        const StateTypeRegistry& registry,
        const ActionGeneratorType& actionGenerator,
        HeuristicFunction heuristic) const
    {
        Goal initialTasks;
        initialTasks.push_back(std::make_unique<AchieveStateTask>(AgentState(goalState)));
        
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
            std::move(initialTasks)
        );
        startNode->gCost = 0.0f;
        startNode->hCost = heuristic(startNode->currentState, startNode->tasksRemaining);
        startNode->CalculateFCost();
        openSet.push(startNode);

        auto createDecompositionNode = [&](std::shared_ptr<PlannerNode> parent, Goal&& newTasks) {
            auto neighborNode = std::make_shared<PlannerNode>(AgentState(parent->currentState), std::move(newTasks));
            neighborNode->parent = parent;
            neighborNode->gCost = parent->gCost; // Cost doesn't increase on decomposition
            neighborNode->hCost = heuristic(neighborNode->currentState, neighborNode->tasksRemaining);
            neighborNode->CalculateFCost();
            openSet.push(neighborNode);
        };
        
        while (!openSet.empty()) {
            std::shared_ptr<PlannerNode> currentNode = openSet.top();
            openSet.pop();

            if (currentNode->tasksRemaining.empty()) {
                std::vector<std::unique_ptr<BaseAction>> finalPlan;
                std::shared_ptr<PlannerNode> pathNode = currentNode;

                // Loop until we walk all the way back to the start node (whose parent is null)
                while (pathNode != nullptr && pathNode->parent != nullptr) {
                    // If the transition to this node had an action, add it to the plan.
                    if (pathNode->parentAction) {
                        finalPlan.push_back(pathNode->parentAction->Clone());
                    }
                    pathNode = pathNode->parent;
                }
                std::reverse(finalPlan.begin(), finalPlan.end());
                return finalPlan;
            }
            
            size_t currentHash = Utils::GetStateHash(currentNode->currentState, registry) ^ (Utils::GetGoalHash(currentNode->tasksRemaining, registry) << 1);
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
                    createDecompositionNode(currentNode, std::move(remainingTasks));
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
                    createDecompositionNode(currentNode, std::move(decompTasks));
                }
            }
            else if (currentTask->Decompose(currentNode->currentState, registry, subTasks, primitiveAction)) {
                // This branch handles tasks that are NOT AchieveStateTask.
                // It can decompose into a primitive action OR a list of sub-tasks.
                
                if (primitiveAction) {
                    
                    // 1. Get the requirements for this primitive action.
                    AgentState requirements = primitiveAction->GetRequirements(currentNode->currentState);

                    // 2. Check if the requirements are ALREADY met.
                    if (Utils::IsStateSatisfyingGoal(currentNode->currentState, requirements, registry)) {
                        // PRECONDITIONS MET: We can execute this action right now.
                        // This is the "base case" of the recursion.
                        
                        AgentState nextState = Utils::CombineStates(currentNode->currentState, primitiveAction->GetEffects(currentNode->currentState), registry);

                        // Create a neighbor node with the updated state and the remaining tasks.
                        auto neighborNode = std::make_shared<PlannerNode>(std::move(nextState), std::move(remainingTasks));
                        neighborNode->parent = currentNode;
                        neighborNode->parentAction = primitiveAction->Clone();
                        neighborNode->gCost = currentNode->gCost + primitiveAction->GetCost(currentNode->currentState);
                        neighborNode->hCost = heuristic(neighborNode->currentState, neighborNode->tasksRemaining);
                        neighborNode->CalculateFCost();
                        openSet.push(neighborNode);

                    } else {
                        // PRECONDITIONS NOT MET: We need to create a sub-goal!
                        // This is the logic that was missing.
                        
                        // Create a new task list for the decomposition.
                        Goal decompTasks;
                        
                        // a. Add a task to achieve the unmet requirements.
                        decompTasks.push_back(std::make_unique<AchieveStateTask>(std::move(requirements)));
                        
                        // b. Add the original task back so we can try it again later.
                        decompTasks.push_back(currentTask->Clone());
                        
                        // c. Add the rest of the parent's plan.
                        for (auto& task : remainingTasks) {
                            decompTasks.push_back(task->Clone());
                        }

                        // Create a neighbor node that will attempt this new sub-plan.
                        // The world state and plan-so-far do not change yet.
                        createDecompositionNode(currentNode, std::move(decompTasks));
                    }
                } else {
                    // The task decomposed into a list of sub-tasks, not a primitive action.
                    // This is for more complex user-defined tasks.
                    
                    // Prepend the new sub-tasks to the list of remaining tasks.
                    for (auto& task : remainingTasks) {
                        subTasks.push_back(task->Clone());
                    }
                    
                    createDecompositionNode(currentNode, std::move(subTasks));
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
