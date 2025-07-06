#pragma once
// This file should be included at the very end of SAGOAP.h
// It contains the definitions for all template members and functions.
#include <queue>
#include <unordered_set>
#include <algorithm> // for std::reverse

namespace SAGOAP
{
    // =============================================================================
    // StateTypeRegistry Implementation
    // =============================================================================
    template <typename T>
    void StateTypeRegistry::RegisterType()
    {
        std::type_index type = std::type_index(typeid(T));

        StateTypeFunctions functions;

        // add
        functions.add = [](const StateProperty& base, const StateProperty& delta) -> StateProperty {
            T new_value = *std::any_cast<const T>(&base);
            const T& delta_value = *std::any_cast<const T>(&delta);
            new_value.AddValues(delta_value); // User's type MUST have this method.
            return new_value;
        };

        // subtract
        functions.subtract = [](const StateProperty& base, const StateProperty& delta) -> StateProperty {
            T new_value = *std::any_cast<const T>(&base);
            const T& delta_value = *std::any_cast<const T>(&delta);
            new_value.SubtractValues(delta_value); // User's type MUST have this method.
            return new_value;
        };

        // get_hash
        functions.get_hash = [](const StateProperty& prop) -> size_t {
            return std::any_cast<const T>(&prop)->GetHash(); // User's type MUST have this method.
        };

        // is_empty
        functions.is_empty = [](const StateProperty& prop) -> bool {
            return std::any_cast<const T>(&prop)->IsEmpty(); // User's type MUST have this method.
        };

        // to_string
        functions.to_string = [](const StateProperty& prop) -> std::string {
            return std::any_cast<const T>(&prop)->ToString();
        };

        registry[type] = functions;
    }


    // =============================================================================
    // ActionGenerator Implementation
    // =============================================================================
    template <typename... ActionTypes>
    std::vector<std::unique_ptr<BaseAction>> ActionGenerator<ActionTypes...>::GenerateActions(
        const AgentState& currentState, const Goal& goal) const
    {
        std::vector<std::unique_ptr<BaseAction>> actions;
        (CreateActionIfRelevant<ActionTypes>(currentState, goal, actions), ...);
        return actions;
    }

    template <typename... ActionTypes>
    template <typename ActionType>
    void ActionGenerator<ActionTypes...>::CreateActionIfRelevant(
        const AgentState& currentState, const Goal& goal, std::vector<std::unique_ptr<BaseAction>>& actions) const
    {
        auto action = std::make_unique<ActionType>();
        if (action->IsRelevant(currentState, goal))
        {
            action->Configure(currentState, goal);
            actions.push_back(std::move(action));
        }
    }

    // =============================================================================
    // GoapPlanner Implementation
    // =============================================================================
    template <typename ActionGeneratorType>
    std::vector<std::unique_ptr<BaseAction>> GoapPlanner::Plan(
        const AgentState& initialCurrentState,
        const Goal& initialGoal,
        HeuristicFunction heuristic,
        const ActionGeneratorType& actionGenerator,
        const StateTypeRegistry& registry)
    {
        // A helper function for hashing state-goal pairs inside the planner
        auto computeStateGoalHash = [&](const AgentState& state, const Goal& goal) -> size_t {
            size_t h1 = Utils::GetStateHash(state, registry);
            size_t h2 = Utils::GetStateHash(goal, registry);
            return h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
        };
        
        std::priority_queue<std::shared_ptr<GoapNode>, std::vector<std::shared_ptr<GoapNode>>, CompareGoapNodes> openSet;
        std::unordered_set<size_t> closedSet;

        auto startNode = std::make_shared<GoapNode>(AgentState(initialCurrentState), Goal(initialGoal));
        startNode->gCost = 0.0f;
        startNode->hCost = heuristic(startNode->currentState, startNode->currentGoal);
        startNode->CalculateFCost();

        openSet.push(startNode);

        while (!openSet.empty())
        {
            std::shared_ptr<GoapNode> currentNode = openSet.top();
            openSet.pop();

            if (currentNode->currentGoal.properties.empty())
            {
                std::vector<std::unique_ptr<BaseAction>> plan;
                std::shared_ptr<GoapNode> pathNode = currentNode;
                while (pathNode->parent != nullptr)
                {
                    plan.push_back(pathNode->action->Clone());
                    pathNode = pathNode->parent;
                }
                //std::reverse(plan.begin(), plan.end());
                return plan;
            }

            size_t currentHash = computeStateGoalHash(currentNode->currentState, currentNode->currentGoal);
            if (closedSet.count(currentHash))
            {
                continue;
            }
            closedSet.insert(currentHash);

            auto potentialActions = actionGenerator.GenerateActions(currentNode->currentState, currentNode->currentGoal);

            for (auto& action : potentialActions)
            {
                if (!action) continue;

                AgentState nextState = Utils::CombineStates(currentNode->currentState, action->results, registry);
                Goal tempGoal = Utils::SubtractStates(currentNode->currentGoal, action->results, registry);
                Goal nextGoal = Utils::CombineStates(tempGoal, action->requirements, registry);

                size_t nextHash = computeStateGoalHash(nextState, nextGoal);
                if (closedSet.count(nextHash))
                {
                    continue;
                }
                
                auto neighborNode = std::make_shared<GoapNode>(std::move(nextState), std::move(nextGoal));
                neighborNode->parent = currentNode;
                neighborNode->action = std::move(action);
                neighborNode->gCost = currentNode->gCost + neighborNode->action->GetCost();
                neighborNode->hCost = heuristic(neighborNode->currentState, neighborNode->currentGoal);
                neighborNode->CalculateFCost();

                openSet.push(neighborNode);
            }
        }
        return {}; // No solution found
    }

    // =============================================================================
    // Utility Function Implementations
    // =============================================================================
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
            if (it != state.properties.end())
            {
                return std::any_cast<T>(&(it->second));
            }
            return nullptr;
        }

        template<typename T>
        T* GetMutable(AgentState& state)
        {
            auto it = state.properties.find(std::type_index(typeid(T)));
            if (it != state.properties.end())
            {
                return std::any_cast<T>(&(it->second));
            }
            return nullptr;
        }
    } // namespace Utils

} // namespace SAGOAP