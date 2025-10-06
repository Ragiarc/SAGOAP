#include "SAHGOAP.h"
#include <queue>
#include <unordered_set>
#include <algorithm>
#include <string>

namespace SAHGOAP
{
    // =============================================================================
    // World Model Implementation
    // =============================================================================

    void WorldModel::RegisterCondition(const std::string& name, ConditionFunction func) {
        registered_conditions[name] = std::move(func);
    }

    void WorldModel::RegisterEffect(const std::string& name, EffectFunction func) {
        registered_effects[name] = std::move(func);
    }

    void WorldModel::RegisterActionGenerator(const std::string& name, ActionInstanceGenerator func) {
        registered_generators[name] = std::move(func);
    }

    void WorldModel::RegisterSymbol(const std::string& symbol) {
        if (symbol_to_id.find(symbol) == symbol_to_id.end()) {
            symbol_to_id[symbol] = next_symbol_id;
            id_to_symbol[next_symbol_id] = symbol;
            next_symbol_id++;
        }
    }

    int WorldModel::GetSymbolId(const std::string& symbol) const {
        auto it = symbol_to_id.find(symbol);
        if (it != symbol_to_id.end()) {
            return it->second;
        }
        // Consider throwing an error or returning a specific "not found" value
        return -1;
    }

    const std::string& WorldModel::GetSymbolName(int symbolId) const {
        // In a real implementation, add bounds checking.
        return id_to_symbol[symbolId];
    }

    const ConditionFunction* WorldModel::GetCondition(const std::string& name) const {
        auto it = registered_conditions.find(name);
        return (it != registered_conditions.end()) ? &it->second : nullptr;
    }

    const EffectFunction* WorldModel::GetEffect(const std::string& name) const {
        auto it = registered_effects.find(name);
        return (it != registered_effects.end()) ? &it->second : nullptr;
    }
    
    const ActionInstanceGenerator* WorldModel::GetActionGenerator(const std::string& name) const {
        auto it = registered_generators.find(name);
        return (it != registered_generators.end()) ? &it->second : nullptr;
    }

    // =============================================================================
    // Planner Internals & Implementation
    // =============================================================================

    // Helper function to resolve an ActionInstance into an optimized, planner-friendly format.
    std::optional<ResolvedAction> ResolveAction(const ActionInstance& inst, const WorldModel& model) {
        ResolvedAction res;
        res.schema = inst.schema;
        res.cost = inst.schema->cost;

        auto resolve_params = [&](const std::vector<std::string>& param_strings) -> std::optional<std::vector<int>> {
            std::vector<int> resolved_params;
            resolved_params.reserve(param_strings.size());
            for (const std::string& param_str : param_strings) {
                if (param_str.empty()) continue;
                if (param_str[0] == '$') { // Parameter lookup
                    std::string param_name = param_str.substr(1);
                    auto it = inst.params.find(param_name);
                    if (it == inst.params.end()) return std::nullopt; // Failed to find param
                    resolved_params.push_back(it->second);
                } else if (param_str[0] == '@') { // Symbol lookup
                    resolved_params.push_back(model.GetSymbolId(param_str.substr(1)));
                } else { // Literal integer
                    resolved_params.push_back(std::stoi(param_str));
                }
            }
            return resolved_params;
        };

        for (const auto& cond_schema : inst.schema->preconditions) {
            auto func = model.GetCondition(cond_schema.name);
            if (!func) return std::nullopt; // A required C++ function wasn't registered
            
            auto params = resolve_params(cond_schema.params);
            if (!params) return std::nullopt; // A parameter name was wrong

            res.resolved_preconditions.push_back({func, std::move(*params), cond_schema.op});
        }
        // Similar logic would be needed here to resolve effects into a `resolved_effects` vector.

        return res;
    }


    std::optional<std::vector<ActionInstance>> H_Planner::Plan(
        const AgentState& initialState,
        Goal initialGoal,
        const WorldModel& worldModel,
        const std::vector<ActionSchema>& allActionSchemas,
        HeuristicFunction heuristic) const
    {
        std::priority_queue<std::shared_ptr<H_PlannerNode>, std::vector<std::shared_ptr<H_PlannerNode>>, internal::ComparePlannerNodes> openSet;
        std::unordered_set<size_t> closedSet;
        
        auto startNode = std::make_shared<H_PlannerNode>();
        startNode->currentState = initialState;
        startNode->tasksRemaining = std::move(initialGoal);
        startNode->gCost = 0.0f;
        startNode->hCost = heuristic(startNode->currentState, startNode->tasksRemaining);
        startNode->CalculateFCost();
        openSet.push(startNode);

        while (!openSet.empty()) {
            std::shared_ptr<H_PlannerNode> currentNode = openSet.top();
            openSet.pop();

            if (currentNode->tasksRemaining.empty()) {
                // SUCCESS: Reconstruct the plan by walking up the parent pointers.
                std::vector<ActionInstance> finalPlan;
                std::shared_ptr<H_PlannerNode> pathNode = currentNode;
                while (pathNode != nullptr && pathNode->parent != nullptr) {
                    if (pathNode->parentActionInstance) {
                        finalPlan.push_back(*pathNode->parentActionInstance);
                    }
                    pathNode = pathNode->parent;
                }
                std::reverse(finalPlan.begin(), finalPlan.end());
                return finalPlan;
            }
            
            size_t currentHash = currentNode->GetHash();
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

            Goal subTasks;
            if (currentTask->Decompose(currentNode->currentState, subTasks)) {
                // Prepend the new sub-tasks to the list of remaining tasks.
                subTasks.insert(subTasks.end(), 
                                std::make_move_iterator(remainingTasks.begin()), 
                                std::make_move_iterator(remainingTasks.end()));
                
                auto neighborNode = std::make_shared<H_PlannerNode>();
                neighborNode->currentState = currentNode->currentState; // State doesn't change on decomposition
                neighborNode->tasksRemaining = std::move(subTasks);
                neighborNode->parent = currentNode;
                neighborNode->gCost = currentNode->gCost;
                neighborNode->hCost = heuristic(neighborNode->currentState, neighborNode->tasksRemaining);
                neighborNode->CalculateFCost();
                openSet.push(neighborNode);
            }
        }

        return std::nullopt; // No plan found
    }
}
