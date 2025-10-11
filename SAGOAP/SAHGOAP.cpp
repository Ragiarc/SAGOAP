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

    void WorldModel::RegisterActionGenerator(const std::string& name, ActionInstanceGenerator func) {
        registered_generators[name] = std::move(func);
    }

    void WorldModel::RegisterSymbol(const std::string& symbol) {
        if (symbol_to_id.find(symbol) == symbol_to_id.end()) {
            int new_id = static_cast<int>(id_to_symbol.size());
            symbol_to_id[symbol] = new_id;
            id_to_symbol.push_back(symbol);
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
    
    const ActionInstanceGenerator* WorldModel::GetActionGenerator(const std::string& name) const {
        auto it = registered_generators.find(name);
        return (it != registered_generators.end()) ? &it->second : nullptr;
    }

    const std::vector<ActionSchema>& WorldModel::GetActionSchemas() const
    {
        return registered_schemas;
    }

    void WorldModel::RegisterGoalApplier(const std::string& conditionName, GoalApplierFunction func) {
        registered_goal_appliers[conditionName] = std::move(func);
    }

    void WorldModel::RegisterActionSchema(ActionSchema schema)
    {
        registered_schemas.push_back(std::move(schema));
    }

    const WorldModel::ConditionInfo* WorldModel::GetConditionInfo(const std::string& name) const {
        auto it = registered_conditions.find(name);
        return (it != registered_conditions.end()) ? &it->second : nullptr;
    }

    const WorldModel::EffectInfo* WorldModel::GetEffectInfo(const std::string& name) const {
        auto it = registered_effects.find(name);
        return (it != registered_effects.end()) ? &it->second : nullptr;
    }
    const GoalApplierFunction* WorldModel::GetGoalApplier(const std::string& name) const {
        auto it = registered_goal_appliers.find(name);
        return (it != registered_goal_appliers.end()) ? &it->second : nullptr;
    }

    // =============================================================================
    // Planner Internals & Implementation
    // =============================================================================

    // Helper function to resolve an ActionInstance into an optimized, planner-friendly format.

    std::optional<std::vector<int>> ResolveParams(const std::optional<ActionInstance>& inst, const std::vector<std::string>& param_strings, const WorldModel& model) {
        std::vector<int> resolved_params;
        resolved_params.reserve(param_strings.size());
        for (const std::string& param_str : param_strings) {
            if (param_str.empty()) continue;
            if (param_str[0] == '$') {
                if (!inst) return std::nullopt;
                std::string param_name = param_str.substr(1);
                auto it = inst.value().params.find(param_name);
                if (it == inst.value().params.end()) return std::nullopt;
                resolved_params.push_back(it->second);
            } else if (param_str[0] == '@') {
                resolved_params.push_back(model.GetSymbolId(param_str.substr(1)));
            } else {
                try {
                    resolved_params.push_back(std::stoi(param_str));
                } catch (const std::exception&) {
                    return std::nullopt; // Invalid literal integer
                }
            }
        }
        return resolved_params;
    }
    
    std::optional<ResolvedAction> ResolveAction(const ActionInstance& inst, const WorldModel& model) {
        ResolvedAction res;
        res.schema = inst.schema;
        res.cost = inst.schema->cost;

        // Resolve Preconditions
        for (const auto& cond_schema : inst.schema->preconditions) {
            // 1. Get the info struct.
            const auto* cond_info = model.GetConditionInfo(cond_schema.name);
            if (!cond_info) return std::nullopt; // Condition name not registered.

            // 2. Resolve the parameters for this instance.
            auto params = ResolveParams(inst, cond_schema.params, model);
            if (!params) return std::nullopt; // Parameter name was wrong.
        
            // 3. Create the ResolvedOperation and push it back.
            // This is the corrected line. We are moving the function, params, and op into the struct.
            res.resolved_preconditions.push_back({&cond_info->erased_func, std::move(*params), cond_schema.op});
        }

        for (const auto& effect_schema : inst.schema->effects) {
            // 1. Get the EffectInfo, not ConditionInfo
            const auto* effect_info = model.GetEffectInfo(effect_schema.name);
            if (!effect_info) return std::nullopt;

            // 2. Resolve parameters
            auto params = ResolveParams(inst, effect_schema.params, model);
            if (!params) return std::nullopt;

            // 3. Create the ResolvedEffect and push to the correct vector
            res.resolved_effects.push_back({&effect_info->erased_func, std::move(*params)});
        }

        return res;
    }

    using StateGoal = std::vector<Condition>;
    
    class AchieveStateTask : public BaseTask {
    public:
        StateGoal targetConditions;
        AchieveStateTask(StateGoal conditions) : targetConditions(std::move(conditions)) {}
        bool Decompose(const AgentState&, Goal&) const override { return false; /* Handled by planner */ }
        std::unique_ptr<BaseTask> Clone() const override {return std::make_unique<AchieveStateTask>(this->targetConditions); }
        std::string GetName() const override { return "AchieveState"; }
    };

    class ExecuteActionTask : public BaseTask {
    public:
        ActionInstance action;
        ExecuteActionTask(ActionInstance act) : action(std::move(act)) {}
        bool Decompose(const AgentState&, Goal&) const override { return false; /* Handled by planner */ }
        std::unique_ptr<BaseTask> Clone() const override { return std::make_unique<ExecuteActionTask>(action); }
        std::string GetName() const override { return "Execute: " + action.schema->name; }
    };

    namespace internal
    {
        // =============================================================================
        // Planner Internals (Implementation Detail)
        // =============================================================================

        struct PlannerNode {
            AgentState currentState;
            Goal tasksRemaining;
            std::shared_ptr<ActionInstance> parentActionInstance;

            float gCost = 0.0f;
            float hCost = 0.0f;
            float fCost = 0.0f;
            std::shared_ptr<PlannerNode> parent = nullptr;
        
            void CalculateFCost() { fCost = gCost + hCost; }
            // A more robust hash is needed for production.
            size_t GetHash() const {
                size_t stateHash = 0;
                size_t goalHash = 0;
                for(const auto& task : tasksRemaining) { goalHash ^= std::hash<std::string>()(task->GetName()); }
                return stateHash ^ (goalHash << 1);
            }
        };

        struct ComparePlannerNodes
        {
            bool operator()(const std::shared_ptr<PlannerNode>& a, const std::shared_ptr<PlannerNode>& b) const {
                if (std::abs(a->fCost - b->fCost) > 1e-6) return a->fCost > b->fCost;
                return a->hCost > b->hCost;
            }
        };

    } // namespace internal



    std::optional<std::vector<ActionInstance>> Planner::Plan(
        const AgentState& initialState,
        StateGoal& initialConditions,
        const WorldModel& worldModel,
        HeuristicFunction heuristic) const
    {
        Goal initialGoal;
        initialGoal.push_back(std::make_unique<AchieveStateTask>(initialConditions));
        
        std::priority_queue<std::shared_ptr<internal::PlannerNode>, std::vector<std::shared_ptr<internal::PlannerNode>>, internal::ComparePlannerNodes> openSet;
        std::unordered_set<size_t> closedSet;
        
        auto startNode = std::make_shared<internal::PlannerNode>();
        startNode->currentState = initialState;
        startNode->tasksRemaining = std::move(initialGoal);
        startNode->gCost = 0.0f;
        startNode->hCost = heuristic(startNode->currentState, startNode->tasksRemaining);
        startNode->CalculateFCost();
        openSet.push(startNode);

        auto createDecompositionNode = [&](std::shared_ptr<internal::PlannerNode> parent, Goal&& newTasks) {
            auto neighborNode = std::make_shared<internal::PlannerNode>();
            neighborNode->currentState = parent->currentState; // State doesn't change on decomposition
            neighborNode->tasksRemaining = std::move(newTasks);
            neighborNode->parent = parent;
            neighborNode->gCost = parent->gCost; // Cost doesn't increase
            neighborNode->hCost = heuristic(neighborNode->currentState, neighborNode->tasksRemaining);
            neighborNode->CalculateFCost();
            openSet.push(neighborNode);
        };

        while (!openSet.empty()) {
            std::shared_ptr<internal::PlannerNode> currentNode = openSet.top();
            openSet.pop();

            if (currentNode->tasksRemaining.empty()) {
                std::vector<ActionInstance> finalPlan;
                std::shared_ptr<internal::PlannerNode> pathNode = currentNode;
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
            if (closedSet.count(currentHash)) continue;
            closedSet.insert(currentHash);

            auto currentTask = currentNode->tasksRemaining.front()->Clone();
            Goal remainingTasks;
            for (size_t i = 1; i < currentNode->tasksRemaining.size(); ++i) {
                remainingTasks.push_back(currentNode->tasksRemaining[i]->Clone());
            }

            // --- Case 1: AchieveStateTask ---
            if (auto* achieveTask = dynamic_cast<AchieveStateTask*>(currentTask.get()))
            {
                StateGoal unsatisfiedConditions;
                for (const auto& condition : achieveTask->targetConditions) {
                    const auto* func = worldModel.GetConditionInfo(condition.name);
                    auto params = ResolveParams({}, condition.params, worldModel); // No instance params for this check

                    if (!func || !params || !(func->erased_func)(currentNode->currentState, *params, condition.op)) {
                        unsatisfiedConditions.push_back(condition);
                    }
                }

                if (unsatisfiedConditions.empty()) {
                    // The task is already complete. Create a "do nothing" node that simply pops the task.
                    auto neighborNode = std::make_shared<internal::PlannerNode>();
                    neighborNode->currentState = currentNode->currentState; // State is unchanged
                    neighborNode->tasksRemaining = std::move(remainingTasks); // Use the list of tasks without the current one
                    neighborNode->parent = currentNode;
                    neighborNode->gCost = currentNode->gCost; // No action was taken, so cost does not increase
                    neighborNode->hCost = heuristic(neighborNode->currentState, neighborNode->tasksRemaining);
                    neighborNode->CalculateFCost();
                    openSet.push(neighborNode);
                    continue;
                }

                // Generate actions based on the UNSATISFIED conditions.
                std::vector<ActionInstance> potentialInstances;
                for (const auto& schema : worldModel.GetActionSchemas()) {
                    if (const auto* generator = worldModel.GetActionGenerator(schema.generator_name)) {
                        // Call the generator with the correct StateGoal type
                        auto newInstances = (*generator)(currentNode->currentState, unsatisfiedConditions, worldModel);
                        for(auto& inst : newInstances) { inst.schema = &schema; }
                        potentialInstances.insert(potentialInstances.end(), 
                                                  std::make_move_iterator(newInstances.begin()), 
                                                  std::make_move_iterator(newInstances.end()));
                    }
                }
                if (potentialInstances.empty()) continue; // Dead end.

                for (auto& instance : potentialInstances)
                {
                    Goal decompTasks;
                    decompTasks.push_back(std::make_unique<AchieveStateTask>(instance.schema->preconditions));
                    decompTasks.push_back(std::make_unique<ExecuteActionTask>(std::move(instance))); 
                    decompTasks.insert(decompTasks.end(), 
                                       std::make_move_iterator(remainingTasks.begin()), 
                                       std::make_move_iterator(remainingTasks.end()));

                    createDecompositionNode(currentNode, std::move(decompTasks));
                }
            }
            // --- Case 2: ExecuteActionTask ---
            else if (auto* executeTask = dynamic_cast<ExecuteActionTask*>(currentTask.get())) {
                const ActionInstance& instance = executeTask->action;
                bool preconditionsMet = true;
                for (const auto& cond_schema : instance.schema->preconditions) {
                    const auto* cond_info = worldModel.GetConditionInfo(cond_schema.name);
                    auto params = ResolveParams(instance, cond_schema.params, worldModel);
                    if (!cond_info || !params || !(cond_info->erased_func)(currentNode->currentState, *params, cond_schema.op)) {
                        preconditionsMet = false;
                        break;
                    }
                }

                if (preconditionsMet) {
                    AgentState nextState = currentNode->currentState;
                    for (const auto& effect_schema : instance.schema->effects) {
                        const auto* effect_info = worldModel.GetEffectInfo(effect_schema.name);
                        auto params = ResolveParams(instance, effect_schema.params, worldModel);
                        if (effect_info && params) {
                            (effect_info->erased_func)(nextState, *params);
                        }
                    }

                    auto neighborNode = std::make_shared<internal::PlannerNode>();
                    neighborNode->currentState = std::move(nextState);
                    neighborNode->tasksRemaining = std::move(remainingTasks);
                    neighborNode->parent = currentNode;
                    neighborNode->parentActionInstance = std::make_shared<ActionInstance>(instance);
                    neighborNode->gCost = currentNode->gCost + instance.schema->cost;
                    neighborNode->hCost = heuristic(neighborNode->currentState, neighborNode->tasksRemaining);
                    neighborNode->CalculateFCost();
                    openSet.push(neighborNode);
                } else {
                    Goal decompTasks;
                    decompTasks.push_back(std::make_unique<AchieveStateTask>(instance.schema->preconditions));
                    decompTasks.push_back(currentTask->Clone());
                    decompTasks.insert(decompTasks.end(), std::make_move_iterator(remainingTasks.begin()), std::make_move_iterator(remainingTasks.end()));
                    createDecompositionNode(currentNode, std::move(decompTasks));
                }
            }
            // --- Case 3: Custom, user-defined complex task ---
            else {
                Goal subTasks;
                if (currentTask->Decompose(currentNode->currentState, subTasks)) {
                     subTasks.insert(subTasks.end(), std::make_move_iterator(remainingTasks.begin()), std::make_move_iterator(remainingTasks.end()));
                    createDecompositionNode(currentNode, std::move(subTasks));
                }
            }
        }

        return std::nullopt;
    }
}
