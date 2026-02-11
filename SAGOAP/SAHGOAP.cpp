#include "SAHGOAP.h"
#include <queue>
#include <unordered_set>
#include <algorithm>
#include <set>
#include <string>
#include <sstream>

namespace SAHGOAP
{
    // =============================================================================
    // World Model Implementation
    // =============================================================================

    void WorldModel::RegisterActionGenerator(ActionInstanceGenerator func) {
        registered_generators.push_back(std::move(func));
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
        return (it != symbol_to_id.end()) ? it->second : -1;
    }

    const std::string& WorldModel::GetSymbolName(int symbolId) const {
        if(symbolId >= 0 && symbolId < id_to_symbol.size()) return id_to_symbol[symbolId];
        static std::string empty = "";
        return empty;
    }
    

    const std::vector<ActionInstanceGenerator> WorldModel::GetActionGenerators() const
    {
        return registered_generators;   
    }

    void WorldModel::RegisterGoalApplier(const std::string& conditionName, GoalApplierFunction func) {
        registered_goal_appliers[conditionName] = std::move(func);
    }

    size_t WorldModel::HashState(const AgentState& state) const
    {
        size_t seed = 0;
        std::vector<int> sorted_indices = state.active_indices;
        std::sort(sorted_indices.begin(), sorted_indices.end());
        // Note: To be perfectly safe against "floating point" errors in insertion order affecting hash 
        // (e.g. A then B vs B then A producing different seeds), 
        // you might want to sort active_indices. However, for a planner, 
        // the state usually evolves from a parent, preserving relative order. 
        // If you experience hash oscillation, Sort active_indices here.

        for (int id : sorted_indices) {
            if (id < registered_hashers.size() && registered_hashers[id]) {
                const std::any& comp_data = state.components[id];
                size_t component_hash = registered_hashers[id](comp_data);
                seed ^= component_hash + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
        }
        return seed;
    }

    int WorldModel::GetConditionID(const std::string& name) const {
        auto it = condition_name_to_id.find(name);
        return (it != condition_name_to_id.end()) ? it->second : -1;
    }

    const std::string& WorldModel::GetConditionName(int id) const {
        if (id >= 0 && id < conditions_by_id.size()) {
            return conditions_by_id[id].name;
        }
        static const std::string empty = "";
        return empty;
    }
 

    const WorldModel::ConditionInfo* WorldModel::GetConditionInfo(int id) const {
        if (id >= 0 && id < conditions_by_id.size()) {
            return &conditions_by_id[id];
        }
        return nullptr;
    }

    int WorldModel::GetEffectID(const std::string& name) const {
        auto it = effect_name_to_id.find(name);
        return (it != effect_name_to_id.end()) ? it->second : -1;
    }    

    const WorldModel::EffectInfo* WorldModel::GetEffectInfo(int id) const {
        if (id >= 0 && id < effects_by_id.size()) {
            return &effects_by_id[id];
        }
        return nullptr;
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

    std::optional<std::vector<int>> ResolveParams(const internal::ResolvedAction& action, const std::vector<std::string>& param_strings, const WorldModel& model) {
        std::vector<int> resolved_params;
        resolved_params.reserve(param_strings.size());
        for (const std::string& param_str : param_strings) {
            if (param_str.empty()) continue;
            if (param_str[0] == '$') {
                std::string param_name = param_str.substr(1);
                auto it = action.params.find(param_name);
                if (it == action.params.end()) return std::nullopt;
                resolved_params.push_back(it->second);
            } else if (param_str[0] == '@') {
                resolved_params.push_back(model.GetSymbolId(param_str.substr(1)));
            } else {
                try {
                    resolved_params.push_back(std::stoi(param_str));
                } catch (...) { return std::nullopt; }
            }
        }
        return resolved_params;
    }

    // Helper for non-action contexts (like Goal checking) where there are no $Variable params
    std::optional<std::vector<int>> ResolveParamsNoAction(const std::vector<std::string>& param_strings, const WorldModel& model) {
        std::vector<int> resolved_params;
        resolved_params.reserve(param_strings.size());
        for (const std::string& param_str : param_strings) {
            if (param_str.empty()) continue;
            if (param_str[0] == '@') {
                resolved_params.push_back(model.GetSymbolId(param_str.substr(1)));
            } else if (param_str[0] == '$') {
                return std::nullopt; // Cannot resolve variables without an action context
            } else {
                try {
                    resolved_params.push_back(std::stoi(param_str));
                } catch (...) { return std::nullopt; }
            }
        }
        return resolved_params;
    }

    internal::ResolvedAction WorldModel::ResolveAction(const ActionInstance& inst) const {
        internal::ResolvedAction resolved;
        resolved.name = inst.name;
        resolved.cost = inst.cost;
        resolved.precondition_cost = inst.precondition_cost;
        resolved.params = inst.params; // Copy the parameters

        // 1. Convert Preconditions to Int IDs
        for (const auto& cond : inst.preconditions) {
            int id = GetConditionID(cond.name);
            if (id != -1) {
                resolved.preconditions.push_back({id, cond.params, cond.op});
            }
        }

        // 2. Convert Effects to Int IDs
        for (const auto& eff : inst.effects) {
            int id = GetEffectID(eff.name);
            if (id != -1) {
                resolved.effects.push_back({id, eff.params});
            }
        }

        return resolved;
    }
    
    using StateGoal = std::vector<Condition>;
    
    class AchieveStateTask : public BaseTask {
    public:
        StateGoal targetConditions;
        AchieveStateTask(StateGoal conditions) : targetConditions(std::move(conditions)) {}
        bool Decompose(const AgentState&, Goal&) const override { return false; /* Handled by planner */ }
        std::unique_ptr<BaseTask> Clone() const override {return std::make_unique<AchieveStateTask>(this->targetConditions); }
        std::string GetName() const override { if (targetConditions.empty()) {
            return "AchieveState(EMPTY)";
        }

            std::string debug_name = "AchieveState(";
            bool first_cond = true;
            for (const auto& cond : targetConditions) {
                if (!first_cond) {
                    debug_name += ", ";
                }
                debug_name += cond.name;
                debug_name += "(";
                bool first_param = true;
                for (const auto& param : cond.params) {
                    if (!first_param) {
                        debug_name += ",";
                    }
                    debug_name += param;
                    first_param = false;
                }
                debug_name += ")";
                first_cond = false;
            }
            debug_name += ")";
            return debug_name;
        }


    };

    class ExecuteResolvedTask : public BaseTask {
    public:
        internal::ResolvedAction action; 

        ExecuteResolvedTask(internal::ResolvedAction act) : action(std::move(act)) {}

        bool Decompose(const AgentState&, Goal&) const override { return false; }
        
        std::unique_ptr<BaseTask> Clone() const override { 
            return std::make_unique<ExecuteResolvedTask>(action); 
        }
        
        std::string GetName() const override {
            return "Execute: " + action.name;
        }
    };

    namespace internal
    {
        // =============================================================================
        // Planner Internals (Implementation Detail)
        // =============================================================================

        void PlannerNode::CalculateFCost() { fCost = gCost + hCost; }

        size_t PlannerNode::GetHash(const WorldModel& model) const {
            if (this->hash != 0) return this->hash;
            
            size_t stateHash = model.HashState(currentState);
            size_t goalHash = model.HashGoal(tasksRemaining);

            size_t combinedHash = stateHash;
            combinedHash ^= goalHash + 0x9e3779b9 + (combinedHash << 6) + (combinedHash >> 2);

            this->hash = combinedHash;
            return combinedHash;
        }

        size_t HashCondition(const Condition& cond)
        {
            size_t seed = std::hash<std::string>()(cond.name);
            seed ^= std::hash<int>()(static_cast<int>(cond.op)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            for (const auto& param : cond.params) {
                seed ^= std::hash<std::string>()(param) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
            return seed;
        }
    } // namespace internal

    size_t WorldModel::HashGoal(const Goal& goal) const
    {
        size_t seed = 0;
        for (const auto& task_ptr : goal) {
            size_t task_hash = 0;
            // Use dynamic_cast to determine the type of task and hash its specific contents.
        
            if (const auto* achieveTask = dynamic_cast<const AchieveStateTask*>(task_ptr.get())) {
                // For an AchieveStateTask, hash its list of target conditions.
                task_hash = std::hash<std::string>()("AchieveStateTask");
                for (const auto& cond : achieveTask->targetConditions) {
                    task_hash ^= SAHGOAP::internal::HashCondition(cond);
                }
            } 
            else if (const auto* executeTask = dynamic_cast<const ExecuteResolvedTask*>(task_ptr.get())) {
                // For an ExecuteActionTask, hash the action's name and parameters.
                task_hash = std::hash<std::string>()(executeTask->action.name);
                for (const auto& [key, value] : executeTask->action.params) {
                    task_hash ^= std::hash<std::string>()(key);
                    task_hash ^= std::hash<int>()(value);
                }
            }
            else if (task_ptr) {
                // Fallback for custom user tasks: just hash the name.
                task_hash = std::hash<std::string>()(task_ptr->GetName());
            }

            // Combine the hash for this task into the total seed.
            seed ^= task_hash + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }

    
    
    std::optional<std::vector<ActionInstance>> Planner::Plan(
        const AgentState& initialState,
        StateGoal& initialConditions,
        const WorldModel& worldModel,
        HeuristicFunction heuristic,
        NodeCollector collector = nullptr) const
    {

        int nodes_generated = 0;
        int nodes_expanded = 0;
        printf("\n[PLANNER START]\n---------------\n");

        std::vector<internal::ResolvedCondition> internalGoalConditions;
        for(const auto& cond : initialConditions) {
            int id = worldModel.GetConditionID(cond.name);
            if(id != -1) {
                internalGoalConditions.push_back({ id, cond.params, cond.op });
            }
        }

        auto internalHeuristic = [&](const AgentState& state, const Goal& goal) -> float {
            float total_estimated_cost = 0.0f;
            for (const auto& task_ptr : goal) {
                if (const auto* executeTask = dynamic_cast<const ExecuteResolvedTask*>(task_ptr.get())) {
                    const auto& action = executeTask->action;
                    bool preconditionsMet = true;
                    
                    // FAST LOOP: iterating vector<ResolvedCondition>, no string lookups
                    for (const auto& condition : action.preconditions) {
                        // O(1) Lookup
                        const auto* cond_info = worldModel.GetConditionInfo(condition.conditionId);
                        
                        // Note: ResolveParams still does some string parsing, but map lookup is removed
                        auto params = ResolveParams(action, condition.params, worldModel);
                        
                        if (!cond_info || !params || !(cond_info->erased_func)(state, *params, condition.op)) {
                            preconditionsMet = false;
                            break;
                        }
                    }

                    if (preconditionsMet) total_estimated_cost += action.cost;
                    else total_estimated_cost += action.cost + action.precondition_cost;
                } else {
                    total_estimated_cost += 1.0f; 
                }
            }
            return total_estimated_cost;
        };
        
        Goal initialGoal;
        initialGoal.push_back(std::make_unique<AchieveStateTask>(initialConditions));
        
        std::priority_queue<std::shared_ptr<internal::PlannerNode>, std::vector<std::shared_ptr<internal::PlannerNode>>, internal::ComparePlannerNodes> openSet;
        std::unordered_map<size_t, float> closedSet;
        
        auto startNode = std::make_shared<internal::PlannerNode>();
        startNode->currentState = initialState;
        startNode->tasksRemaining = std::move(initialGoal);
        startNode->gCost = 0.0f;
        startNode->hCost = heuristic(startNode->currentState, startNode->tasksRemaining);
        startNode->CalculateFCost();
        openSet.push(startNode);
        nodes_generated++;

        auto createDecompositionNode = [&](std::shared_ptr<internal::PlannerNode> parent, Goal&& newTasks) {
            auto neighborNode = std::make_shared<internal::PlannerNode>();
            neighborNode->currentState = parent->currentState; // State doesn't change on decomposition
            neighborNode->tasksRemaining = std::move(newTasks);
            neighborNode->parent = parent;
            neighborNode->gCost = parent->gCost; // Cost doesn't increase
            neighborNode->hCost = heuristic(neighborNode->currentState, neighborNode->tasksRemaining);
            neighborNode->CalculateFCost();
            openSet.push(neighborNode);
            nodes_generated++;

            std::string topTaskName;

            if (neighborNode->tasksRemaining.empty()) {
                topTaskName = "[GOAL]";
            } else {
                std::ostringstream oss;
                bool first = true;
                for (const auto& task : neighborNode->tasksRemaining) {
                    if (!first) oss << " -> "; // separator
                    oss << task->GetName();
                    first = false;
                }
                topTaskName = oss.str();
            }
            //printf("  [DECOMP] Pushing new node. Remaining Goal(Tasks): %s, (g=%.1f, h=%.1f, f=%.1f)\n", 
            //       topTaskName.c_str(), neighborNode->gCost, neighborNode->hCost, neighborNode->fCost);
        };

        while (!openSet.empty())
        {
            std::shared_ptr<internal::PlannerNode> currentNode = openSet.top();
            openSet.pop();

            if (currentNode->tasksRemaining.empty()) {
                printf("---------------\n[PLANNER SUCCESS]\n");
                printf("Total Nodes Generated: %d\n", nodes_generated);
                printf("Total Nodes Expanded: %d\n", nodes_expanded);
                printf("---------------\n");
                
                std::vector<ActionInstance> finalPlan;
                std::shared_ptr<internal::PlannerNode> pathNode = currentNode;
                while (pathNode != nullptr && pathNode->parent != nullptr) {
                    if (pathNode->parentActionInstance) {
                         // Reconstruct ActionInstance from ResolvedAction
                        // We do not reconstruct the string lists for preconditions/effects as they aren't needed for execution.
                        ActionInstance inst;
                        inst.name = pathNode->parentActionInstance->name;
                        inst.cost = pathNode->parentActionInstance->cost;
                        inst.params = pathNode->parentActionInstance->params;
                        finalPlan.push_back(inst);
                    }
                    pathNode = pathNode->parent;
                }
                std::reverse(finalPlan.begin(), finalPlan.end());
                return finalPlan;
            }

            //printf("\n[POP] Popped Node #%d (f=%.1f, g=%.1f, h=%.1f). Tasks left: %zu. Top Task: %s\n", 
            //       nodes_expanded, currentNode->fCost, currentNode->gCost, currentNode->hCost, 
            //       currentNode->tasksRemaining.size(), currentNode->tasksRemaining.front()->GetName().c_str());
            
            size_t currentHash = currentNode->GetHash(worldModel);
            auto it = closedSet.find(currentHash);
            if (it != closedSet.end()) {
                // If we've been to this state before, only proceed if the new path is cheaper.
                if (currentNode->gCost >= it->second) {
                    //printf("  [SKIP] Node is already in closed set with a better or equal gCost (%.1f >= %.1f).\n", currentNode->gCost, it->second);
                    continue;
                }
            }
            // Add/update the node in the closed set with its gCost.
            closedSet[currentHash] = currentNode->gCost;

            nodes_expanded++;
            if (collector)
            {
                collector(currentNode);
            }

            auto currentTask = currentNode->tasksRemaining.front()->Clone();
            Goal remainingTasks;
            remainingTasks.reserve(currentNode->tasksRemaining.size() - 1);
            for (size_t i = 1; i < currentNode->tasksRemaining.size(); ++i) {
                remainingTasks.push_back(currentNode->tasksRemaining[i]->Clone());
            }

            // --- Case 1: AchieveStateTask ---
            if (auto* achieveTask = dynamic_cast<AchieveStateTask*>(currentTask.get()))
            {
                // 1. Build a "shopping list" of all conditions we need to solve right now.
                // Start with the conditions from the current AchieveStateTask.
                StateGoal conditionsToConsider = achieveTask->targetConditions;

                // 2. Look ahead to the next ExecuteActionTask and add its preconditions to our list.
                // This allows the planner to make opportunistic, efficient choices.
                for (const auto& nextTask : remainingTasks) {
                    if (const auto* executeTask = dynamic_cast<const ExecuteResolvedTask*>(nextTask.get())) {
                        // Manually convert ResolvedCondition back to Condition
                        for(const auto& rc : executeTask->action.preconditions) {
                            Condition c;
                            c.name = worldModel.GetConditionName(rc.conditionId);
                            c.params = rc.params;
                            c.op = rc.op;
                            conditionsToConsider.push_back(std::move(c));
                        }
                        
                        // conditionsToConsider.insert(conditionsToConsider.end(),
                        //                             executeTask->action.preconditions.begin(),
                        //                             executeTask->action.preconditions.end());


                    }
                    else {
                        // Stop looking ahead if we hit a non-primitive task (e.g., another AchieveStateTask)
                        break;
                    }
                }

                // 3. From our shopping list, find out which conditions are actually unmet in the current state.
                
                StateGoal unsatisfiedConditions;
                for (const auto& condition : conditionsToConsider) {
                    auto params = ResolveParamsNoAction(condition.params, worldModel);
                    int conditionIndex = worldModel.GetConditionID(condition.name);
                    const auto* cond_info = worldModel.GetConditionInfo(conditionIndex);

                    if (!cond_info || !params || !(cond_info->erased_func)(currentNode->currentState, *params, condition.op)) {
                        unsatisfiedConditions.push_back(condition);
                    }
                }

                if (unsatisfiedConditions.empty()) {
                    //printf("  [SATISFIED] AchieveStateTask is already met.\n");
                    // Task is already complete. Create a "do nothing" node.
                    createDecompositionNode(currentNode, std::move(remainingTasks));
                    continue;
                }

                // Generate actions that can satisfy the unmet conditions.
                std::vector<ActionInstance> potentialActions;
                for (const auto& generator : worldModel.GetActionGenerators()) {
                    auto newInstances = generator(currentNode->currentState, unsatisfiedConditions, worldModel);
                    potentialActions.insert(potentialActions.end(), 
                                              std::make_move_iterator(newInstances.begin()), 
                                              std::make_move_iterator(newInstances.end()));
                }
                
                if (potentialActions.empty()) {
                    //printf("  [DEAD END] No action generator could satisfy goal.\n");
                    continue; // Dead end.
                }
                // remove duplicate actions
                std::sort(potentialActions.begin(), potentialActions.end());
                potentialActions.erase(std::unique(potentialActions.begin(), potentialActions.end()), potentialActions.end());

                std::vector<internal::ResolvedAction> resolvedActions;
                resolvedActions.reserve(potentialActions.size());
                for(const auto& raw : potentialActions) {
                    resolvedActions.push_back(worldModel.ResolveAction(raw));
                }
                
                //printf("  [EXPAND] Found %zu potential actions to satisfy goal.\n", potentialActions.size());
                // For each potential action, create a new branch where the next task is to EXECUTE that action.
                while(!resolvedActions.empty())
                {
                    auto action = std::move(resolvedActions.back());
                    resolvedActions.pop_back();

                    Goal decompTasks;
                    decompTasks.push_back(std::make_unique<ExecuteResolvedTask>(std::move(action)));
                    // Re-evaluate the original goal after the action is done.
                    decompTasks.push_back(currentTask->Clone());
                    // Add the rest of the original plan tasks.
                    for(auto& task : remainingTasks) {
                        decompTasks.push_back(task->Clone());
                    }

                    createDecompositionNode(currentNode, std::move(decompTasks));
                }
            }
            // --- Case 2: ExecuteActionTask ---
            // This branch handles tasks that are NOT AchieveStateTask.
            // It can decompose into a list of sub-tasks.
            else if (auto* executeTask = dynamic_cast<ExecuteResolvedTask*>(currentTask.get()))
            {
                bool currentActionPreconditionsMet = true;
                for (const auto& rCond : executeTask->action.preconditions) {
                    const auto* cond_info = worldModel.GetConditionInfo(rCond.conditionId);
                    auto params = ResolveParams(executeTask->action, rCond.params, worldModel);
                    
                    if (!cond_info || !params || !(cond_info->erased_func)(currentNode->currentState, *params, rCond.op)) {
                        currentActionPreconditionsMet = false;
                        break;
                    }
                }

                if (!currentActionPreconditionsMet) {
                    // --- PRECONDITIONS NOT MET ---
                    //printf("  [DECOMP-STRATEGIC] Simulating future tasks to find all required preconditions...\n");
                    StateGoal allNeededPreconditions;
                    AgentState simulatedState = currentNode->currentState; // Start simulation from current state.

                    // Create a temporary list of tasks to simulate, starting with the current one.
                    std::vector<const BaseTask*> tasksToSimulate;
                    tasksToSimulate.push_back(currentTask.get());
                    for (const auto& task : remainingTasks) {
                        tasksToSimulate.push_back(task.get());
                    }

                    // --- NEW SIMULATION LOOP ---
                    for (const auto* taskToSim : tasksToSimulate) {
                        if (const auto* nextExecuteTask = dynamic_cast<const ExecuteResolvedTask*>(taskToSim)) {
                            // Step 1: Check preconditions against the CURRENT simulated state.
                            // If a precondition isn't met in our simulation, we need to solve for it.
                            for (const auto& cond : nextExecuteTask->action.preconditions) {
                                auto params = ResolveParams(nextExecuteTask->action, cond.params, worldModel);
                                const auto* cond_info = worldModel.GetConditionInfo(cond.conditionId);
                                if (!cond_info || !params || !(cond_info->erased_func)(simulatedState, *params, cond.op)) {
                                    Condition userFacingCond;
                                    userFacingCond.name = worldModel.GetConditionName(cond.conditionId);
                                    userFacingCond.params = cond.params; // These are still strings, so simple copy
                                    userFacingCond.op = cond.op;
                                    
                                    allNeededPreconditions.push_back(std::move(userFacingCond));
                                }
                            }

                            // Step 2: Apply effects to the simulated state for the NEXT iteration.
                            for (const auto& effect : nextExecuteTask->action.effects) {
                                const auto* effectInfo = worldModel.GetEffectInfo(effect.effectId);
                                auto params = ResolveParams(nextExecuteTask->action, effect.params, worldModel);
                                if(effectInfo && params) (effectInfo->erased_func)(simulatedState, *params);
                            }
                        } else if (const auto* nextAchieveTask = dynamic_cast<const AchieveStateTask*>(taskToSim)) {
                            // We've hit a high-level goal. Check which parts of it are not already satisfied
                            // by our simulated sequence of actions.
                            for (const auto& cond : nextAchieveTask->targetConditions) {
                                auto params = ResolveParamsNoAction(cond.params, worldModel);
                                int conditionIndex = worldModel.GetConditionID(cond.name);
                                const auto* cond_info = worldModel.GetConditionInfo(conditionIndex);
                                if (!cond_info || !params || !(cond_info->erased_func)(simulatedState, *params, cond.op)) {
                                    allNeededPreconditions.push_back(cond);
                                }
                            }
                            break; // Lookahead stops at the first AchieveState task.
                        } else {
                            // Stop at any other complex task type.
                            break;
                        }
                    }

                    // Now, filter the massive list of preconditions to only those that are ACTUALLY unmet in the REAL current state.
                    StateGoal futureUnmetPreconditions;
                     for(const auto& cond : allNeededPreconditions) {
                        auto params = ResolveParamsNoAction(cond.params, worldModel);
                        int conditionIndex = worldModel.GetConditionID(cond.name);
                        const auto* cond_info = worldModel.GetConditionInfo(conditionIndex);
                        if (!cond_info || !params || !(cond_info->erased_func)(currentNode->currentState, *params, cond.op)) {
                            futureUnmetPreconditions.push_back(cond);
                        }
                    }

                    // Only create a strategic branch if it offers a different (larger) goal than the greedy branch.
                    if (!futureUnmetPreconditions.empty()) {
                        //printf("  [DECOMP-STRATEGIC] Found %zu total unmet preconditions. Creating strategic branches.\n", futureUnmetPreconditions.size());
                        // Generate actions that can satisfy the unmet conditions.
                        std::vector<ActionInstance> potentialActions;
                        for (const auto& generator : worldModel.GetActionGenerators()) {
                            auto newInstances = generator(currentNode->currentState, futureUnmetPreconditions, worldModel);
                            potentialActions.insert(potentialActions.end(), 
                                                      std::make_move_iterator(newInstances.begin()), 
                                                      std::make_move_iterator(newInstances.end()));
                        }
                        
                        if (potentialActions.empty()) {
                            //printf("  [DEAD END] No action generator could satisfy goal.\n");
                            continue; // Dead end.
                        }
                        // remove duplicate actions
                        std::sort(potentialActions.begin(), potentialActions.end());
                        potentialActions.erase(std::unique(potentialActions.begin(), potentialActions.end()), potentialActions.end());
                        
                        //printf("  [EXPAND] Found %zu potential actions to satisfy goal.\n", potentialActions.size());
                        // For each potential action, create a new branch where the next task is to EXECUTE that action.
                        while(!potentialActions.empty())
                        {
                            auto rawAction = std::move(potentialActions.back());
                            potentialActions.pop_back();

                            // Resolve the raw action into the optimized format expected by the Task
                            internal::ResolvedAction resolvedAction = worldModel.ResolveAction(rawAction);

                            Goal decompTasks;
                            decompTasks.push_back(std::make_unique<ExecuteResolvedTask>(std::move(resolvedAction)));
                            // Re-evaluate the original goal after the action is done.
                            decompTasks.push_back(currentTask->Clone());
                            // Add the rest of the original plan tasks.
                            for(auto& task : remainingTasks) {
                                decompTasks.push_back(task->Clone());
                            }

                            createDecompositionNode(currentNode, std::move(decompTasks));
                        }
                    }
                }
                else
                {
                    // --- PRECONDITIONS ARE MET ---
                    // If we reach here, the current action IS executable. We only need to execute it.
                    //printf("  [EXECUTE] Preconditions met for '%s'. Applying effects.\n", executeTask->action.name.c_str());
                    AgentState nextState = currentNode->currentState;
                    for (const auto& rEff : executeTask->action.effects) {
                        const auto* effect_info = worldModel.GetEffectInfo(rEff.effectId);
                        auto params = ResolveParams(executeTask->action, rEff.params, worldModel);
                        if (effect_info && params) {
                            (effect_info->erased_func)(nextState, *params);
                        }
                    }
                    auto neighborNode = std::make_shared<internal::PlannerNode>();
                    neighborNode->currentState = std::move(nextState);
                    neighborNode->tasksRemaining = std::move(remainingTasks);
                    neighborNode->parent = currentNode;
                    neighborNode->parentActionInstance = std::make_shared<internal::ResolvedAction>(executeTask->action);
                    neighborNode->gCost = currentNode->gCost + executeTask->action.cost;
                    neighborNode->hCost = heuristic(neighborNode->currentState, neighborNode->tasksRemaining);
                    neighborNode->CalculateFCost();
                    openSet.push(neighborNode);
                    nodes_generated++;
                    //printf("  [APPLY] Created new state node (f=%.1f)\n", neighborNode->fCost);
                }
            }
        
                // --- Case 3: Custom, user-defined complex task ---
                else {
                    std::vector<std::unique_ptr<BaseTask>> subTasks;
                    for (auto& task : remainingTasks) {
                        subTasks.push_back(task->Clone());
                    }
                    
                    createDecompositionNode(currentNode, std::move(subTasks));
                }
            }
            printf("---------------\n[PLANNER FAILED]\nTotal Nodes Generated: %d\nTotal Nodes Expanded: %d\n---------------\n", nodes_generated, nodes_expanded);
            return std::nullopt;
        }
    }

