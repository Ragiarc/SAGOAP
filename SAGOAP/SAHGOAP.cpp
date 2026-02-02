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
    

    const std::vector<ActionInstanceGenerator> WorldModel::GetActionGenerators() const
    {
        return registered_generators;   
    }

    /*const std::vector<ActionSchema>& WorldModel::GetActionSchemas() const
    {
        return registered_schemas;
    }*/

    void WorldModel::RegisterGoalApplier(const std::string& conditionName, GoalApplierFunction func) {
        registered_goal_appliers[conditionName] = std::move(func);
    }

    /*void WorldModel::RegisterActionSchema(ActionSchema schema)
    {
        registered_schemas.push_back(std::move(schema));
    }*/

    size_t WorldModel::HashState(const AgentState& state) const
    {
        size_t seed = 0;
        // Iterate through the components of the given AgentState.
        // std::map iteration is ordered, which is good for a consistent hash.
        for (const auto& [type_idx, component_any] : state.components) {
            // Find the registered hasher for this component's type.
            auto hasher_it = registered_hashers.find(type_idx);
            if (hasher_it != registered_hashers.end()) {
                // A hasher was found, so call it.
                const HasherFunction& hasher = hasher_it->second;
                size_t component_hash = hasher(component_any);

                // Combine the component's hash into the total seed (boost::hash_combine style).
                seed ^= component_hash + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
        }
        return seed;
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
    
    /*std::optional<ResolvedAction> ResolveAction(const ActionInstance& inst, const WorldModel& model) {
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
    }*/

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

    class ExecuteActionTask : public BaseTask {
    public:
        ActionInstance action;
        ExecuteActionTask(ActionInstance act) : action(std::move(act)) {}
        bool Decompose(const AgentState&, Goal&) const override { return false; /* Handled by planner */ }
        std::unique_ptr<BaseTask> Clone() const override { return std::make_unique<ExecuteActionTask>(action); }
        std::string GetName() const override {
            std::string debug_name = "Execute: " + action.name;
            if (action.params.empty()) {
                return debug_name;
            }

            debug_name += "(";
            bool first = true;
            for (const auto& [key, value] : action.params) {
                if (!first) {
                    debug_name += ", ";
                }
                // Note: We can't know the symbol name for 'value' here, and that's okay.
                // Printing the key and the raw ID is still very informative.
                debug_name += key + "=" + std::to_string(value);
                first = false;
            }
            debug_name += ")";
            return debug_name;
        }
        
    };

    namespace internal
    {
        // =============================================================================
        // Planner Internals (Implementation Detail)
        // =============================================================================

        void PlannerNode::CalculateFCost() { fCost = gCost + hCost; }
            // A more robust hash is needed for production.
        size_t PlannerNode::GetHash(const WorldModel& model) const {
            if (this->hash != 0) return this->hash;
            
            size_t stateHash = model.HashState(currentState);
            
            size_t goalHash = model.HashGoal(tasksRemaining);

            //size_t parentHash = 0;
            //if (this->parent && this->parent->hash != 0) parentHash = this->parent->hash;
            
            size_t combinedHash = stateHash;
            combinedHash ^= goalHash + 0x9e3779b9 + (combinedHash << 6) + (combinedHash >> 2);
            //combinedHash ^= parentHash + 0x9e3779b9 + (combinedHash << 6) + (combinedHash >> 2);

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
            else if (const auto* executeTask = dynamic_cast<const ExecuteActionTask*>(task_ptr.get())) {
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

        heuristic = [&](const AgentState& state, const Goal& goal) -> float {
            float total_estimated_cost = 0.0f;
            for (const auto& task_ptr : goal) {
                if (const auto* executeTask = dynamic_cast<const ExecuteActionTask*>(task_ptr.get())) {
                    const auto& action = executeTask->action;
                    bool preconditionsMet = true;
                    for (const auto& condition : action.preconditions) {
                        auto params = ResolveParams(action, condition.params, worldModel);
                        const auto* cond_info = worldModel.GetConditionInfo(condition.name);
                        if (!cond_info || !params || !(cond_info->erased_func)(state, *params, condition.op)) {
                            preconditionsMet = false;
                            break;
                        }
                    }

                    if (preconditionsMet) {
                        total_estimated_cost += action.cost;
                    } else {
                        total_estimated_cost += action.cost + action.precondition_cost;
                    }
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

            if (currentNode->tasksRemaining.empty() /*|| nodes_expanded > 50*/) {
                printf("---------------\n[PLANNER SUCCESS]\n");
                printf("Total Nodes Generated: %d\n", nodes_generated);
                printf("Total Nodes Expanded: %d\n", nodes_expanded);
                printf("---------------\n");
                
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
                    if (const auto* executeTask = dynamic_cast<const ExecuteActionTask*>(nextTask.get())) {
                        conditionsToConsider.insert(conditionsToConsider.end(),
                                                    executeTask->action.preconditions.begin(),
                                                    executeTask->action.preconditions.end());
                        // We only look ahead to the very next primitive action.
                        //break; 
                    }
                    else {
                        // Stop looking ahead if we hit a non-primitive task (e.g., another AchieveStateTask)
                        break;
                    }
                }

                // 3. From our shopping list, find out which conditions are actually unmet in the current state.
                
                StateGoal unsatisfiedConditions;
                for (const auto& condition : conditionsToConsider) {
                    auto params = ResolveParams(std::nullopt, condition.params, worldModel);
                    const auto* cond_info = worldModel.GetConditionInfo(condition.name);

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
                
                //printf("  [EXPAND] Found %zu potential actions to satisfy goal.\n", potentialActions.size());
                // For each potential action, create a new branch where the next task is to EXECUTE that action.
                while(!potentialActions.empty())
                {
                    auto action = std::move(potentialActions.back());
                    potentialActions.pop_back();

                    Goal decompTasks;
                    decompTasks.push_back(std::make_unique<ExecuteActionTask>(std::move(action)));
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
            else if (auto* executeTask = dynamic_cast<ExecuteActionTask*>(currentTask.get()))
            {
                bool currentActionPreconditionsMet = true;
                for (const auto& cond_schema : executeTask->action.preconditions) {
                    const auto* cond_info = worldModel.GetConditionInfo(cond_schema.name);
                    auto params = ResolveParams(executeTask->action, cond_schema.params, worldModel);
                    if (!cond_info || !params || !(cond_info->erased_func)(currentNode->currentState, *params, cond_schema.op)) {
                        currentActionPreconditionsMet = false;
                        break;
                    }
                }

                if (!currentActionPreconditionsMet) {
                    // --- PRECONDITIONS NOT MET ---
                    // This is where the planner needs to be strategic.
                    
                    // BRANCH 1: The "Greedy" path. Solve only for the immediate action's needs.
                    //printf("  [DECOMP-GREEDY] Preconditions not met for '%s'. Decomposing to achieve them.\n", executeTask->action.name.c_str());
                    {
                        Goal decompTasks;
                        decompTasks.push_back(std::make_unique<AchieveStateTask>(executeTask->action.preconditions));    
                        decompTasks.push_back(currentTask->Clone());
                        for (auto& task : remainingTasks) {
                            decompTasks.push_back(task->Clone());
                        }
                        createDecompositionNode(currentNode, std::move(decompTasks));
                    }

                    // BRANCH 2: The "Strategic" path. Look ahead and solve for the entire sequence.
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
                        if (const auto* nextExecuteTask = dynamic_cast<const ExecuteActionTask*>(taskToSim)) {
                            // Step 1: Check preconditions against the CURRENT simulated state.
                            // If a precondition isn't met in our simulation, we need to solve for it.
                            for (const auto& cond : nextExecuteTask->action.preconditions) {
                                auto params = ResolveParams(nextExecuteTask->action, cond.params, worldModel);
                                const auto* cond_info = worldModel.GetConditionInfo(cond.name);
                                if (!cond_info || !params || !(cond_info->erased_func)(simulatedState, *params, cond.op)) {
                                    allNeededPreconditions.push_back(cond);
                                }
                            }

                            // Step 2: Apply effects to the simulated state for the NEXT iteration.
                            for (const auto& effect : nextExecuteTask->action.effects) {
                                if (const auto* effect_info = worldModel.GetEffectInfo(effect.name)) {
                                    auto params = ResolveParams(nextExecuteTask->action, effect.params, worldModel);
                                    if (params) (effect_info->erased_func)(simulatedState, *params);
                                }
                            }
                        } else if (const auto* nextAchieveTask = dynamic_cast<const AchieveStateTask*>(taskToSim)) {
                            // We've hit a high-level goal. Check which parts of it are not already satisfied
                            // by our simulated sequence of actions.
                            for (const auto& cond : nextAchieveTask->targetConditions) {
                                auto params = ResolveParams(std::nullopt, cond.params, worldModel);
                                const auto* cond_info = worldModel.GetConditionInfo(cond.name);
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
                        auto params = ResolveParams(std::nullopt, cond.params, worldModel);
                        const auto* cond_info = worldModel.GetConditionInfo(cond.name);
                        if (!cond_info || !params || !(cond_info->erased_func)(currentNode->currentState, *params, cond.op)) {
                            futureUnmetPreconditions.push_back(cond);
                        }
                    }
                    //std::sort(futureUnmetPreconditions.begin(), futureUnmetPreconditions.end());
                    //futureUnmetPreconditions.erase(std::unique(futureUnmetPreconditions.begin(), futureUnmetPreconditions.end()), futureUnmetPreconditions.end());

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
                            auto action = std::move(potentialActions.back());
                            potentialActions.pop_back();

                            Goal decompTasks;
                            decompTasks.push_back(std::make_unique<ExecuteActionTask>(std::move(action)));
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
                    for (const auto& effect_schema : executeTask->action.effects) {
                        const auto* effect_info = worldModel.GetEffectInfo(effect_schema.name);
                        auto params = ResolveParams(executeTask->action, effect_schema.params, worldModel);
                        if (effect_info && params) {
                            (effect_info->erased_func)(nextState, *params);
                        }
                    }
                    auto neighborNode = std::make_shared<internal::PlannerNode>();
                    neighborNode->currentState = std::move(nextState);
                    neighborNode->tasksRemaining = std::move(remainingTasks);
                    neighborNode->parent = currentNode;
                    neighborNode->parentActionInstance = std::make_shared<ActionInstance>(executeTask->action);
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

