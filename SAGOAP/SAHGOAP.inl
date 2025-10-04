#pragma once
#include <queue>
#include <unordered_set>
#include <algorithm>

namespace SAHGOAP
{
class TypeSchema 
	{
	public:
		std::function<void(const StateProperty&, PlannerState&, const WorldSchema&)> bake;
		size_t plannerStateOffset = 0;
		size_t plannerStateSize = 0;
		std::string name;
		std::map<std::string, int> symbolToInt;
		std::vector<std::string> intToSymbol;
	};

	// =============================================================================
	// WorldSchema Implementation
	// =============================================================================

	struct FieldVisitor {
        WorldSchema& worldSchema;
        const std::string& parentTypeName;
        const std::type_index parentTypeIndex;
        const std::vector<std::string>& all_possible_keys;

        template<typename ParentStruct, typename ValueStruct, typename FieldType>
        void visit(const char* fieldName, FieldType ValueStruct::* memberPtr, const char* symbolicTypeName = nullptr) 
        {
            using ParentTrait = Traits::StateTypeTrait<ParentStruct>;
            std::string sub_schema_name = std::string(parentTypeName) + "_" + fieldName;

            TypeSchema sub_schema;
            sub_schema.name = sub_schema_name;
            sub_schema.plannerStateSize = all_possible_keys.size();
        	
            for (int i = 0; i < all_possible_keys.size(); ++i) {
                sub_schema.symbolToInt[all_possible_keys[i]] = i;
                sub_schema.intToSymbol.push_back(all_possible_keys[i]);
            }

            // --- Generate baking logic ---
            sub_schema.bake = [this, memberPtr, symbolicTypeName] (const StateProperty& prop, PlannerState& state, const WorldSchema& ws) {
                const auto& main_map = ParentTrait::GetMap(std::any_cast<const ParentStruct&>(prop));
                const TypeSchema* ss = ws.GetTypeSchema(std::string(parentTypeName) + "_" + fieldName);
                size_t offset = ss->plannerStateOffset;
                
                for (const auto& [outer_key, value_struct] : main_map) {
                    size_t key_id = ss->symbolToInt.at(outer_key);
                    const auto& field_value = value_struct.*memberPtr;

                    if constexpr (std::is_same_v<FieldType, int>) {
                        state[offset + key_id] = field_value;
                    } else if constexpr (std::is_same_v<FieldType, std::string>) {
                        // Find the schema for the symbolic type (e.g., "location")
                        const TypeSchema* sym_schema = ws.GetTypeSchema(symbolicTypeName);
                        state[offset + key_id] = sym_schema->symbolToInt.at(field_value);
                    }
                }
            };

            // Add the sub-schema name to the parent type's lookup list.
            worldSchema.schema_names_by_type[parentTypeIndex].push_back(sub_schema_name);
            // Add the sub-schema itself to the main map.
            worldSchema.schemas_by_name[sub_schema_name] = std::move(sub_schema);
        }
    };
	
	template <typename T>
	void WorldSchema::RegisterType(const std::string& name, const std::vector<std::string>& all_possible_symbols)
	{
		using Trait = Traits::StateTypeTrait<T>;
		std::type_index type_idx = typeid(T);
		TypeSchema schema;
		schema.name = name;
		
		// Use if constexpr to select the right registration logic at compile time
		
		if constexpr (std::is_base_of_v<Traits::SymbolicValueTrait<T::name>, Trait> || 
					  std::is_base_of_v<Traits::SymbolicMapTrait<T::items>, Trait>)
		{
			schema.plannerStateSize = 1;
			schema.intToSymbol.reserve(all_possible_symbols.size());
			for (int i = 0; i < all_possible_symbols.size(); ++i) {
				// Set size based on type
				if constexpr (std::is_base_of_v<Traits::SymbolicValueTrait<T::name>, Trait>) {
					schema.plannerStateSize = 1;
				} else {
					schema.plannerStateSize = all_possible_symbols.size();
				}

				// Populate the symbol maps for this type
				for (int i = 0; i < all_possible_symbols.size(); ++i) {
					schema.symbolToInt[all_possible_symbols[i]] = i;
					schema.intToSymbol.push_back(all_possible_symbols[i]);
				}
				if constexpr (std::is_base_of_v<Traits::SymbolicValueTrait<T::name>, Trait>)
				{
					schema.bake = [](const StateProperty& prop, PlannerState& state, const WorldSchema& worldSchema) {
						const auto* typeSchema = worldSchema.GetTypeSchema(typeid(T));
						const auto& value_str = Trait::GetValue(std::any_cast<const T&>(prop));
						state[typeSchema->plannerStateOffset] = typeSchema->symbolToInt.at(value_str);
					};
				}
				else
				{
					schema.bake = [](const StateProperty& prop, PlannerState& state, const WorldSchema& worldSchema) {
						const auto* typeSchema = worldSchema.GetTypeSchema(typeid(T));
						size_t offset = typeSchema->plannerStateOffset;
						std::fill_n(state.begin() + offset, typeSchema->plannerStateSize, 0);
						const auto& item_map = Trait::GetMap(std::any_cast<const T&>(prop));
						for (const auto& [key, value] : item_map) {
							state[offset + typeSchema->symbolToInt.at(key)] = value;
						}
					};
				}
				
				// Store the link: type T -> schema name
				schema_names_by_type[type_idx].push_back(name);
				// Store the schema itself
				schemas_by_name[name] = std::move(schema);
			}
			
		}
		else if constexpr (std::is_base_of_v<Traits::ComplexMapTrait, Trait>) {
			std::type_index parentTypeIndex = typeid(T);
			FieldVisitor visitor{*this, name, parentTypeIndex, all_possible_symbols};
    
			// This call now works. The `ForEachField` implementation generated by the
			// macros will call the correct specialization of `visitor.visit` with all
			// the necessary type information.
			Trait::FieldProvider::ForEachField(visitor);
		}
	}



	// =============================================================================
	// H_Planner Implementation
	// =============================================================================
	struct H_PlannerNode
	{
		PlannerState currentPlannerState;
		Goal tasksRemaining;
		std::unique_ptr<BaseAction> parentAction;

		float gCost = 0.0f;
		float hCost = 0.0f;
		float fCost = 0.0f;
		std::shared_ptr<H_PlannerNode> parent = nullptr;
		
		void CalculateFCost() { fCost = gCost + hCost; }
	};

	struct Compare_H_PlannerNodes {
		bool operator()(const std::shared_ptr<H_PlannerNode>& a, const std::shared_ptr<H_PlannerNode>& b) const {
			if (std::abs(a->fCost - b->fCost) > 1e-6) return a->fCost > b->fCost;
			return a->hCost > b->hCost;
		}
	};
	
	template <typename ActionGeneratorType>
	std::vector<std::unique_ptr<BaseAction>> H_Planner::Plan(
		const AgentState& initialState,
		const AgentState& goalState,
		const ActionGeneratorType& actionGenerator,
		HeuristicFunction heuristic) const
	{
		PlannerState initialPlannerState = worldSchema.BakeState(initialState);
		
		Goal initialTasks;
		initialTasks.push_back(std::make_unique<AchieveStateTask>(goalState));
		
		std::priority_queue<std::shared_ptr<H_PlannerNode>, std::vector<std::shared_ptr<H_PlannerNode>>, Compare_H_PlannerNodes> openSet;
        std::unordered_set<size_t> closedSet; // Hash of PlannerState + Hash of Task List
		
		auto startNode = std::make_shared<H_PlannerNode>();
		startNode->currentPlannerState = initialPlannerState;
		startNode->tasksRemaining = std::move(initialTasks);
        startNode->gCost = 0.0f;
		ReadOnlyStateView startView(&startNode->currentPlannerState, &worldSchema);
        startNode->hCost = heuristic(startView, startNode->tasksRemaining);
        startNode->CalculateFCost();
		openSet.push(startNode);

		while (!openSet.empty()) {
			auto currentNode = openSet.top();
			openSet.pop();

			if (currentNode->tasksRemaining.empty()) {
                std::vector<std::unique_ptr<BaseAction>> finalPlan;
                auto pathNode = currentNode;
                while (pathNode != nullptr && pathNode->parent != nullptr) {
                    if (pathNode->parentAction) {
                        finalPlan.push_back(pathNode->parentAction->Clone());
                    }
                    pathNode = pathNode->parent;
                }
                std::reverse(finalPlan.begin(), finalPlan.end());
                return finalPlan;
			}
            
            // ... (Full A* loop with state views, baking, etc. needs to be implemented here) ...
            // This is a complex implementation involving hashing PlannerState, cloning tasks,
            // creating views, calling the generator, baking results, and creating neighbor nodes.
		}

		return {}; // No plan found
	}



    
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
        const ReadOnlyStateView& currentState, const AgentState& goal) const
    {
        std::vector<std::unique_ptr<BaseAction>> actions;
        (CreateActionIfRelevant<ActionTypes>(currentState, goal, actions), ...);
        return actions;
    }

    template <typename... ActionTypes>
    template <typename ActionType>
    void ActionGenerator<ActionTypes...>::CreateActionIfRelevant(const ReadOnlyStateView& currentState,
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
                // SUCCESS: Reconstruct the plan by walking up the parent pointers.
                std::vector<std::unique_ptr<BaseAction>> finalPlan;
                std::shared_ptr<PlannerNode> pathNode = currentNode;
                while (pathNode != nullptr && pathNode->parent != nullptr) {
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
