#pragma once
#include <vector>
#include <any>
#include <functional>
#include <iostream>
#include <typeindex>
#include <map>
#include <memory> // For std::unique_ptr, std::shared_ptr
#include <optional>


namespace SAHGOAP
{
	// =============================================================================
	// Core State Representation
	// =============================================================================
	using StateProperty = std::any;

	enum class ComparisonOperator
	{
		EqualTo, NotEqualTo, GraterThan, GreaterThanOrEqualTo, LessThan, LessThanOrEqualTo
	};

	namespace internal
    {
        // ID Generator
        // This effectively assigns a unique integer to every Type at runtime/link time.
        struct ComponentFamily {
            static inline int counter = 0;
        };

        template<typename T>
        struct ComponentTypeID {
            static inline int Get() {
                static int id = ComponentFamily::counter++;
                return id;
            }
        };

		struct ResolvedCondition {
			int conditionId; 
			std::vector<std::string> params; 
			ComparisonOperator op;
		};

		struct ResolvedEffect {
			int effectId;
			std::vector<std::string> params;
		};

		struct ResolvedAction {
			std::string name; // For debug
			int cost;
			int precondition_cost;
			
			std::map<std::string, int> params; 
			
			std::vector<ResolvedCondition> preconditions; 
			std::vector<ResolvedEffect> effects;
			
			bool operator<(const ResolvedAction& other) const {
				if (cost != other.cost) return cost < other.cost;
				return name < other.name; 
			}
		};

	}

	/**
	 * @class AgentState
	 * @brief A generic container for an agent's world state, composed of multiple Component structs.
	 * The planner simulates changes to this object to find a valid plan.
	 */
	class AgentState
    {
    public:
        // Direct O(1) access by ID.
        std::vector<std::any> components; 
        
        // Keep track of which indices actually have data.
        // This makes hashing and iterating O(NumActiveComponents) instead of O(TotalComponentTypes).
        std::vector<int> active_indices; 

        template<typename T>
        T* AddComponent() {
            int id = SAHGOAP::internal::ComponentTypeID<T>::Get();

            // Resize vector if this is a new, higher ID
            if (id >= components.size()) {
                components.resize(id + 1);
            }

            // If empty, initialize and mark active
            if (!components[id].has_value()) {
                components[id] = T{};
                active_indices.push_back(id);
            }

            // Return pointer
            return std::any_cast<T>(&components[id]);
        }

        template<typename T>
        const T* GetComponent() const {
            int id = internal::ComponentTypeID<T>::Get();
            
            if (id < components.size() && components[id].has_value()) {
                // std::any_cast on a pointer avoids exceptions and is fast
                return std::any_cast<T>(&components[id]);
            }
            return nullptr;
        }
    };

	using PlannerState = std::vector<int>;

	
	/** @brief A data-only definition for a single precondition check. */
	struct Condition
	{
		std::string name;              // The registered name, e.g., "Inventory.Has"
		std::vector<std::string> params; // Parameters referenced by name, e.g., ["$itemToGet", "1"]
		ComparisonOperator op = ComparisonOperator::EqualTo;

		// Manual comparison for std::sort and std::unique
		bool operator<(const Condition& other) const {
			if (name != other.name) return name < other.name;
			if (op != other.op) return op < other.op;
			return params < other.params;
		}
		bool operator==(const Condition& other) const {
			return name == other.name && op == other.op && params == other.params;
		}
	};

	/** @brief A data-only definition for a single state-modifying effect. */
	struct Effect
	{
		std::string name;              // The registered name, e.g., "Inventory.Add"
		std::vector<std::string> params; // Parameters referenced by name, e.g., ["$itemToGet", "1"]

		// Manual comparison for std::sort and std::unique
		bool operator<(const Effect& other) const {
			if (name != other.name) return name < other.name;
			return params < other.params;
		}
		bool operator==(const Effect& other) const {
			return name == other.name && params == other.params;
		}
	};
	
	/*struct ActionSchema
	{
		std::string name;
		std::string generator_name;
		std::vector<std::string> param_names;
		int cost = 1;
		std::vector<Condition> preconditions;
		std::vector<Effect> effects;
	};*/
    
	struct ActionInstance
	{
		std::string name;
		int cost;
		int precondition_cost = 0;
		std::map<std::string, int> params; // Still useful for debugging and identification
		std::vector<Condition> preconditions;
		std::vector<Effect> effects;

		// Added to allow sorting and de-duplication of actions
		bool operator<(const ActionInstance& other) const {
			if (name != other.name) return name < other.name;
			if (cost != other.cost) return cost < other.cost;
			if (params != other.params) return params < other.params;
			if (preconditions != other.preconditions) return preconditions < other.preconditions;
			return effects < other.effects;
		}
		bool operator==(const ActionInstance& other) const {
			return name == other.name &&
				   cost == other.cost &&
				   params == other.params &&
				   preconditions == other.preconditions &&
				   effects == other.effects;
		}
	};

	const std::map<std::string, SAHGOAP::ComparisonOperator> operator_map =
	{
		{"EqualTo", SAHGOAP::ComparisonOperator::EqualTo},
		{"NotEqualTo", SAHGOAP::ComparisonOperator::NotEqualTo},
		{"GreaterTHan", SAHGOAP::ComparisonOperator::GraterThan},
		{"GreaterThanOrEqualTo", SAHGOAP::ComparisonOperator::GreaterThanOrEqualTo},
		{"LessThan", SAHGOAP::ComparisonOperator::LessThan},
		{"LessThanOrEqualTo", SAHGOAP::ComparisonOperator::LessThanOrEqualTo}
	};

	class BaseTask; // Forward declaration

	/** @brief The Goal is a list of tasks to be completed in sequence. */
	using Goal = std::vector<std::unique_ptr<BaseTask>>;

	/**
	 * @class BaseTask
	 * @brief The base class for all tasks in a hierarchical plan.
	 * A task's primary job is to decompose itself into a sequence of sub-tasks.
	 */
	class BaseTask
	{
	public:
		virtual ~BaseTask() = default;

		/**
		 * @brief Decomposes the task into a list of sub-tasks.
		 * @param currentState The current simulated world state.
		 * @param subTasks (out) The list of sub-tasks that this task breaks down into.
		 * @return True if decomposition is possible, false otherwise.
		 */
		virtual bool Decompose(const AgentState& currentState, Goal& subTasks) const = 0;

		virtual std::unique_ptr<BaseTask> Clone() const = 0;
		virtual std::string GetName() const = 0;
	};

	// The Goal is a list of tasks.
	using Goal = std::vector<std::unique_ptr<BaseTask>>;
	

	// =============================================================================
	// World Model Registration
	// =============================================================================

	class WorldModel; // Forward declaration

	using ConditionFunction = std::function<bool(const AgentState&, const std::vector<int>&, ComparisonOperator)>;
	using EffectFunction = std::function<void(AgentState&, const std::vector<int>&)>;
	using GoalApplierFunction = std::function<void(AgentState&, const std::vector<int>&)>;
	using StateGoal = std::vector<Condition>;
	using ActionInstanceGenerator = std::function<std::vector<ActionInstance>(
		const AgentState&,        // Current world state
		const StateGoal&,         // The conditions we are trying to satisfy
		const WorldModel&
	)>;

	class WorldModel
	{
	private:
		// Define internal structs first
		

	public:
		struct ConditionInfo {
			std::string name;
			ConditionFunction erased_func;
			std::type_index component_type;
			
			ConditionInfo(std::string n, ConditionFunction func, std::type_index type)
        	: name(std::move(n)), erased_func(std::move(func)), component_type(type) {}
		};

		struct EffectInfo {
			EffectFunction erased_func;
			std::type_index component_type;
			
			EffectInfo(EffectFunction func, std::type_index type)
				: erased_func(std::move(func)), component_type(type) {}
		};

		void RegisterActionGenerator(ActionInstanceGenerator func);
		void RegisterGoalApplier(const std::string& conditionName, GoalApplierFunction func);
		//void RegisterActionSchema(ActionSchema schema);
		const std::string& GetConditionName(int id) const;
		
		template<typename ComponentType>
		void RegisterCondition(const std::string& name, 
			std::function<bool(const ComponentType&, const std::vector<int>&, ComparisonOperator)> func)
			{
				ConditionFunction erased_func = [func](const AgentState& state, const std::vector<int>& params, ComparisonOperator op) -> bool {
					if (const auto* comp = state.GetComponent<ComponentType>()) {
						return func(*comp, params, op);
					}
					return false;
			};
			int new_id = conditions_by_id.size();
        	condition_name_to_id[name] = new_id;
        	conditions_by_id.push_back({name, std::move(erased_func), std::type_index(typeid(ComponentType))});
		}

		template<typename ComponentType>
		void RegisterEffect(const std::string& name,
		std::function<void(ComponentType&, const std::vector<int>&)> func)
		{
			EffectFunction erased_func = [func](AgentState& state, const std::vector<int>& params) {
				if (auto* comp = state.AddComponent<ComponentType>()) {
					func(*comp, params);
				}
			};
    
			int new_id = effects_by_id.size();
			effect_name_to_id[name] = new_id;
			effects_by_id.push_back({std::move(erased_func), std::type_index(typeid(ComponentType))});
		}

		template<typename ComponentType>
		void RegisterComponentHasher(std::function<size_t(const ComponentType&)> hasher);

		void RegisterSymbol(const std::string& symbol);
		int GetSymbolId(const std::string& symbol) const;
		const std::string& GetSymbolName(int symbolId) const;

		// Internal accessors
		int GetConditionID(const std::string& name) const;
		const ConditionInfo* GetConditionInfo(int id) const;
		int GetEffectID(const std::string& name) const;
		const EffectInfo* GetEffectInfo(int id) const;
		//const std::vector<ActionSchema>& GetActionSchemas() const;
		const GoalApplierFunction* GetGoalApplier(const std::string& name) const;
		const std::vector<ActionInstanceGenerator> GetActionGenerators() const;

		size_t HashState(const AgentState& state) const;

		size_t HashGoal(const Goal& goal) const;

		internal::ResolvedAction ResolveAction(const ActionInstance& inst) const;

	private:
		using HasherFunction = std::function<size_t(const std::any&)>;
		
		std::unordered_map<std::string, int> condition_name_to_id;
    	std::unordered_map<std::string, int> effect_name_to_id;
		std::vector<ConditionInfo> conditions_by_id;
    	std::vector<EffectInfo> effects_by_id;
		//std::vector<ActionSchema> registered_schemas;
		std::vector<HasherFunction> registered_hashers;
		std::map<std::string, GoalApplierFunction> registered_goal_appliers;
		std::vector<ActionInstanceGenerator> registered_generators;
		std::map<std::string, int> symbol_to_id;
		std::vector<std::string> id_to_symbol;
	};
	

	// =============================================================================
	// Hierarchical Forward Planner
	// =============================================================================

	namespace internal
    {
        struct PlannerNode
        {
	        AgentState currentState;
        	Goal tasksRemaining;
        	std::shared_ptr<ResolvedAction> parentActionInstance;

        	float gCost = 0.0f;
        	float hCost = 0.0f;
        	float fCost = 0.0f;
        	mutable size_t hash = 0;
        	std::shared_ptr<PlannerNode> parent = nullptr;
        
        	void CalculateFCost();

        	size_t GetHash(const WorldModel& model) const;
        };
        struct ComparePlannerNodes
        {
        	bool operator()(const std::shared_ptr<PlannerNode>& a, const std::shared_ptr<PlannerNode>& b) const
        	{
        		if (std::abs(a->fCost - b->fCost) > 1e-6) return a->fCost > b->fCost;
        		return a->hCost > b->hCost;
        	}
        };

        size_t HashCondition(const Condition& cond);

    } // namespace internal
	
	std::optional<std::vector<int>> ResolveParams(const internal::ResolvedAction& action, const std::vector<std::string>& param_strings, const WorldModel& model);
	class ExecuteResolvedTask;

	using NodeCollector = std::function<void(const std::shared_ptr<internal::PlannerNode>&)>;
	class Planner
	{
	public:
		/** @brief A function that estimates the cost to complete a plan from a given state. */
		using HeuristicFunction = std::function<float(const AgentState&, const Goal&)>;

		std::optional<std::vector<ActionInstance>> Plan(
			const AgentState& initialState,
			StateGoal& initialConditions,
			const WorldModel& worldModel,
			HeuristicFunction heuristic,
			NodeCollector collector) const;
	};
}
#include "SAHGOAP.inl"