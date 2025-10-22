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

	/**
	 * @class AgentState
	 * @brief A generic container for an agent's world state, composed of multiple Component structs.
	 * The planner simulates changes to this object to find a valid plan.
	 */
	class AgentState
	{
	public:
		std::map<std::type_index, std::any> components;
		/** @brief Gets a mutable pointer to a component of type T. Adds a default-constructed component if not present. */
		template<typename T>
		T* AddComponent();

		/** @brief Gets a const pointer to a component of type T. Returns nullptr if not present. */
		template<typename T>
		const T* GetComponent() const;

	private:
		
	};

	using PlannerState = std::vector<int>;

	enum class ComparisonOperator
	{
		EqualTo, NotEqualTo, GraterThan, GreaterThanOrEqualTo, LessThan, LessThanOrEqualTo
	};

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
		struct ConditionInfo {
			ConditionFunction erased_func;
			std::type_index component_type;
			
			ConditionInfo(ConditionFunction func, std::type_index type)
				: erased_func(std::move(func)), component_type(type) {}
		};

		struct EffectInfo {
			EffectFunction erased_func;
			std::type_index component_type;
			
			EffectInfo(EffectFunction func, std::type_index type)
				: erased_func(std::move(func)), component_type(type) {}
		};

	public:
		void RegisterActionGenerator(ActionInstanceGenerator func);
		void RegisterGoalApplier(const std::string& conditionName, GoalApplierFunction func);
		//void RegisterActionSchema(ActionSchema schema);
		
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
			
			registered_conditions.emplace(
				name, 
				ConditionInfo{std::move(erased_func), std::type_index(typeid(ComponentType))}
			);
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
    
			registered_effects.emplace(
				name,
				EffectInfo{std::move(erased_func), std::type_index(typeid(ComponentType))}
			);
		}

		template<typename ComponentType>
		void RegisterComponentHasher(std::function<size_t(const ComponentType&)> hasher);

		void RegisterSymbol(const std::string& symbol);
		int GetSymbolId(const std::string& symbol) const;
		const std::string& GetSymbolName(int symbolId) const;

		// Internal accessors
		const ConditionInfo* GetConditionInfo(const std::string& name) const;
		const EffectInfo* GetEffectInfo(const std::string& name) const;
		//const std::vector<ActionSchema>& GetActionSchemas() const;
		const GoalApplierFunction* GetGoalApplier(const std::string& name) const;
		const std::vector<ActionInstanceGenerator> GetActionGenerators() const;

		size_t HashState(const AgentState& state) const;

		size_t HashGoal(const Goal& goal) const;

	private:
		using HasherFunction = std::function<size_t(const std::any&)>;
		
		std::map<std::string, ConditionInfo> registered_conditions;
		std::map<std::string, EffectInfo> registered_effects;
		//std::vector<ActionSchema> registered_schemas;
		std::map<std::type_index, HasherFunction> registered_hashers;
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
        // =============================================================================
        // Planner Internals (Implementation Detail)
        // =============================================================================

        struct PlannerNode
        {
	        AgentState currentState;
        	Goal tasksRemaining;
        	std::shared_ptr<ActionInstance> parentActionInstance;

        	float gCost = 0.0f;
        	float hCost = 0.0f;
        	float fCost = 0.0f;
        	std::shared_ptr<PlannerNode> parent = nullptr;
        
        	void CalculateFCost();
        	// A more robust hash is needed for production.
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
	
	std::optional<std::vector<int>> ResolveParams(const std::optional<ActionInstance>& inst, const std::vector<std::string>& param_strings, const WorldModel& model);
	class ExecuteActionTask;
	/*struct ResolvedAction
	{
		struct ResolvedCondition {
			// It holds the type-erased function directly, not the whole info struct.
			const ConditionFunction* func; 
			std::vector<int> params;
			ComparisonOperator op;
		};
		struct ResolvedEffect {
			const EffectFunction* func; 
			std::vector<int> params;
		};

		const ActionSchema* schema;
		float cost;
		std::vector<ResolvedCondition> resolved_preconditions;
		std::vector<ResolvedEffect> resolved_effects;
	};*/
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