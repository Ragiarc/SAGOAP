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
		/** @brief Gets a mutable pointer to a component of type T. Adds a default-constructed component if not present. */
		template<typename T>
		T* AddComponent();

		/** @brief Gets a const pointer to a component of type T. Returns nullptr if not present. */
		template<typename T>
		const T* GetComponent() const;

	private:
		std::map<std::type_index, std::any> components;
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
	};

	/** @brief A data-only definition for a single state-modifying effect. */
	struct Effect
	{
		std::string name;              // The registered name, e.g., "Inventory.Add"
		std::vector<std::string> params; // Parameters referenced by name, e.g., ["$itemToGet", "1"]
	};
	
	struct ActionSchema
	{
		std::string name;
		std::string generator_name;
		std::vector<std::string> param_names;
		float cost = 1.0f;
		std::vector<Condition> preconditions;
		std::vector<Effect> effects;
	};
    
	struct ActionInstance
	{
		const ActionSchema* schema;
		// Parameters are now stored by name, which is robust. The planner will optimize this.
		std::map<std::string, int> params;
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

	class WorldModel;
	// Type-erased function wrappers for the planner's internal use.
	using ConditionFunction = std::function<bool(const AgentState&, const std::vector<int>&, ComparisonOperator)>;
	using EffectFunction = std::function<void(AgentState&, const std::vector<int>&)>;
	using ActionInstanceGenerator = std::function<std::vector<ActionInstance>(const AgentState&, const AgentState&, const WorldModel&)>;
	/**
	 * @class WorldModel
	 * @brief The central registry where the developer teaches the library about their game's specific components and logic.
	 */
	class WorldModel
	{
	public:
		/** @brief Registers a function that can check a condition on the world state. */
		void RegisterCondition(const std::string& name, ConditionFunction func);

		void WorldModel::RegisterActionGenerator(const std::string& name, ActionInstanceGenerator func);
		
		/** @brief Registers a function that can apply an effect to the world state. */
		void RegisterEffect(const std::string& name, EffectFunction func);

		/** @brief Registers a symbol (e.g., "Forge") and assigns it a unique integer ID. */
		void RegisterSymbol(const std::string& symbol);
		int GetSymbolId(const std::string& symbol) const;
		const std::string& WorldModel::GetSymbolName(int symbolId) const;

		// Internal accessors used by the planner.
		const ConditionFunction* GetCondition(const std::string& name) const;
		const EffectFunction* GetEffect(const std::string& name) const;
		const ActionInstanceGenerator* WorldModel::GetActionGenerator(const std::string& name) const;

	private:
		std::map<std::string, ConditionFunction> registered_conditions;
		std::map<std::string, EffectFunction> registered_effects;
		std::map<std::string, ActionInstanceGenerator> registered_generators;
		std::map<std::string, int> symbol_to_id;
		std::map<int, std::string> id_to_symbol;
		int next_symbol_id = 0;
	};

	// =============================================================================
	// Hierarchical Forward Planner
	// =============================================================================
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


	// This is the optimized, internal representation the planner uses in its hot loop.
	// Parameter names are resolved to integer values.
	struct ResolvedAction
	{
		struct ResolvedOperation {
			const ConditionFunction* func; // Direct function pointer
			std::vector<int> params;
			ComparisonOperator op;
		};

		const ActionSchema* schema;
		float cost;
		std::vector<ResolvedOperation> resolved_preconditions;
		// A similar ResolvedEffect struct would be used for effects
	};
	
	class H_Planner
	{
	public:
		/** @brief A function that estimates the cost to complete a plan from a given state. */
		using HeuristicFunction = std::function<float(const AgentState&, const Goal&)>;

		std::optional<std::vector<ActionInstance>> Plan(
			const AgentState& initialState,
			Goal initialGoal,
			const WorldModel& worldModel,
			const std::vector<ActionSchema>& allActionSchemas,
			HeuristicFunction heuristic) const;
	};
}
#include "SAHGOAP.inl"