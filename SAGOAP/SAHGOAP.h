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

	struct AgentState {
		std::map<std::type_index, StateProperty> properties;
	};

	using PlannerState = std::vector<int>;

	// =============================================================================
	// Schema & Baking System
	// =============================================================================
	class WorldSchema; // Fwd declare

	class ReadOnlyStateView
	{
	public:
		ReadOnlyStateView(const PlannerState* state, const WorldSchema* schema);
		std::optional<std::string_view> GetSymbolicValue(const std::string& typeName) const;
		std::optional<int> GetMapValue(const std::string& typeName, const std::string& key) const;
		template<typename T>
			std::optional<T> GetStateComponent() const;
	private:
		const PlannerState* plannerState;
		const WorldSchema* worldSchema;
	};

	class TypeSchema; // Fwd declare
	class WorldSchema
	{
	public:
		template <typename T>
		void RegisterType(const std::string& name, const std::vector<std::string>& all_possible_symbols);
		void Finalize();
		
		PlannerState BakeState(const AgentState& highLevelState) const;
		PlannerState BakeGoal(const AgentState& highLevelGoalState) const;

		const TypeSchema* GetTypeSchema(const std::string& name) const;
		const TypeSchema* GetTypeSchema(std::type_index type) const;
		const std::vector<std::string>* GetSchemaNamesForType(std::type_index type) const;
		size_t GetTotalStateSize() const;
	private:
		friend struct FieldVisitor; // Allow the helper to call this
		
		std::map<std::string, TypeSchema> schemas_by_name;
		std::map<std::type_index, std::vector<std::string>> schema_names_by_type;

		size_t totalPlannerStateSize = 0;
	};

	class StateTypeRegistry; // Forward declaration
	extern const StateTypeRegistry* g_pDebugRegistry;

	struct StateTypeFunctions
	{
		std::function<StateProperty(const StateProperty&, const StateProperty&)> add;
		std::function<StateProperty(const StateProperty&, const StateProperty&)> subtract;
		std::function<size_t(const StateProperty&)> get_hash;
		std::function<bool(const StateProperty&)> is_empty;
		std::function<std::string(const StateProperty&)> to_string;
	};

	class StateTypeRegistry
	{
	public:
		template <typename T>
		void RegisterType();
		const StateTypeFunctions* GetFunctions(std::type_index type) const;
	private:
		std::map<std::type_index, StateTypeFunctions> registry;
	};
	
	class BaseAction; // Forward declare

	// A Task is a building block of a plan.
	class BaseTask
	{
	public:
		virtual ~BaseTask() = default;

		// The core of the system: Decompose this task into sub-tasks or a primitive action.
		virtual bool Decompose(
			const AgentState& currentState,
			const StateTypeRegistry& registry,
			std::vector<std::unique_ptr<BaseTask>>& subTasks,
			std::unique_ptr<BaseAction>& primitiveAction) const = 0;

		virtual std::unique_ptr<BaseTask> Clone() const = 0;
		virtual std::string GetName() const = 0;
	};

	// The Goal is a list of tasks.
	using Goal = std::vector<std::unique_ptr<BaseTask>>;

	// Actions are primitive operations.
	class BaseAction
	{
	public:
		virtual ~BaseAction() = default;

		// Each Action class must be able to generate all of its relevant, configured instances.
		// static std::vector<std::unique_ptr<BaseAction>> GenerateInstances(const AgentState&, const AgentState& goalState);
		
		virtual AgentState GetRequirements(const ReadOnlyStateView& currentState) const = 0;
		virtual AgentState GetEffects(const ReadOnlyStateView& currentState) const = 0;
		virtual float GetCost(const ReadOnlyStateView& currentState) const = 0;
		virtual std::unique_ptr<BaseAction> Clone() const = 0;
		virtual std::string GetName() const = 0;
	};
	

	// =============================================================================
	// Hierarchical Forward Planner
	// =============================================================================
	struct H_PlannerNode; // Fwd declare

	class H_Planner
	{
	public:
		using HeuristicFunction = std::function<float(const ReadOnlyStateView&, const Goal&)>;
		
		H_Planner(const WorldSchema& schema);

		template <typename ActionGeneratorType>
		std::vector<std::unique_ptr<BaseAction>> Plan(
			const AgentState& initialState,
			const AgentState& goalState,
			const ActionGeneratorType& actionGenerator,
			HeuristicFunction heuristic) const;
	private:
		const WorldSchema& worldSchema;
	};

	// Helper function for easy access to state properties (no change)
	namespace Utils {
		template<typename T>
		void Set(AgentState& state, T value);
	}

	
	// Wraps the user's action types into a single factory.
	template <typename... ActionTypes>
	class ActionGenerator
	{
	public:
		std::vector<std::unique_ptr<BaseAction>> GenerateActions(
			const ReadOnlyStateView& currentState, const AgentState& goal) const;
	private:
		template <typename ActionType>
		void CreateActionIfRelevant(const ReadOnlyStateView& currentState,
									const AgentState& goal,
									std::vector<std::unique_ptr<BaseAction>>& actions) const;
	};
	

	// =============================================================================
	// Library-Provided Generic Tasks
	// =============================================================================

	// A generic task whose purpose is to make a certain world state true.
	class AchieveStateTask : public BaseTask
	{
	public:
		AgentState targetState;
		AchieveStateTask(AgentState state) : targetState(std::move(state)) {}
		
		// The decomposition logic is now internal to the planner itself.
		bool Decompose(const AgentState&, const StateTypeRegistry&, std::vector<std::unique_ptr<BaseTask>>&, std::unique_ptr<BaseAction>&) const override;
		
		std::unique_ptr<BaseTask> Clone() const override;
		std::string GetName() const override;
	};

	// A generic task that simply executes a primitive action.
	class ExecuteActionTask : public BaseTask
	{
	private:
		std::unique_ptr<BaseAction> actionToExecute;
	public:
		ExecuteActionTask(std::unique_ptr<BaseAction> action);
		const BaseAction* GetAction() const { return actionToExecute.get(); }
		
		bool Decompose(const AgentState&, const StateTypeRegistry&, std::vector<std::unique_ptr<BaseTask>>&, std::unique_ptr<BaseAction>& primitiveAction) const override;

		std::unique_ptr<BaseTask> Clone() const override;
		std::string GetName() const override;
	};


	// =============================================================================
	// Hierarchical Forward Planner
	// =============================================================================

	struct PlannerNode
	{
		AgentState currentState;
		Goal tasksRemaining;
		std::unique_ptr<BaseAction> parentAction;

		float gCost = 0.0f;
		float hCost = 0.0f;
		float fCost = 0.0f;
		std::shared_ptr<PlannerNode> parent = nullptr;
		
		PlannerNode(AgentState&& state, Goal&& tasks);
		void CalculateFCost();
	};

	struct ComparePlannerNodes
	{
		bool operator()(const std::shared_ptr<PlannerNode>& a, const std::shared_ptr<PlannerNode>& b) const;
	};

	class HierarchicalPlanner
	{
	public:
		using HeuristicFunction = std::function<float(const AgentState&, const Goal&)>;

		template <typename ActionGeneratorType>
		std::vector<std::unique_ptr<BaseAction>> Plan(
			const AgentState& initialState,
			const AgentState& goalState,
			const StateTypeRegistry& registry,
			const ActionGeneratorType& actionGenerator,
			HeuristicFunction heuristic) const;
	};
	
	namespace Utils
	{
		template<typename T>
		void Set(AgentState& state, T value);
		template<typename T>
		const T* Get(const AgentState& state);
		template<typename T>
		T* GetMutable(AgentState& state);
		
		AgentState CombineStates(const AgentState& base, const AgentState& delta, const StateTypeRegistry& registry);
		AgentState SubtractStates(const AgentState& base, const AgentState& to_subtract, const StateTypeRegistry& registry);
		bool IsStateSatisfyingGoal(const AgentState& state, const AgentState& goal, const StateTypeRegistry& registry);
		
		size_t GetStateHash(const AgentState& state, const StateTypeRegistry& registry);
		size_t GetGoalHash(const Goal& goal, const StateTypeRegistry& registry); 
		std::string DebugToString(const AgentState& state, const StateTypeRegistry& registry);
	}

	namespace Debug
	{
		void SetRegistryForVisualization(const StateTypeRegistry* registry);
	}
}

extern "C" {
	__declspec(dllexport) const char* GetAgentStateDebugString2(const SAHGOAP::AgentState* state);
}
#include "SAHGOAP_Traits.h"
#include "SAHGOAP.inl"