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

	// =============================================================================
	// Hierarchical Task & Action Representation
	// =============================================================================

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
		
		virtual AgentState GetRequirements(const AgentState& currentState) const = 0;
		virtual AgentState GetEffects(const AgentState& currentState) const = 0;
		virtual float GetCost(const AgentState& currentState) const = 0;
		virtual std::unique_ptr<BaseAction> Clone() const = 0;
		virtual std::string GetName() const = 0;
	};

	// Wraps the user's action types into a single factory.
	template <typename... ActionTypes>
	class ActionGenerator
	{
	public:
		std::vector<std::unique_ptr<BaseAction>> GenerateActions(
			const AgentState& currentState, const AgentState& goal) const;
	private:
		template <typename ActionType>
		void CreateActionIfRelevant(const AgentState& currentState,
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

#include "SAHGOAP.inl"