#pragma once
#include <vector>
#include <any>
#include <functional>
#include <iostream>
#include <typeindex>
#include <map>
#include <memory> // For std::unique_ptr, std::shared_ptr
#include <optional>


namespace SAGOAP
{
	
	// A type-erased container for a single piece of state data.
	using StateProperty = std::any;

	// The Agent's state is a map from a type to its value.
	struct AgentState {
		std::map<std::type_index, StateProperty> properties;
	};

	// Forward declaration for the planner
	class StateTypeRegistry;

	// Declaration for the debug-only global pointer to the last used registry.
	// The `extern` keyword tells the compiler this is defined in another file.
	extern const StateTypeRegistry* g_pDebugRegistry;

	// A Goal is a partial AgentState. The planner's job is to make the current state
	// match the goal state.
	using Goal = AgentState;

	struct StateTypeFunctions
	{
		// (const base_property, const delta_property) -> new_property
		std::function<StateProperty(const StateProperty&, const StateProperty&)> add;
		// (const base_property, const delta_property) -> new_property
		std::function<StateProperty(const StateProperty&, const StateProperty&)> subtract;
		std::function<size_t(const StateProperty&)> get_hash;
		std::function<bool(const StateProperty&)> is_empty;
		std::function<std::string(const StateProperty&)> to_string;
	};

	// The registry holds the function pointers for all user-defined state types.
	class StateTypeRegistry
	{
	public:
		// Registers a new type T. T must have the methods:
		// - T(const T&) (be copyable)
		// - void AddValues(const T& other)
		// - void SubtractValues(const T& other)
		// - size_t GetHash() const
		// - bool IsEmpty() const
		template <typename T>
		void RegisterType();

		const StateTypeFunctions* GetFunctions(std::type_index type) const;

	private:
		std::map<std::type_index, StateTypeFunctions> registry;
	};
	
	class BaseAction
	{
	public:
		virtual ~BaseAction() = default;

		AgentState requirements;
		AgentState results;

		// Override this to give their action a name for debugging.
		virtual std::string GetName() const = 0;
		
		// Is this action a candidate to help satisfy the given goal?
		virtual bool IsRelevant(const AgentState& currentState, const Goal& goal) const = 0;

		// Populate the requirements and results based on the goal.
		void Configure(const AgentState& currentState, const Goal& goal);
		
		// Implement these to be called by Configure.
		virtual AgentState GenerateRequirements(const AgentState& currentState, const Goal& goal) = 0;
		virtual AgentState GenerateResults(const AgentState& currentState, const Goal& goal) = 0;

		// Create a deep copy of this action, including its configured state.
		virtual std::unique_ptr<BaseAction> Clone() const = 0;

		// Get the cost of performing this action.
		virtual float GetCost() const = 0;

		// To String method for Debugging
		std::string DebugToString(const StateTypeRegistry& registry) const;
	};

	template <typename... ActionTypes>
	class ActionGenerator
	{
	public:
		std::vector<std::unique_ptr<BaseAction>> GenerateActions(
			const AgentState& currentState, const Goal& goal) const;

	private:
		template <typename ActionType>
		void CreateActionIfRelevant(const AgentState& currentState,
									const Goal& goal,
									std::vector<std::unique_ptr<BaseAction>>& actions) const;
	};

	// =============================================================================
	// A* Node and Supporting Structures
	// =============================================================================

	struct GoapNode
	{
		AgentState currentState;
		Goal currentGoal;

		float gCost = 0.0f; // Cost from start to this node
		float hCost = 0.0f; // Heuristic cost from this node to the final goal
		float fCost = 0.0f; // gCost + hCost

		std::shared_ptr<GoapNode> parent = nullptr; // To reconstruct the path
		std::unique_ptr<BaseAction> action = nullptr; // Action that led to this node

		GoapNode(AgentState&& state, Goal&& goal);
		void CalculateFCost();
	};

	// Comparator for A* priority queue (min-heap on fCost)
	struct CompareGoapNodes
	{
		bool operator()(const std::shared_ptr<GoapNode>& a, const std::shared_ptr<GoapNode>& b) const;
	};
	
	using HeuristicFunction = std::function<float(const AgentState& /*state*/, const Goal& /*goal*/)>;

	// =============================================================================
	// GOAP Planner
	// =============================================================================

	class GoapPlanner
	{
	public:
		template <typename ActionGeneratorType>
		static std::vector<std::unique_ptr<BaseAction>> Plan(
			const AgentState& initialCurrentState,
			const Goal& initialGoal,
			HeuristicFunction heuristic,
			const ActionGeneratorType& actionGenerator,
			const StateTypeRegistry& registry);
	};
	
	namespace Utils
	{
		// Helper functions for easy access to state properties
		template<typename T>
		void Set(AgentState& state, T value);

		template<typename T>
		const T* Get(const AgentState& state);

		template<typename T>
		T* GetMutable(AgentState& state);

		// Core state manipulation functions
		AgentState CombineStates(const AgentState& base, const AgentState& delta, const StateTypeRegistry& registry);
		AgentState SubtractStates(const AgentState& base, const AgentState& to_subtract, const StateTypeRegistry& registry);

		// Hashing for AgentState
		size_t GetStateHash(const AgentState& state, const StateTypeRegistry& registry);

		// ToString for Debugging
		std::string DebugToString(const AgentState& state, const StateTypeRegistry& registry);
	} // namespace Utils

	namespace Debug
	{
		// Call this function once at the start of your main/test to enable
		// rich debugger visualizations for AgentState and Actions.
		void SetRegistryForVisualization(const StateTypeRegistry* registry);
	}
	
}

#ifdef __cplusplus
extern "C" {
#endif

	// This is the function the debugger will call.
	// It takes a pointer to an AgentState and returns a temporary string buffer.
	// We make it __declspec(dllexport) to ensure it's visible.
	__declspec(dllexport) const char* GetAgentStateDebugString(const SAGOAP::AgentState* state);

#ifdef __cplusplus
}
#endif

// --- Template Implementations ---
#include "SAGOAP.inl"