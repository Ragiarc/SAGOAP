#pragma once
#include <vector>
#include <any>
#include <functional>
#include <iostream>
#include <typeindex>
#include <queue>
#include <unordered_set>

class StateComponent
{
public:
	virtual ~StateComponent() = default;

	virtual void AddValues(const StateComponent& otherComponent) = 0;
	virtual void SubtractValues(const StateComponent& otherComponent) = 0;
	virtual std::unique_ptr<StateComponent> Clone() const = 0;
	virtual bool IsEmpty() const = 0;
	virtual size_t GetHash() const = 0;
};

class BaseAction
{
public:
	virtual ~BaseAction() = default;

	std::vector<std::unique_ptr<StateComponent>> requirements;
	std::vector<std::unique_ptr<StateComponent>> results;
	
	virtual bool IsRelevant(const std::vector<std::unique_ptr<StateComponent>>& currentState,
	                        const std::vector<std::unique_ptr<StateComponent>>& goal) const = 0;
	virtual std::vector<std::unique_ptr<StateComponent>> GenerateRequirements(const std::vector<std::unique_ptr<StateComponent>>& goal) = 0;
	virtual std::vector<std::unique_ptr<StateComponent>> GenerateResults(const std::vector<std::unique_ptr<StateComponent>>& goal) = 0;

	void Configure(const std::vector<std::unique_ptr<StateComponent>>& goal);

	virtual std::unique_ptr<BaseAction> Clone() const = 0;

	// Get the cost of performing this action
	// TODO: Change to use utility function
	virtual float GetCost() const = 0;
};

template <typename... ActionTypes>
class ActionGenerator
{
public:
	std::vector<std::unique_ptr<BaseAction>> GenerateActions(
		const std::vector<std::unique_ptr<StateComponent>>& currentState, const std::vector<std::unique_ptr<StateComponent>>& goal);

private:
	template <typename ActionType>
	void CreateActionIfRelevant(const std::vector<std::unique_ptr<StateComponent>>& currentState,
	                            const std::vector<std::unique_ptr<StateComponent>>& goal,
	                            std::vector<std::unique_ptr<BaseAction>>& actions);
};

template <typename ... ActionTypes>
std::vector<std::unique_ptr<BaseAction>> ActionGenerator<ActionTypes...>::GenerateActions(
	const std::vector<std::unique_ptr<StateComponent>>& currentState,
	const std::vector<std::unique_ptr<StateComponent>>& goal)
{
	std::vector<std::unique_ptr<BaseAction>> actions;

	// For each action type, instantiate and add if relevant
	(CreateActionIfRelevant<ActionTypes>(currentState, goal, actions), ...);

	return actions;
}

template <typename ... ActionTypes>
template <typename ActionType>
void ActionGenerator<ActionTypes...>::CreateActionIfRelevant(
	const std::vector<std::unique_ptr<StateComponent>>& currentState,
	const std::vector<std::unique_ptr<StateComponent>>& goal, std::vector<std::unique_ptr<BaseAction>>& actions)
{
	std::unique_ptr<ActionType> action = std::make_unique<ActionType>();

	if (action->IsRelevant(currentState, goal))
	{
		action->Configure(goal); // Populate requirements and results
		actions.push_back(std::move(action));
	}
}



namespace StateComponentUtils
{
	std::vector<std::unique_ptr<StateComponent>> CombineComponentLists(
		const std::vector<std::unique_ptr<StateComponent>>& baseComponents,
		const std::vector<std::unique_ptr<StateComponent>>& addComponents);

	std::vector<std::unique_ptr<StateComponent>> RemoveComponentList(
		const std::vector<std::unique_ptr<StateComponent>>& baseComponents,
		const std::vector<std::unique_ptr<StateComponent>>& componentsToRemove)
	;
} // namespace GoalUtils

// --- Need std::hash specializations defined before use ---
// Define a unique preprocessor symbol to guard the hash specializations
#ifndef SAGOAP_HASH_SPECIALIZATIONS
#define SAGOAP_HASH_SPECIALIZATIONS

namespace std
{
	// Hash for vector of unique_ptr<StateComponent>
	template <>
	struct hash<std::vector<std::unique_ptr<StateComponent>>>
	{
		size_t operator()(const std::vector<std::unique_ptr<StateComponent>>& vec) const
		{
			size_t seed = 0;
			// Sort by hash before combining for order-independence? Maybe not needed if StateComponentUtils produces consistent order.
			for (const auto& component : vec)
			{
				if (component)
				{
					// Check for nullptrs if they can occur
					seed ^= component->GetHash() + 0x9e3779b9 + (seed << 6) + (seed >> 2);
				}
			}
			return seed;
		}
	};

	// Hash specialization for vector<StateComponent> (if you ever use it directly)
	template <>
	struct hash<std::vector<StateComponent>>
	{
		size_t operator()(const std::vector<StateComponent>& vec) const noexcept
		{
			size_t seed = 0;
			for (const auto& component : vec)
			{
				seed ^= component.GetHash() + 0x9e3779b9 + (seed << 6) + (seed >> 2);
			}
			return seed;
		}
	};
}

#endif // SAGOAP_HASH_SPECIALIZATIONS

#pragma region A*_Node_and_Supporting_Structures
//=============================================================================
// A* Node and Supporting Structures
//=============================================================================

struct GoapNode; // Forward declaration

struct GoapNode
{
	std::vector<std::unique_ptr<StateComponent>> currentState;
	std::vector<std::unique_ptr<StateComponent>> currentGoal;
	float gCost = 0.0f; // Cost from start to this node
	float hCost = 0.0f; // Heuristic cost from this node to the final goal
	float fCost = 0.0f; // gCost + hCost

	std::shared_ptr<GoapNode> parent = nullptr; // To reconstruct the path
	std::unique_ptr<BaseAction> action = nullptr; // Action that led to this node

	// Constructor for convenience
	GoapNode(std::vector<std::unique_ptr<StateComponent>>&& state,
	         std::vector<std::unique_ptr<StateComponent>>&& goal);

	// Helper to calculate fCost
	void CalculateFCost();

	// Helper to clone a state vector
	static std::vector<std::unique_ptr<StateComponent>> CloneState(
		const std::vector<std::unique_ptr<StateComponent>>& state);
};

// Comparator for A* priority queue (min-heap on fCost)
struct CompareGoapNodes
{
	bool operator()(const std::shared_ptr<GoapNode>& a, const std::shared_ptr<GoapNode>& b) const;
};

// Function to compute a combined hash for a state-goal pair
inline size_t ComputeStateGoalHash(const std::vector<std::unique_ptr<StateComponent>>& state,
                            const std::vector<std::unique_ptr<StateComponent>>& goal)
{
	size_t h1 = std::hash<std::vector<std::unique_ptr<StateComponent>>>{}(state);
	size_t h2 = std::hash<std::vector<std::unique_ptr<StateComponent>>>{}(goal);
	// Combine hashes (boost::hash_combine style)
	return h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
}

// Define the heuristic function signature
using HeuristicFunction = std::function<float(const std::vector<std::unique_ptr<StateComponent>>& /*state*/,
                                              const std::vector<std::unique_ptr<StateComponent>>& /*goal*/)>;

#pragma endregion

class GoapPlanner
{
public:

	template <typename ActionGeneratorType>
	static std::vector<std::unique_ptr<BaseAction>> Plan(
		const std::vector<std::unique_ptr<StateComponent>>& initialCurrentState,
		const std::vector<std::unique_ptr<StateComponent>>& initialGoalState,
		HeuristicFunction heuristic,
		const ActionGeneratorType& actionGenerator // Pass the generator instance by const reference
	);
};

template <typename ActionGeneratorType>
std::vector<std::unique_ptr<BaseAction>> GoapPlanner::Plan(
	const std::vector<std::unique_ptr<StateComponent>>& initialCurrentState,
	const std::vector<std::unique_ptr<StateComponent>>& initialGoalState, HeuristicFunction heuristic,
	const ActionGeneratorType& actionGenerator)
{
	// --- Initialization ---
	std::priority_queue<std::shared_ptr<GoapNode>,
	                    std::vector<std::shared_ptr<GoapNode>>,
	                    CompareGoapNodes> openSet;

	// Use an unordered_set of the combined hash for the closed set
	std::unordered_set<size_t> closedSet;

	// Create the start node
	auto startNode = std::make_shared<GoapNode>(
		GoapNode::CloneState(initialCurrentState), // Clone initial state
		GoapNode::CloneState(initialGoalState) // Clone initial goal
	);
	startNode->gCost = 0.0f;
	startNode->hCost = heuristic(startNode->currentState, startNode->currentGoal);
	startNode->CalculateFCost();

	openSet.push(startNode);

	// --- A* Search Loop ---
	while (!openSet.empty())
	{
		// Get node with lowest fCost
		std::shared_ptr<GoapNode> currentNode = openSet.top();
		openSet.pop();

		// --- Goal Check ---
		// If the current goal is empty, we've found a plan!
		// (Could also add a check: IsGoalSatisfied(currentNode->currentState, currentNode->currentGoal))
		if (currentNode->currentGoal.empty())
		{
			// Reconstruct path
			std::vector<std::unique_ptr<BaseAction>> plan;
			std::shared_ptr<GoapNode> pathNode = currentNode;
			while (pathNode->parent != nullptr)
			{
				// Clone the action as the node owns it now
				// Or transfer ownership if actions aren't reused
				plan.push_back(pathNode->action->Clone()); // Assumes Action implements Clone()
				pathNode = pathNode->parent;
			}
			std::reverse(plan.begin(), plan.end());
			return plan;
		}

		// --- Add to Closed Set ---
		size_t currentHash = ComputeStateGoalHash(currentNode->currentState, currentNode->currentGoal);
		// If already processed this state-goal combo, skip
		if (closedSet.count(currentHash))
		{
			continue;
		}
		closedSet.insert(currentHash);

		
		std::vector<std::unique_ptr<BaseAction>> potentialActions;
		if (!currentNode->currentGoal.empty())
		{
			potentialActions = actionGenerator.GenerateActions(currentNode->currentState,
			                                                   currentNode->currentGoal);
		}


		for (auto& action : potentialActions)
		{
			if (!action) continue; // Should not happen if generator is correct

			// --- Calculate Successor State and Goal ---
			std::vector<std::unique_ptr<StateComponent>> nextState = StateComponentUtils::CombineComponentLists(
				currentNode->currentState, action->results
			);

			// nextGoal = currentGoal - action.results + action.requirements
			std::vector<std::unique_ptr<StateComponent>> tempGoal = StateComponentUtils::RemoveComponentList(
				currentNode->currentGoal, action->results
			);
			std::vector<std::unique_ptr<StateComponent>> nextGoal = StateComponentUtils::CombineComponentLists(
				tempGoal, action->requirements
			);

			// --- Check if Successor is in Closed Set ---
			size_t nextHash = ComputeStateGoalHash(nextState, nextGoal);
			if (closedSet.count(nextHash))
			{
				continue;
			}

			// --- Create Neighbor Node ---
			float tentativeGCost = currentNode->gCost + action->GetCost();

			// Check if a node for this state/goal is already in the open set with a lower gCost
			// (Simple A* often skips this check and relies on the closed set, just adding the new node)

			auto neighborNode = std::make_shared<GoapNode>(std::move(nextState), std::move(nextGoal));
			neighborNode->parent = currentNode;
			neighborNode->action = std::move(action); // Transfer ownership to the node
			neighborNode->gCost = tentativeGCost;
			neighborNode->hCost = heuristic(neighborNode->currentState, neighborNode->currentGoal);
			neighborNode->CalculateFCost();

			openSet.push(neighborNode);
		}
		// Prevent use-after-move for actions transferred to neighbor nodes
		potentialActions.clear();
	}

	// --- No Solution ---
	return {}; // Return empty vector if open set is empty and goal not found
}
