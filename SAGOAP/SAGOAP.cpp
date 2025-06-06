#include "SAGOAP.h"
#include <vector>
#include <any>

#include <iostream>
#include <map>

#include <typeindex>
#include <typeinfo>



void BaseAction::Configure(const std::vector<std::unique_ptr<StateComponent>>& goal)
{
	requirements = GenerateRequirements(goal);
	results = GenerateResults(goal);
}

float BaseAction::GetCost() const { return 1.0f; }


namespace StateComponentUtils
{
	std::vector<std::unique_ptr<StateComponent>> CombineComponentLists(
		const std::vector<std::unique_ptr<StateComponent>>& baseComponents,
		const std::vector<std::unique_ptr<StateComponent>>& addComponents)
	{
		std::map<std::type_index, std::unique_ptr<StateComponent>> componentTypeToUpdatedValue;

		// Add base components to the map
		for (const auto& baseComponent : baseComponents)
		{
			std::type_index typeIndex = typeid(*baseComponent);
			componentTypeToUpdatedValue[typeIndex] = baseComponent->Clone();
		}

		// Add or update components based on addComponents
		for (const auto& addComponent : addComponents)
		{
			std::type_index typeIndex = typeid(*addComponent);
			if (componentTypeToUpdatedValue.count(typeIndex))
			{
				// If component of this type already exists, add to it
				componentTypeToUpdatedValue[typeIndex]->AddValues(*addComponent);
			}
			else
			{
				// Otherwise, clone and add the new component
				componentTypeToUpdatedValue[typeIndex] = addComponent->Clone();
			}
		}

		// Move unique_ptr values from the map to the vector
		std::vector<std::unique_ptr<StateComponent>> combinedComponents;
		combinedComponents.reserve(componentTypeToUpdatedValue.size());
		for (auto& [typeIndex, component] : componentTypeToUpdatedValue)
		{
			combinedComponents.push_back(std::move(component));
		}

		return combinedComponents;
	}

	std::vector<std::unique_ptr<StateComponent>> RemoveComponentList(
		const std::vector<std::unique_ptr<StateComponent>>& baseComponents,
		const std::vector<std::unique_ptr<StateComponent>>& componentsToRemove)
	{
		std::vector<std::unique_ptr<StateComponent>> updatedComponents;

		for (const auto& baseComponent : baseComponents)
		{
			bool toRemove = false;
			std::unique_ptr<StateComponent> clonedBase = baseComponent->Clone(); // Deep copy

			for (const auto& removeComponent : componentsToRemove)
			{
				if (typeid(*baseComponent) == typeid(*removeComponent))
				{
					clonedBase->SubtractValues(*removeComponent);
					toRemove = true;
					break;
				}
			}

			if (!toRemove || !clonedBase->IsEmpty())
			{
				updatedComponents.push_back(std::move(clonedBase));
			}
		}

		return updatedComponents;
	}
} // namespace GoalUtils



#pragma region A*_Node_and_Supporting_Structures
//=============================================================================
// A* Node and Supporting Structures
//=============================================================================
 GoapNode::GoapNode(std::vector<std::unique_ptr<StateComponent>>&& state,
	         std::vector<std::unique_ptr<StateComponent>>&& goal)
		: currentState(std::move(state)), currentGoal(std::move(goal))
{
}

// Helper to calculate fCost
void GoapNode::CalculateFCost()
{
	fCost = gCost + hCost;
}

// Helper to clone a state vector
std::vector<std::unique_ptr<StateComponent>> GoapNode::CloneState(
	const std::vector<std::unique_ptr<StateComponent>>& state)
{
	std::vector<std::unique_ptr<StateComponent>> newState;
	newState.reserve(state.size());
	for (const auto& comp : state)
	{
		newState.push_back(comp->Clone());
	}
	return newState;
}


// Comparator for A* priority queue (min-heap on fCost)

	bool CompareGoapNodes::operator()(const std::shared_ptr<GoapNode>& a, const std::shared_ptr<GoapNode>& b) const
	{
		// Node with lower fCost has higher priority
		if (std::abs(a->fCost - b->fCost) > 1e-6)
		{
			// Tolerance for float comparison
			return a->fCost > b->fCost;
		}
		// Tie-breaking (e.g., prefer lower hCost) can be added here
		return a->hCost > b->hCost;
	}


// Define the heuristic function signature
/*using HeuristicFunction = std::function<float(const std::vector<std::unique_ptr<StateComponent>>& /*state#1#,
                                              const std::vector<std::unique_ptr<StateComponent>>& /*goal#1#)>;*/

#pragma endregion






