#include "SAGOAP.h"
#include <queue>
#include <unordered_set>
#include <algorithm> // for std::reverse

namespace SAGOAP
{
    // =============================================================================
    // Type Registry
    // =============================================================================
    const StateTypeFunctions* StateTypeRegistry::GetFunctions(std::type_index type) const
    {
        auto it = registry.find(type);
        return it != registry.end() ? &it->second : nullptr;
    }

    // =============================================================================
    // Base Action
    // =============================================================================
    void BaseAction::Configure(const AgentState& currentState, const Goal& goal)
    {
        requirements = GenerateRequirements(currentState, goal);
        results = GenerateResults(currentState, goal);
    }

    // =============================================================================
    // A* Node and Supporting Structures
    // =============================================================================
    GoapNode::GoapNode(AgentState&& state, Goal&& goal)
        : currentState(std::move(state)), currentGoal(std::move(goal))
    {
    }

    void GoapNode::CalculateFCost()
    {
        fCost = gCost + hCost;
    }

    bool CompareGoapNodes::operator()(const std::shared_ptr<GoapNode>& a, const std::shared_ptr<GoapNode>& b) const
    {
        if (std::abs(a->fCost - b->fCost) > 1e-6)
        {
            return a->fCost > b->fCost;
        }
        return a->hCost > b->hCost;
    }

    // Hash function for a state-goal pair, used by the closed set
    size_t ComputeStateGoalHash(const AgentState& state, const Goal& goal, const StateTypeRegistry& registry)
    {
        size_t h1 = Utils::GetStateHash(state, registry);
        size_t h2 = Utils::GetStateHash(goal, registry);
        // Combine hashes (boost::hash_combine style)
        return h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
    }
    

    // Explicitly instantiate the template for any linker issues, or move to INL if preferred
    // For a library, it's often better to keep this in the .cpp and force the user to define their
    // action generators, or provide a header-only library. For now, this is fine.
    // template class GoapPlanner; // This may cause issues depending on compiler. Best to keep template def in header/inl.
    // Let's move the GoapPlanner::Plan definition into the .inl file as well to avoid linker issues with templates.


    // =============================================================================
    // Utility Functions
    // =============================================================================
    namespace Utils
    {
        AgentState CombineStates(const AgentState& base, const AgentState& delta, const StateTypeRegistry& registry)
        {
            AgentState new_state = base;
            for (const auto& [type, delta_prop] : delta)
            {
                const auto* funcs = registry.GetFunctions(type);
                if (!funcs || !funcs->add) continue;

                auto it = new_state.find(type);
                if (it != new_state.end())
                {
                    it->second = funcs->add(it->second, delta_prop);
                }
                else
                {
                    new_state[type] = delta_prop;
                }
            }
            return new_state;
        }

        AgentState SubtractStates(const AgentState& base, const AgentState& to_subtract, const StateTypeRegistry& registry)
        {
            AgentState new_state = base;
            for (const auto& [type, sub_prop] : to_subtract)
            {
                const auto* funcs = registry.GetFunctions(type);
                if (!funcs || !funcs->subtract) continue;

                auto it = new_state.find(type);
                if (it != new_state.end())
                {
                    // Apply the subtraction
                    it->second = funcs->subtract(it->second, sub_prop);
                    // Check if the property became empty and should be removed
                    if (funcs->is_empty && funcs->is_empty(it->second))
                    {
                        new_state.erase(it);
                    }
                }
            }
            return new_state;
        }

        size_t GetStateHash(const AgentState& state, const StateTypeRegistry& registry)
        {
            size_t seed = 0;
            // Note: std::map is ordered, so iterating gives a consistent hash.
            for (const auto& [type, prop] : state)
            {
                const auto* funcs = registry.GetFunctions(type);
                size_t prop_hash = 0;
                if (funcs && funcs->get_hash)
                {
                    prop_hash = funcs->get_hash(prop);
                }
                // Combine hashes
                seed ^= prop_hash + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
            return seed;
        }
    } // namespace Utils
} // namespace SAGOAP




