#include "SAGOAP.h"
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

    // A global pointer for debugger access ONLY.
    // This is not thread-safe and should only be used for debugging sessions.
    const StateTypeRegistry* g_pDebugRegistry = nullptr;

    // A static buffer to hold the string for the debugger.
    // The debugger will read from this before it's overwritten.
    thread_local char g_DebugStringBuffer[4096];

    // =============================================================================
    // Base Action
    // =============================================================================
    void BaseAction::Configure(const AgentState& currentState, const Goal& goal)
    {
        requirements = GenerateRequirements(currentState, goal);
        results = GenerateResults(currentState, goal);
    }

    std::string BaseAction::DebugToString(const StateTypeRegistry& registry) const
    {
        std::string s = "ACTION: " + GetName();
        s += "  - Requirements: " + Utils::DebugToString(requirements, registry) + "\n";
        s += "  - Results:      " + Utils::DebugToString(results, registry);
        return s;
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
    
    // =============================================================================
    // Utility Functions
    // =============================================================================
    namespace Utils
    {
        AgentState CombineStates(const AgentState& base, const AgentState& delta, const StateTypeRegistry& registry)
        {
            AgentState new_state = base;
            for (const auto& [type, delta_prop] : delta.properties)
            {
                const auto* funcs = registry.GetFunctions(type);
                if (!funcs || !funcs->add) continue;

                auto it = new_state.properties.find(type);
                if (it != new_state.properties.end())
                {
                    it->second = funcs->add(it->second, delta_prop);
                }
                else
                {
                    new_state.properties[type] = delta_prop;
                }
            }
            return new_state;
        }

        AgentState SubtractStates(const AgentState& base, const AgentState& to_subtract, const StateTypeRegistry& registry)
        {
            AgentState new_state = base;
            for (const auto& [type, sub_prop] : to_subtract.properties)
            {
                const auto* funcs = registry.GetFunctions(type);
                if (!funcs || !funcs->subtract) continue;

                auto it = new_state.properties.find(type);
                if (it != new_state.properties.end())
                {
                    // Apply the subtraction
                    it->second = funcs->subtract(it->second, sub_prop);
                    // Check if the property became empty and should be removed
                    if (funcs->is_empty && funcs->is_empty(it->second))
                    {
                        new_state.properties.erase(it);
                    }
                }
            }
            return new_state;
        }

        size_t GetStateHash(const AgentState& state, const StateTypeRegistry& registry)
        {
            size_t seed = 0;
            // Note: std::map is ordered, so iterating gives a consistent hash.
            for (const auto& [type, prop] : state.properties)
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

        std::string DebugToString(const AgentState& state, const StateTypeRegistry& registry)
        {
            if (state.properties.empty()) {
                return "{}";
            }

            std::string s = "{ ";
            for (auto it = state.properties.begin(); it != state.properties.end(); ++it) {
                const auto* funcs = registry.GetFunctions(it->first);
                if (funcs && funcs->to_string) {
                    s += funcs->to_string(it->second);
                } else {
                    s += "[" + std::string(it->first.name()) + ": (no toString registered)]";
                }
                if (std::next(it) != state.properties.end()) {
                    s += ", ";
                }
            }
            s += " }";
            return s;
        }
    } // namespace Utils

    namespace Debug
    {
        void SetRegistryForVisualization(const StateTypeRegistry* registry)
        {
            g_pDebugRegistry = registry;
        }
    }
} // namespace SAGOAP

// This pragma tells the Microsoft linker to ensure that the symbol for
// GetAgentStateDebugString is included in the final executable, even if
// it appears unreferenced. The name needs to match the C-style name.
#pragma comment(linker, "/include:GetAgentStateDebugString")

const char* GetAgentStateDebugString(const SAGOAP::AgentState* state)
{
    if (!state || !SAGOAP::g_pDebugRegistry) {
        strcpy_s(SAGOAP::g_DebugStringBuffer, "(no state or registry)");
        return SAGOAP::g_DebugStringBuffer;
    }

    // Use our existing utility function!
    std::string str = SAGOAP::Utils::DebugToString(*state, *SAGOAP::g_pDebugRegistry);
    
    // Copy the result into the global buffer for the debugger to read.
    strcpy_s(SAGOAP::g_DebugStringBuffer, str.c_str());

    return SAGOAP::g_DebugStringBuffer;
}




