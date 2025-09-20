#include "SAHGOAP.h"
#include <algorithm> // for std::reverse
#include <queue> // For planner

namespace SAHGOAP
{
    const StateTypeFunctions* StateTypeRegistry::GetFunctions(std::type_index type) const
    {
        auto it = registry.find(type);
        return it != registry.end() ? &it->second : nullptr;
    }
    
    const StateTypeRegistry* g_pDebugRegistry = nullptr;
    thread_local char g_DebugStringBuffer[4096];

    // =============================================================================
    // Library-Provided Generic Tasks Implementation
    // =============================================================================
    bool AchieveStateTask::Decompose(const AgentState&, const StateTypeRegistry&, std::vector<std::unique_ptr<BaseTask>>&, std::unique_ptr<BaseAction>&) const {
        // This task is special. Its decomposition logic is handled directly inside the planner
        // because it needs access to the user's ActionGenerator. Returning false signals the
        // planner to handle it.
        return false;
    }
    std::unique_ptr<BaseTask> AchieveStateTask::Clone() const {
        return std::make_unique<AchieveStateTask>(AgentState(this->targetState));
    }
    std::string AchieveStateTask::GetName() const {
        return "AchieveState"; // TODO: Add state debug string
    }

    ExecuteActionTask::ExecuteActionTask(std::unique_ptr<BaseAction> action) : actionToExecute(std::move(action)) {}
    
    bool ExecuteActionTask::Decompose(const AgentState&, const StateTypeRegistry&, std::vector<std::unique_ptr<BaseTask>>&, std::unique_ptr<BaseAction>& primitiveAction) const {
        primitiveAction = actionToExecute->Clone();
        return true;
    }
    std::unique_ptr<BaseTask> ExecuteActionTask::Clone() const {
        return std::make_unique<ExecuteActionTask>(actionToExecute->Clone());
    }
    std::string ExecuteActionTask::GetName() const {
        return "Execute: " + actionToExecute->GetName();
    }
    
    // =============================================================================
    // Planner Node Implementation
    // =============================================================================
    PlannerNode::PlannerNode(AgentState&& state, Goal&& tasks, std::vector<std::unique_ptr<BaseAction>>&& plan)
        : currentState(std::move(state)), tasksRemaining(std::move(tasks)), planSoFar(std::move(plan)) {}
        
    void PlannerNode::CalculateFCost() { fCost = gCost + hCost; }
    
    bool ComparePlannerNodes::operator()(const std::shared_ptr<PlannerNode>& a, const std::shared_ptr<PlannerNode>& b) const {
        if (std::abs(a->fCost - b->fCost) > 1e-6) return a->fCost > b->fCost;
        return a->hCost > b->hCost;
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
                if (it != new_state.properties.end()) {
                    it->second = funcs->add(it->second, delta_prop);
                } else {
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
                if (it != new_state.properties.end()) {
                    it->second = funcs->subtract(it->second, sub_prop);
                    if (funcs->is_empty && funcs->is_empty(it->second)) {
                        new_state.properties.erase(it);
                    }
                }
            }
            return new_state;
        }

        bool IsStateSatisfyingGoal(const AgentState& state, const AgentState& goal, const StateTypeRegistry& registry)
        {
            AgentState remaining = SubtractStates(goal, state, registry);
            return remaining.properties.empty();
        }

        size_t GetStateHash(const AgentState& state, const StateTypeRegistry& registry)
        {
            size_t seed = 0;
            for (const auto& [type, prop] : state.properties)
            {
                const auto* funcs = registry.GetFunctions(type);
                size_t prop_hash = 0;
                if (funcs && funcs->get_hash) {
                    prop_hash = funcs->get_hash(prop);
                }
                seed ^= prop_hash + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
            return seed;
        }
        
        size_t GetGoalHash(const Goal& goal) {
            size_t seed = 0;
            for (const auto& task : goal) {
                 seed ^= std::hash<std::string>()(task->GetName()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
            return seed;
        }

        std::string DebugToString(const AgentState& state, const StateTypeRegistry& registry)
        {
            if (state.properties.empty()) return "{}";
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
    }
    
    namespace Debug {
        void SetRegistryForVisualization(const StateTypeRegistry* registry) {
            g_pDebugRegistry = registry;
        }
    }
}
extern "C" {
    #pragma comment(linker, "/include:GetAgentStateDebugString2")
    const char* GetAgentStateDebugString2(const SAHGOAP::AgentState* state) {
        if (!state || !SAHGOAP::g_pDebugRegistry) {
            strcpy_s(SAHGOAP::g_DebugStringBuffer, "(no state or registry)");
            return SAHGOAP::g_DebugStringBuffer;
        }
        std::string str = SAHGOAP::Utils::DebugToString(*state, *SAHGOAP::g_pDebugRegistry);
        strcpy_s(SAHGOAP::g_DebugStringBuffer, str.c_str());
        return SAHGOAP::g_DebugStringBuffer;
    }
}