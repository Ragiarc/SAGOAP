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
    PlannerNode::PlannerNode(AgentState&& state, Goal&& tasks)
        : currentState(std::move(state)), tasksRemaining(std::move(tasks)), parentAction(nullptr) {}
        
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
    // Start with a copy of the base (the goal). We will remove properties from this copy as they are satisfied.
    AgentState remaining_goal = base;

    // Use an iterator-based loop so we can safely erase elements from the map.
    for (auto goal_it = remaining_goal.properties.begin(); goal_it != remaining_goal.properties.end(); /* no increment */)
    {
        const std::type_index& type = goal_it->first;
        StateProperty& goal_prop = goal_it->second;

        const auto* funcs = registry.GetFunctions(type);
        if (!funcs || !funcs->subtract) {
            // This property type doesn't support subtraction, so we can't determine if it's met.
            // We'll assume it's not and leave it in the remaining_goal.
            ++goal_it;
            continue;
        }

        // Now, check if the state-to-subtract (the current world state) has this same property.
        auto current_state_it = to_subtract.properties.find(type);
        if (current_state_it != to_subtract.properties.end())
        {
            // Both the goal and the current state have this property. Perform the subtraction.
            const StateProperty& current_prop = current_state_it->second;
            goal_prop = funcs->subtract(goal_prop, current_prop); // e.g., LocationState("Forest").SubtractValues(LocationState("Village"))

            // After subtraction, check if the goal property is now "empty" (i.e., satisfied).
            if (funcs->is_empty && funcs->is_empty(goal_prop))
            {
                // It is satisfied, so remove it from the remaining_goal map.
                // erase() returns the iterator to the next element.
                goal_it = remaining_goal.properties.erase(goal_it);
            }
            else {
                // The property is not fully satisfied, so leave it and move to the next one.
                ++goal_it;
            }
        }
        else
        {
            // The current state doesn't have this property at all, so it's definitely not satisfied.
            // Leave it in the remaining_goal and move to the next one.
            ++goal_it;
        }
    }

    return remaining_goal;
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
        
        size_t GetGoalHash(const Goal& goal, const StateTypeRegistry& registry)
        {
            size_t seed = 0;
            for (const auto& task : goal)
            {
                size_t task_hash = 0;

                // Use dynamic_cast to check if the current task is the special library-provided type.
                if (const auto* achieveTask = dynamic_cast<const AchieveStateTask*>(task.get()))
                {
                    // It IS an AchieveStateTask. We must hash its internal targetState
                    // to get a unique value for this specific goal.
                    task_hash = GetStateHash(achieveTask->targetState, registry);
                }
                else
                {
                    // It is some other user-defined task. The best we can do is fall back
                    // to hashing its name, assuming the user gives unique names to
                    // different task types.
                    task_hash = std::hash<std::string>()(task->GetName());
                }

                // Combine the hash for this task into the total seed for the goal.
                seed ^= task_hash + 0x9e3779b9 + (seed << 6) + (seed >> 2);
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