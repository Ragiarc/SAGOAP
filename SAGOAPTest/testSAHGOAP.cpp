#include "gtest/gtest.h"
#include "SAHGOAP.h"

// =============================================================================
// 1. DEVELOPER-DEFINED COMPONENTS & GAME DATA
// =============================================================================

struct LocationComponent {
    int location_id = -1;
};

struct InventoryComponent {
    std::map<int, int> items; // ItemID -> Quantity
};

struct KnowledgeComponent {
    // For simplicity, we'll store recipes here. A real game might have a global manager.
    std::vector<struct Recipe> recipes;
};

struct WorldResourceComponent {
    struct ResourceInfo { int location_id; int quantity; };
    std::map<int, ResourceInfo> resources; // ItemID -> Info
};

struct Recipe {
    int productId;
    int locationId;
    std::map<int, int> ingredients;
    std::map<int, int> tools;
};

const SAHGOAP::ActionSchema SetFlagSchema = {
    "SetFlag",      // name
    "SimpleGen",    // generator_name
    {},             // param_names
    1.0f,           // cost
    {},             // preconditions
    {               // effects
            {"Flag.Set", {"1"}}
    }
};

struct FlagComponent {
    bool hasFlag = false;
};

// =============================================================================
// TEST FIXTURE & TESTS
// =============================================================================

TEST(HierarchicalPlanner, FindsOneStepPlan) {
    // --- 1. ARRANGE ---
    SAHGOAP::WorldModel model;
    
    // Register the component and its systems
    model.RegisterEffect<FlagComponent>("Flag.Set", 
        [](FlagComponent& comp, const std::vector<int>& params) {
            if (!params.empty()) {
                comp.hasFlag = (params[0] == 1);
            }
        });

    model.RegisterCondition<FlagComponent>("Flag.IsSet", 
        [](const FlagComponent& comp, const std::vector<int>& params, SAHGOAP::ComparisonOperator op) {
            if (params.empty()) return false;
            bool target = (params[0] == 1);
            if (op == SAHGOAP::ComparisonOperator::EqualTo) {
                return comp.hasFlag == target;
            }
            return false;
        });

    // Register a generator that will provide the SetFlagAction when needed.
    model.RegisterActionGenerator("SimpleGen", 
        [](const SAHGOAP::AgentState& state, const SAHGOAP::StateGoal& goal, const SAHGOAP::WorldModel& model) {
            std::vector<SAHGOAP::ActionInstance> instances;
            // This generator is relevant if the goal is to set the flag.
            for(const auto& cond : goal) {
                if (cond.name == "Flag.IsSet") {
                    instances.push_back({&SetFlagSchema, {}});
                    break; // Found our relevance, no need to check other conditions.
                }
            }
            return instances;
    });

    // Define initial and goal states
    SAHGOAP::AgentState initialState;
    initialState.AddComponent<FlagComponent>()->hasFlag = false;

    SAHGOAP::StateGoal goalConditions = {
        {"Flag.IsSet", {"1"}, SAHGOAP::ComparisonOperator::EqualTo}
    };
    
    SAHGOAP::Planner::HeuristicFunction heuristic = [](const SAHGOAP::AgentState&, const SAHGOAP::Goal& goal) {
        return (float)goal.size();
    };

    model.RegisterActionSchema(SetFlagSchema);

    // --- 2. ACT ---
    SAHGOAP::Planner planner;
    // Create a vector of the schemas available to the planner.
    std::vector<SAHGOAP::ActionSchema> allSchemas = {SetFlagSchema};
    auto plan = planner.Plan(initialState, goalConditions, model, heuristic);

    // --- 3. ASSERT ---
    ASSERT_TRUE(plan.has_value()) << "Planner failed to find a solution.";
    ASSERT_EQ(plan->size(), 1);
    EXPECT_EQ((*plan)[0].schema->name, "SetFlag");
}
