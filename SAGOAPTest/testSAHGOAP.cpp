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

struct WorldKnowledgeComponent {
    struct ResourceInfo { int location_id; int quantity; };
    std::map<int, ResourceInfo> resources; // ItemID -> Info
};

struct RecipeKnowledgeComponent {
    struct Recipe {
        int productId;
        int locationId;
        std::map<int, int> ingredients;
        std::map<int, int> tools;
    };
    std::map<int, Recipe> recipes; // ProductID -> Recipe
};

struct FlagComponent {
    bool hasFlag = false;
};

// =============================================================================
// 2. HASHER STRUCTS FOR EACH COMPONENT
// =============================================================================

struct LocationComponentHasher {
    std::size_t operator()(const LocationComponent& comp) const {
        return std::hash<int>()(comp.location_id);
    }
};

struct InventoryComponentHasher {
    std::size_t operator()(const InventoryComponent& comp) const {
        size_t seed = comp.items.size();
        for (const auto& [key, value] : comp.items) {
            seed ^= std::hash<int>()(key) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            seed ^= std::hash<int>()(value) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

struct FlagComponentHasher {
    std::size_t operator()(const FlagComponent& comp) const {
        return std::hash<bool>()(comp.hasFlag);
    }
};

// =============================================================================
// TEST FIXTURE & TESTS
// =============================================================================

class SAHGOAP_Test : public ::testing::Test {
protected:
    SAHGOAP::WorldModel model;

    // Symbol IDs, stored for easy access in tests
    int loc_village, loc_forge, loc_mines;
    int item_sword, item_ingot, item_ore, item_wood, item_hammer;

    void SetUp() override {
        // --- A. Register Hasher functions ---
        model.RegisterComponentHasher<LocationComponent>(LocationComponentHasher{});
        model.RegisterComponentHasher<InventoryComponent>(InventoryComponentHasher{});
        model.RegisterComponentHasher<FlagComponent>(FlagComponentHasher{});
        // We will not hash knowledge components as they are static for these tests.

        // --- B. Register all game symbols ---
        model.RegisterSymbol("Village"); loc_village = model.GetSymbolId("Village");
        model.RegisterSymbol("Forge");   loc_forge = model.GetSymbolId("Forge");
        model.RegisterSymbol("Mines");   loc_mines = model.GetSymbolId("Mines");
        model.RegisterSymbol("Sword");   item_sword = model.GetSymbolId("Sword");
        model.RegisterSymbol("IronIngot"); item_ingot = model.GetSymbolId("IronIngot");
        model.RegisterSymbol("IronOre"); item_ore = model.GetSymbolId("IronOre");
        model.RegisterSymbol("Wood");    item_wood = model.GetSymbolId("Wood");
        model.RegisterSymbol("Hammer");  item_hammer = model.GetSymbolId("Hammer");
    }
};


TEST_F(SAHGOAP_Test, FindsOneStepPlan) {
    // --- 1. ARRANGE ---
    const SAHGOAP::ActionSchema SetFlagSchema = {
        "SetFlag", "SimpleGen", {}, 1.0f, {}, { {"Flag.Set", {"1"}} }
    };

    model.RegisterEffect<FlagComponent>("Flag.Set", 
        [](FlagComponent& comp, const std::vector<int>& params) {
            comp.hasFlag = (params.empty() || params[0] == 1);
        });

    model.RegisterCondition<FlagComponent>("Flag.IsSet", 
        [](const FlagComponent& comp, const std::vector<int>& params, SAHGOAP::ComparisonOperator op) {
            if (params.empty()) return false;
            bool target = (params[0] == 1);
            return op == SAHGOAP::ComparisonOperator::EqualTo && comp.hasFlag == target;
        });

    model.RegisterActionGenerator("SimpleGen", 
        [&](const SAHGOAP::AgentState& state, const SAHGOAP::StateGoal& goal, const SAHGOAP::WorldModel& model) {
            std::vector<SAHGOAP::ActionInstance> instances;
            for(const auto& cond : goal) {
                if (cond.name == "Flag.IsSet") {
                    instances.push_back({&SetFlagSchema, {}});
                    break;
                }
            }
            return instances;
    });

    SAHGOAP::AgentState initialState;
    initialState.AddComponent<FlagComponent>()->hasFlag = false;

    SAHGOAP::StateGoal goalConditions = {
        {"Flag.IsSet", {"1"}, SAHGOAP::ComparisonOperator::EqualTo}
    };
    
    SAHGOAP::Planner::HeuristicFunction heuristic = [](const SAHGOAP::AgentState&, const SAHGOAP::Goal& goal) {
        return (float)goal.size();
    };

    // --- 2. ACT ---
    SAHGOAP::Planner planner;
    model.RegisterActionSchema(SetFlagSchema);
    auto plan = planner.Plan(initialState, goalConditions, model, heuristic);

    // --- 3. ASSERT ---
    ASSERT_TRUE(plan.has_value()) << "Planner failed to find a solution.";
    ASSERT_EQ(plan->size(), 1);
    EXPECT_EQ((*plan)[0].schema->name, "SetFlag");
}


TEST_F(SAHGOAP_Test, Planner_CraftsSword) {
    // --- 1. ARRANGE ---

    // --- A. Register all Conditions, Effects, and Goal Appliers ---
    model.RegisterCondition<LocationComponent>("Location.Is", 
        [](const LocationComponent& comp, const std::vector<int>& params, SAHGOAP::ComparisonOperator op) {
            return op == SAHGOAP::ComparisonOperator::EqualTo && comp.location_id == params[0];
        });
    model.RegisterEffect<LocationComponent>("Location.Set",
        [](LocationComponent& comp, const std::vector<int>& params) {
            comp.location_id = params[0];
        });
    model.RegisterGoalApplier("Location.Is", 
        [](SAHGOAP::AgentState& goalState, const std::vector<int>& params) {
            goalState.AddComponent<LocationComponent>()->location_id = params[0];
        });

    model.RegisterCondition<InventoryComponent>("Inventory.Has",
        [](const InventoryComponent& comp, const std::vector<int>& params, SAHGOAP::ComparisonOperator op) {
            int item_id = params[0];
            int quantity = params[1];
            auto it = comp.items.find(item_id);
            int current_quant = (it == comp.items.end()) ? 0 : it->second;
            return op == SAHGOAP::ComparisonOperator::GreaterThanOrEqualTo && current_quant >= quantity;
        });
    model.RegisterEffect<InventoryComponent>("Inventory.Add",
        [](InventoryComponent& comp, const std::vector<int>& params) {
            comp.items[params[0]] += params[1];
        });
    model.RegisterGoalApplier("Inventory.Has",
        [](SAHGOAP::AgentState& goalState, const std::vector<int>& params) {
            goalState.AddComponent<InventoryComponent>()->items[params[0]] = params[1];
        });
    
    model.RegisterEffect<WorldKnowledgeComponent>("World.UpdateResource",
        [](WorldKnowledgeComponent& comp, const std::vector<int>& params) {
            comp.resources[params[0]].quantity += params[1];
        });

    // --- B. Register Action Instance Generators ---
    model.RegisterActionGenerator("MoveToGenerator", 
        [](const SAHGOAP::AgentState& state, const SAHGOAP::StateGoal& goal, const SAHGOAP::WorldModel& m) {
            std::vector<SAHGOAP::ActionInstance> instances;
            for (const auto& condition : goal) {
                if (condition.name == "Location.Is") {
                    int target_loc_id = std::stoi(condition.params[0]);
                    if (const auto* currentLoc = state.GetComponent<LocationComponent>()) {
                         if (target_loc_id != currentLoc->location_id) {
                            SAHGOAP::ActionInstance inst;
                            inst.params["targetLocation"] = target_loc_id;
                            instances.push_back(inst);
                            return instances;
                         }
                    }
                }
            }
            return instances;
        });
            
    model.RegisterActionGenerator("GetItemGenerator",
        [this](const SAHGOAP::AgentState& state, const SAHGOAP::StateGoal& goal, const SAHGOAP::WorldModel& m) {
            std::vector<SAHGOAP::ActionInstance> instances;
            const auto* world = state.GetComponent<WorldKnowledgeComponent>();
            if (!world) return instances;

            for (const auto& condition : goal) {
                if (condition.name == "Inventory.Has" && condition.op == SAHGOAP::ComparisonOperator::GreaterThanOrEqualTo) {
                    int item_id = std::stoi(condition.params[0]);
                    int needed_quant = std::stoi(condition.params[1]);
                    
                    int current_quant = 0;
                    if(const auto* inv = state.GetComponent<InventoryComponent>()) {
                        auto it = inv->items.find(item_id);
                        if (it != inv->items.end()) current_quant = it->second;
                    }

                    if (current_quant < needed_quant) {
                        auto res_it = world->resources.find(item_id);
                        if (res_it != world->resources.end() && res_it->second.quantity > 0) {
                            SAHGOAP::ActionInstance inst;
                            inst.params["itemToGet"] = item_id;
                            // This "smart" generator gets the total amount needed at once.
                            inst.params["quantityToGet"] = std::min(needed_quant - current_quant, res_it->second.quantity);
                            inst.params["sourceLocation"] = res_it->second.location_id;
                            instances.push_back(inst);
                        }
                    }
                }
            }
            return instances;
        });

    model.RegisterActionGenerator("CraftRecipeGenerator",
        [this](const SAHGOAP::AgentState& state, const SAHGOAP::StateGoal& goal, const SAHGOAP::WorldModel& m) {
            std::vector<SAHGOAP::ActionInstance> instances;
            const auto* recipes = state.GetComponent<RecipeKnowledgeComponent>();
            if (!recipes) return instances;
            
            for (const auto& condition : goal) {
                 if (condition.name == "Inventory.Has") {
                    int item_id = std::stoi(condition.params[0]);
                    auto recipe_it = recipes->recipes.find(item_id);
                    if (recipe_it != recipes->recipes.end()) {
                        SAHGOAP::ActionInstance inst;
                        inst.params["recipeId"] = item_id;
                        instances.push_back(inst);
                    }
                 }
            }
            return instances;
        });

    // --- C. Define Action Schemas ---
    SAHGOAP::ActionSchema movetoActionSchema = {"MoveTo", "MoveToGenerator", {"targetLocation"}, 5.0f, {}, {{"Location.Set", {"$targetLocation"}}}};
    SAHGOAP::ActionSchema getItemActionSchema = {"GetItem", "GetItemGenerator", {"itemToGet", "sourceLocation", "quantityToGet"}, 1.0f,
            {{"Location.Is", {"$sourceLocation"}, SAHGOAP::ComparisonOperator::EqualTo}},
            {
                    {"Inventory.Add", {"$itemToGet", "$quantityToGet"}},
                    {"World.UpdateResource", {"$itemToGet", "-$quantityToGet"}}
            }
    };
    SAHGOAP::ActionSchema craftRecipeActionSchema = {"Craft", "CraftRecipeGenerator", {"recipeId"}};
    model.RegisterActionSchema(movetoActionSchema);
    model.RegisterActionSchema(getItemActionSchema);
    model.RegisterActionSchema(craftRecipeActionSchema);
    // --- D. Define Initial State ---
    SAHGOAP::AgentState initialState;
    initialState.AddComponent<LocationComponent>()->location_id = loc_village;
    initialState.AddComponent<InventoryComponent>()->items = {{item_hammer, 1}, {item_wood, 2}};
    initialState.AddComponent<WorldKnowledgeComponent>()->resources = {{item_ore, {loc_mines, 2}}};
    auto& recipes = initialState.AddComponent<RecipeKnowledgeComponent>()->recipes;
    recipes[item_ingot] = {item_ingot, loc_forge, {{item_ore, 1}, {item_wood, 1}}, {}};
    recipes[item_sword] = {item_sword, loc_forge, {{item_ingot, 2}}, {{item_hammer, 1}}};

    // --- E. Define Goal ---
    SAHGOAP::StateGoal goalConditions = {
        {"Inventory.Has", {std::to_string(item_sword), "1"}, SAHGOAP::ComparisonOperator::GreaterThanOrEqualTo}
    };
    
    SAHGOAP::Planner::HeuristicFunction heuristic = [](const SAHGOAP::AgentState&, const SAHGOAP::Goal& goal) {
        return (float)goal.size();
    };

    // --- 2. ACT ---
    SAHGOAP::Planner planner;
    auto planResult = planner.Plan(initialState, goalConditions, model, heuristic);

    // --- 3. ASSERT ---
    ASSERT_TRUE(planResult.has_value()) << "Planner failed to find the sword crafting plan.";
    const auto& plan = *planResult;
    
    // The plan with a smart GetItem generator:
    // 1. Move(Mines)
    // 2. Get(Ore, 2)
    // 3. Move(Forge)
    // 4. Craft(Ingot) -> requires 1 ore, 1 wood
    // 5. Craft(Ingot) -> requires 1 ore, 1 wood
    // 6. Craft(Sword) -> requires 2 ingots, 1 hammer
    ASSERT_EQ(plan.size(), 6);

    EXPECT_EQ(plan[0].schema->name, "MoveTo");
    EXPECT_EQ(plan[0].params.at("targetLocation"), loc_mines);

    EXPECT_EQ(plan[1].schema->name, "GetItem");
    EXPECT_EQ(plan[1].params.at("itemToGet"), item_ore);
    EXPECT_EQ(plan[1].params.at("quantityToGet"), 2);

    EXPECT_EQ(plan[2].schema->name, "MoveTo");
    EXPECT_EQ(plan[2].params.at("targetLocation"), loc_forge);

    // The craft actions might be generic "Craft" schemas, so we check the recipeId param.
    EXPECT_EQ(plan[3].schema->name, "Craft");
    EXPECT_EQ(plan[3].params.at("recipeId"), item_ingot);

    EXPECT_EQ(plan[4].schema->name, "Craft");
    EXPECT_EQ(plan[4].params.at("recipeId"), item_ingot);

    EXPECT_EQ(plan[5].schema->name, "Craft");
    EXPECT_EQ(plan[5].params.at("recipeId"), item_sword);
}