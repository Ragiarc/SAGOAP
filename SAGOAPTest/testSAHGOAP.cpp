#include <chrono>
#include "gtest/gtest.h"
#include "SAHGOAP.h"
#include "nlohmann/json.hpp"


using json = nlohmann::json;

json SerializeNode(const std::shared_ptr<SAHGOAP::internal::PlannerNode>& node, const SAHGOAP::WorldModel& worldModel) {
    json result;
    result["id"] = std::to_string(node->GetHash(worldModel));
    
    result["tasksRemaining"] = node->tasksRemaining.size();
    result["gCost"] = node->gCost;
    result["hCost"] = node->hCost;
    result["fCost"] = node->fCost;

    if (node->parent) {
        try {
            result["parentId"] = std::to_string(node->parent->GetHash(worldModel));
            std::string topTaskName;

            if (node->tasksRemaining.empty()) {
                topTaskName = "[GOAL]";
            } else {
                std::ostringstream oss;
                bool first = true;
                for (const auto& task : node->tasksRemaining) {
                    if (!first) oss << " -> "; // separator
                    oss << task->GetName();
                    first = false;
                }
                topTaskName = oss.str();
            }
            result["action"] = topTaskName;
        } catch (...) {
            result["parentId"] = nullptr;
            result["action"] = nullptr;
        }
    } else {
        result["parentId"] = nullptr;
        result["action"] = nullptr;
    }

    return result;
}

json SerializeGraph(const std::vector<std::shared_ptr<SAHGOAP::internal::PlannerNode>>& nodes, const SAHGOAP::WorldModel& worldModel) {
    json graph = json::array();
    for (auto& n : nodes)
    {
        graph.push_back(SerializeNode(n, worldModel));
    }
    return graph;
}
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
    std::map<int, float> item_bootstrap_costs; 
};

struct RecipeKnowledgeComponent {
    struct Recipe {
        int productId;
        int locationId;
        std::map<int, int> ingredients;
        std::map<int, int> tools;
        int cost;
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
    /*const SAHGOAP::ActionSchema SetFlagSchema = {
        "SetFlag", "SimpleGen", {}, 1, {}, { {"Flag.Set", {"1"}} }
    };*/

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

    model.RegisterCondition<WorldKnowledgeComponent>("World.HasResource",
    [](const WorldKnowledgeComponent& comp, const std::vector<int>& params, SAHGOAP::ComparisonOperator op) {
        if (params.size() < 2) return false;
        int item_id = params[0];
        int quantity = params[1];

        auto it = comp.resources.find(item_id);
        int current_quant = (it == comp.resources.end()) ? 0 : it->second.quantity;

        if (op == SAHGOAP::ComparisonOperator::GreaterThanOrEqualTo) {
            return current_quant >= quantity;
        }
        // ... other operators could be implemented here ...
        return false;
    });

    model.RegisterEffect<WorldKnowledgeComponent>("World.UpdateResource",
    [](WorldKnowledgeComponent& comp, const std::vector<int>& params) {
        if (params.size() < 2) return;
        int item_id = params[0];
        int quantity_change = params[1];
        
        // This will correctly add a negative number, depleting the resource.
        comp.resources[item_id].quantity += quantity_change;
    });

    model.RegisterActionGenerator(
        [&](const SAHGOAP::AgentState& state, const SAHGOAP::StateGoal& goal, const SAHGOAP::WorldModel& model) {
            std::vector<SAHGOAP::ActionInstance> instances;

            // This generator is relevant if the goal is to set the flag.
            for(const auto& cond : goal) {
                if (cond.name == "Flag.IsSet") {
                    
                    // --- Build the complete ActionInstance ---
                    SAHGOAP::ActionInstance inst;
                    
                    // 1. Copy the data from the schema template.
                    inst.name = "SetFlag";
                    inst.cost = 1;
                    inst.preconditions = {}; // Copy preconditions
                    inst.effects = { {"Flag.Set", {"1"}}};         // Copy effects
                    
                    // 2. This action has no parameters, so its params map is empty.
                    inst.params = {};

                    instances.push_back(std::move(inst));
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
    auto start_time = std::chrono::high_resolution_clock::now();
    auto plan = planner.Plan(initialState, goalConditions, model, heuristic, NULL);
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    std::cout << duration.count() << " microseconds" << std::endl;

    // --- 3. ASSERT ---
    ASSERT_TRUE(plan.has_value()) << "Planner failed to find a solution.";
    ASSERT_EQ(plan->size(), 1);
    EXPECT_EQ((*plan)[0].name, "SetFlag");
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
    model.RegisterActionGenerator(
        [](const SAHGOAP::AgentState& state, const SAHGOAP::StateGoal& goal, const SAHGOAP::WorldModel& m) {
            std::vector<SAHGOAP::ActionInstance> instances;
            for (const auto& condition : goal) {
                if (condition.name == "Location.Is") {
                    int target_loc_id = std::stoi(condition.params[0]);
                    if (const auto* currentLoc = state.GetComponent<LocationComponent>()) {
                         if (target_loc_id != currentLoc->location_id) {
                             SAHGOAP::ActionInstance inst;
                             inst.name = "MoveTo";
                             inst.cost = 10; // for now
                             inst.params["targetLocation"] = target_loc_id;
                             // It copies the preconditions and effects from the schema.
                             inst.preconditions = {}; 
                             inst.effects = {{"Location.Set", {"$targetLocation"}}};
                             instances.push_back(inst);
                            return instances;
                         }
                    }
                }
            }
            return instances;
        });
            
    model.RegisterActionGenerator(
    [this](const SAHGOAP::AgentState& state, const SAHGOAP::StateGoal& goal, const SAHGOAP::WorldModel& m) {
        std::vector<SAHGOAP::ActionInstance> instances;

        const auto* world = state.GetComponent<WorldKnowledgeComponent>();
        if (!world) return instances;

        // Look for an "Inventory.Has" condition in the goal.
        for (const auto& condition : goal) {
            if (condition.name == "Inventory.Has" && condition.op == SAHGOAP::ComparisonOperator::GreaterThanOrEqualTo) {
                // Assuming parameter at index 0 is the item ID
                int item_id = std::stoi(condition.params[0]);
                int needed_quant = std::stoi(condition.params[1]);
                
                int current_quant = 0;
                if(const auto* inv = state.GetComponent<InventoryComponent>()) {
                    auto it = inv->items.find(item_id);
                    if (it != inv->items.end()) current_quant = it->second;
                }

                // Is the goal for this item already met? If so, we don't need to generate an action.
                if (current_quant >= needed_quant) {
                    continue;
                }

                // We need this item. Check if the world has it.
                auto res_it = world->resources.find(item_id);
                if (res_it != world->resources.end() && res_it->second.quantity > 0) {
                    int quantityToGet = std::min(needed_quant - current_quant, res_it->second.quantity);
                    // --- Build the complete ActionInstance ---
                    SAHGOAP::ActionInstance inst;

                    // 1. Copy the template data from the schema
                    inst.name = "GetItem";
                    inst.cost = 1;
                    inst.preconditions.push_back({
                        "Location.Is", 
                        {std::to_string(res_it->second.location_id)}, 
                        SAHGOAP::ComparisonOperator::EqualTo
                    });
                    inst.effects.push_back({
                        "Inventory.Add", 
                        {std::to_string(item_id), std::to_string(quantityToGet)} // Use parameter references
                    });
                    inst.effects.push_back({
                        "World.UpdateResource",
                        {"$itemToGet", std::to_string(-quantityToGet)} // Effect can be a literal value
                    });
                    
                    // 2. Fill in the specific parameters for THIS instance
                    inst.params["itemToGet"] = item_id;
                    inst.params["quantityToGet"] = quantityToGet;
                    inst.params["sourceLocation"] = res_it->second.location_id;
                    
                    instances.push_back(std::move(inst));
                }
            }
        }
        return instances;
    });

   model.RegisterActionGenerator(
    [this](const SAHGOAP::AgentState& state, const SAHGOAP::StateGoal& goal, const SAHGOAP::WorldModel& m) {
        std::vector<SAHGOAP::ActionInstance> instances;
        const auto* recipes = state.GetComponent<RecipeKnowledgeComponent>();
        const auto* world = state.GetComponent<WorldKnowledgeComponent>(); 
        if (!recipes || !world) return instances;

        for (const auto& condition : goal) {
             if (condition.name == "Inventory.Has") {
                int item_id_to_craft = std::stoi(condition.params[0]);
                
                auto recipe_it = recipes->recipes.find(item_id_to_craft);
                if (recipe_it != recipes->recipes.end()) {
                    const auto& recipe = recipe_it->second;
                    
                    SAHGOAP::ActionInstance inst;
                    inst.name = "Craft(" + m.GetSymbolName(recipe.productId) + ")"; // e.g., "Craft(IronIngot)"
                    inst.cost = recipe.cost;
                    inst.params["recipeId"] = recipe.productId;

                    int calculated_precondition_cost = 0;
                    // Cost of ingredients
                    for (const auto& [ing_id, quant] : recipe.ingredients) {
                        if (world->item_bootstrap_costs.count(ing_id)) {
                            calculated_precondition_cost += quant * world->item_bootstrap_costs.at(ing_id);
                        }
                    }

                    // --- DYNAMICALLY BUILD PRECONDITIONS AND EFFECTS ---
                    // It builds the lists from scratch, it doesn't use the schema's lists.
                    inst.preconditions.push_back({"Location.Is", {std::to_string(recipe.locationId)}});
                    for (const auto& [ing_id, quant] : recipe.ingredients) {
                        inst.preconditions.push_back({"Inventory.Has", {std::to_string(ing_id), std::to_string(quant)}, SAHGOAP::ComparisonOperator::GreaterThanOrEqualTo});
                    }
                    inst.effects.push_back({"Inventory.Add", {std::to_string(recipe.productId), "1"}});
                    for (const auto& [ing_id, quant] : recipe.ingredients) {
                        inst.effects.push_back({"Inventory.Add", {std::to_string(ing_id), std::to_string(-quant)}});
                    }
                    
                    instances.push_back(std::move(inst));
                }
             }
        }
        return instances;
    });
    // --- C. Define Action Schemas ---
    /*SAHGOAP::ActionSchema movetoActionSchema = {"MoveTo", "MoveToGenerator", {"targetLocation"}, 5, {}, {{"Location.Set", {"$targetLocation"}}}};
    SAHGOAP::ActionSchema getItemActionSchema = {"GetItem", "GetItemGenerator", {"itemToGet", "sourceLocation", "quantityToGet"}, 1,
            {{"Location.Is", {"$sourceLocation"}, SAHGOAP::ComparisonOperator::EqualTo}},
            {
                    {"Inventory.Add", {"$itemToGet", "$quantityToGet"}},
                    {"World.UpdateResource", {"$itemToGet", "-$quantityToGet"}}
            }
    };
    SAHGOAP::ActionSchema craftRecipeActionSchema = {"Craft", "CraftRecipeGenerator", {"recipeId"}};
    model.RegisterActionSchema(movetoActionSchema);
    model.RegisterActionSchema(getItemActionSchema);
    model.RegisterActionSchema(craftRecipeActionSchema);*/
    // --- D. Define Initial State ---
    SAHGOAP::AgentState initialState;
    initialState.AddComponent<LocationComponent>()->location_id = loc_village;
    initialState.AddComponent<InventoryComponent>()->items = {{item_hammer, 1}, {item_wood, 2}};
    auto& world_knowledge = *initialState.AddComponent<WorldKnowledgeComponent>();
    world_knowledge.resources = {{item_ore, {loc_mines, 2}}};
    auto& costs = world_knowledge.item_bootstrap_costs;
    // Cost of IronOre = MoveTo(Mines) cost + GetItem cost = 10 + 1 = 11
    costs[item_ore] = 11.0f; 
    // Cost of Wood = (Assuming a GetItem from a forest location) = ~11
    costs[item_wood] = 11.0f;

    // Cost of IronIngot = Cost(1 Ore) + Cost(1 Wood) + Cost(Move to Forge) + Craft Cost
    //                   = 11          + 11           + 10                    + 2 = 34
    costs[item_ingot] = 34.0f;

    // Cost of Sword = Cost(2 Ingots) + Have Hammer (0) + Cost(Move to Forge) + Craft Cost
    //               = 2 * 34         + 0               + 10                    + 4 = 82
    costs[item_sword] = 82.0f;
    auto& recipes = initialState.AddComponent<RecipeKnowledgeComponent>()->recipes;
    recipes[item_ingot] = {item_ingot, loc_forge, {{item_ore, 1}, {item_wood, 1}}, {}, 2};
    recipes[item_sword] = {item_sword, loc_forge, {{item_ingot, 2}}, {{item_hammer, 1}}, 4};
    
    

    // --- E. Define Goal ---
    SAHGOAP::StateGoal goalConditions = {
        {"Inventory.Has", {std::to_string(item_sword), "1"}, SAHGOAP::ComparisonOperator::GreaterThanOrEqualTo}
    };
    
    SAHGOAP::Planner::HeuristicFunction heuristic = [](const SAHGOAP::AgentState&, const SAHGOAP::Goal& goal) {
        return (float)goal.size();
    };

    // --- 2. ACT ---
    std::vector<std::shared_ptr<SAHGOAP::internal::PlannerNode>> nodesCollected;
    SAHGOAP::Planner planner;
    auto start_time = std::chrono::high_resolution_clock::now();
    auto planResult = planner.Plan(initialState, goalConditions, model, heuristic,
        [&](const std::shared_ptr<SAHGOAP::internal::PlannerNode>& node) {
        nodesCollected.push_back(node);
    });
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    std::cout << duration.count() << " microseconds" << std::endl;

    std::cout << SerializeGraph(nodesCollected, model).dump();
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
    ASSERT_EQ(plan.size(), 7);

    EXPECT_EQ(plan[0].name, "MoveTo");
    EXPECT_EQ(plan[0].params.at("targetLocation"), loc_mines);

    EXPECT_EQ(plan[1].name, "GetItem");
    EXPECT_EQ(plan[1].params.at("itemToGet"), item_ore);
    EXPECT_EQ(plan[1].params.at("quantityToGet"), 1);

    EXPECT_EQ(plan[2].name, "GetItem");
    EXPECT_EQ(plan[2].params.at("itemToGet"), item_ore);
    EXPECT_EQ(plan[2].params.at("quantityToGet"), 1);

    EXPECT_EQ(plan[3].name, "MoveTo");
    EXPECT_EQ(plan[3].params.at("targetLocation"), loc_forge);

    // The craft actions might be generic "Craft" schemas, so we check the recipeId param.
    EXPECT_EQ(plan[4].name, "Craft(IronIngot)");
    EXPECT_EQ(plan[4].params.at("recipeId"), item_ingot);

    EXPECT_EQ(plan[5].name, "Craft(IronIngot)");
    EXPECT_EQ(plan[5].params.at("recipeId"), item_ingot);

    EXPECT_EQ(plan[6].name, "Craft(IronIngot)");
    EXPECT_EQ(plan[6].params.at("recipeId"), item_sword);
}


TEST_F(SAHGOAP_Test, Planner_GetOreInCorrectOrder) {
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
    model.RegisterActionGenerator(
        [](const SAHGOAP::AgentState& state, const SAHGOAP::StateGoal& goal, const SAHGOAP::WorldModel& m) {
            std::vector<SAHGOAP::ActionInstance> instances;
            for (const auto& condition : goal) {
                if (condition.name == "Location.Is") {
                    int target_loc_id = std::stoi(condition.params[0]);
                    if (const auto* currentLoc = state.GetComponent<LocationComponent>()) {
                         if (target_loc_id != currentLoc->location_id) {
                             SAHGOAP::ActionInstance inst;
                             inst.name = "MoveTo";
                             inst.cost = 10; // for now
                             inst.params["targetLocation"] = target_loc_id;
                             // It copies the preconditions and effects from the schema.
                             inst.preconditions = {}; 
                             inst.effects = {{"Location.Set", {"$targetLocation"}}};
                             instances.push_back(inst);
                            return instances;
                         }
                    }
                }
            }
            return instances;
        });
            
    model.RegisterActionGenerator(
    [this](const SAHGOAP::AgentState& state, const SAHGOAP::StateGoal& goal, const SAHGOAP::WorldModel& m) {
        std::vector<SAHGOAP::ActionInstance> instances;

        const auto* world = state.GetComponent<WorldKnowledgeComponent>();
        if (!world) return instances;

        // Look for an "Inventory.Has" condition in the goal.
        for (const auto& condition : goal) {
            if (condition.name == "Inventory.Has" && condition.op == SAHGOAP::ComparisonOperator::GreaterThanOrEqualTo) {
                // Assuming parameter at index 0 is the item ID
                int item_id = std::stoi(condition.params[0]);
                int needed_quant = std::stoi(condition.params[1]);
                
                int current_quant = 0;
                if(const auto* inv = state.GetComponent<InventoryComponent>()) {
                    auto it = inv->items.find(item_id);
                    if (it != inv->items.end()) current_quant = it->second;
                }

                // Is the goal for this item already met? If so, we don't need to generate an action.
                if (current_quant >= needed_quant) {
                    continue;
                }

                // We need this item. Check if the world has it.
                auto res_it = world->resources.find(item_id);
                if (res_it != world->resources.end() && res_it->second.quantity > 0) {
                    int quantityToGet = std::min(needed_quant - current_quant, res_it->second.quantity);
                    // --- Build the complete ActionInstance ---
                    SAHGOAP::ActionInstance inst;

                    // 1. Copy the template data from the schema
                    inst.name = "GetItem";
                    inst.cost = 1;
                    inst.preconditions.push_back({
                        "Location.Is", 
                        {std::to_string(res_it->second.location_id)}, 
                        SAHGOAP::ComparisonOperator::EqualTo
                    });
                    inst.effects.push_back({
                        "Inventory.Add", 
                        {std::to_string(item_id), std::to_string(quantityToGet)} // Use parameter references
                    });
                    inst.effects.push_back({
                        "World.UpdateResource",
                        {"$itemToGet", std::to_string(-quantityToGet)} // Effect can be a literal value
                    });
                    
                    // 2. Fill in the specific parameters for THIS instance
                    inst.params["itemToGet"] = item_id;
                    inst.params["quantityToGet"] = quantityToGet;
                    inst.params["sourceLocation"] = res_it->second.location_id;
                    
                    instances.push_back(std::move(inst));
                }
            }
        }
        return instances;
    });

   model.RegisterActionGenerator(
    [this](const SAHGOAP::AgentState& state, const SAHGOAP::StateGoal& goal, const SAHGOAP::WorldModel& m) {
        std::vector<SAHGOAP::ActionInstance> instances;
        const auto* recipes = state.GetComponent<RecipeKnowledgeComponent>();
        const auto* world = state.GetComponent<WorldKnowledgeComponent>(); 
        if (!recipes || !world) return instances;

        for (const auto& condition : goal) {
             if (condition.name == "Inventory.Has") {
                int item_id_to_craft = std::stoi(condition.params[0]);
                
                auto recipe_it = recipes->recipes.find(item_id_to_craft);
                if (recipe_it != recipes->recipes.end()) {
                    const auto& recipe = recipe_it->second;
                    
                    SAHGOAP::ActionInstance inst;
                    inst.name = "Craft(" + m.GetSymbolName(recipe.productId) + ")"; // e.g., "Craft(IronIngot)"
                    inst.cost = recipe.cost;
                    inst.params["recipeId"] = recipe.productId;

                    int calculated_precondition_cost = 0;
                    // Cost of ingredients
                    for (const auto& [ing_id, quant] : recipe.ingredients) {
                        if (world->item_bootstrap_costs.count(ing_id)) {
                            calculated_precondition_cost += quant * world->item_bootstrap_costs.at(ing_id);
                        }
                    }

                    // --- DYNAMICALLY BUILD PRECONDITIONS AND EFFECTS ---
                    // It builds the lists from scratch, it doesn't use the schema's lists.
                    inst.preconditions.push_back({"Location.Is", {std::to_string(recipe.locationId)}});
                    for (const auto& [ing_id, quant] : recipe.ingredients) {
                        inst.preconditions.push_back({"Inventory.Has", {std::to_string(ing_id), std::to_string(quant)}, SAHGOAP::ComparisonOperator::GreaterThanOrEqualTo});
                    }
                    inst.effects.push_back({"Inventory.Add", {std::to_string(recipe.productId), "1"}});
                    for (const auto& [ing_id, quant] : recipe.ingredients) {
                        inst.effects.push_back({"Inventory.Add", {std::to_string(ing_id), std::to_string(-quant)}});
                    }
                    
                    instances.push_back(std::move(inst));
                }
             }
        }
        return instances;
    });

    SAHGOAP::AgentState initialState;
    initialState.AddComponent<LocationComponent>()->location_id = loc_village;
    initialState.AddComponent<InventoryComponent>()->items = {{item_hammer, 1}, {item_wood, 2}};
    auto& world_knowledge = *initialState.AddComponent<WorldKnowledgeComponent>();
    world_knowledge.resources = {{item_ore, {loc_mines, 2}}};
    auto& costs = world_knowledge.item_bootstrap_costs;
    // Cost of IronOre = MoveTo(Mines) cost + GetItem cost = 10 + 1 = 11
    costs[item_ore] = 11.0f; 
    // Cost of Wood = (Assuming a GetItem from a forest location) = ~11
    costs[item_wood] = 11.0f;

    // Cost of IronIngot = Cost(1 Ore) + Cost(1 Wood) + Cost(Move to Forge) + Craft Cost
    //                   = 11          + 11           + 10                    + 2 = 34
    costs[item_ingot] = 34.0f;

    // Cost of Sword = Cost(2 Ingots) + Have Hammer (0) + Cost(Move to Forge) + Craft Cost
    //               = 2 * 34         + 0               + 10                    + 4 = 82
    costs[item_sword] = 82.0f;
    auto& recipes = initialState.AddComponent<RecipeKnowledgeComponent>()->recipes;
    recipes[item_ingot] = {item_ingot, loc_forge, {{item_ore, 1}, {item_wood, 1}}, {}, 2};
    recipes[item_sword] = {item_sword, loc_forge, {{item_ingot, 2}}, {{item_hammer, 1}}, 4};
    
    

    // --- E. Define Goal ---
    SAHGOAP::StateGoal goalConditions = {
        {"Inventory.Has", {std::to_string(item_ingot), "2"}, SAHGOAP::ComparisonOperator::GreaterThanOrEqualTo}
    };
    
    SAHGOAP::Planner::HeuristicFunction heuristic = [](const SAHGOAP::AgentState&, const SAHGOAP::Goal& goal) {
        return (float)goal.size();
    };

    // --- 2. ACT ---
    std::vector<std::shared_ptr<SAHGOAP::internal::PlannerNode>> nodesCollected;
    SAHGOAP::Planner planner;
    auto start_time = std::chrono::high_resolution_clock::now();
    auto planResult = planner.Plan(initialState, goalConditions, model, heuristic,
        [&](const std::shared_ptr<SAHGOAP::internal::PlannerNode>& node) {
        nodesCollected.push_back(node);
    });
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    std::cout << duration.count() << " microseconds" << std::endl;

    std::cout << SerializeGraph(nodesCollected, model).dump();
    // --- 3. ASSERT ---
    ASSERT_TRUE(planResult.has_value()) << "Planner failed to find a plan.";

    
    const auto& plan = *planResult;
    
    
    
    
    // The plan with a smart GetItem generator:
    // 1. Move(Mines)
    // 2. Get(Ore, 1)
    // 3. Get(Ore, 1)
    // 4. Move(Forge)
    // 5. Craft(Ingot) -> requires 1 ore, 1 wood
    // 6. Craft(Ingot) -> requires 1 ore, 1 wood
    ASSERT_EQ(plan.size(), 6);

    EXPECT_EQ(plan[0].name, "MoveTo");
    EXPECT_EQ(plan[0].params.at("targetLocation"), loc_mines);

    EXPECT_EQ(plan[1].name, "GetItem");
    EXPECT_EQ(plan[1].params.at("itemToGet"), item_ore);
    EXPECT_EQ(plan[1].params.at("quantityToGet"), 1);

    EXPECT_EQ(plan[2].name, "GetItem");
    EXPECT_EQ(plan[2].params.at("itemToGet"), item_ore);
    EXPECT_EQ(plan[2].params.at("quantityToGet"), 1);

    EXPECT_EQ(plan[3].name, "MoveTo");
    EXPECT_EQ(plan[3].params.at("targetLocation"), loc_forge);

    // The craft actions might be generic "Craft" schemas, so we check the recipeId param.
    EXPECT_EQ(plan[4].name, "Craft(IronIngot)");
    EXPECT_EQ(plan[4].params.at("recipeId"), item_ingot);

    EXPECT_EQ(plan[5].name, "Craft(IronIngot)");
    EXPECT_EQ(plan[5].params.at("recipeId"), item_ingot);
    
}