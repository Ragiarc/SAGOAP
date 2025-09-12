#include "gtest/gtest.h"
#include "SAGOAP.h"
using namespace SAGOAP;
using namespace SAGOAP::Utils;
// =============================================================================
// 1. DEFINE USER-SPECIFIC STATE TYPES
// These are the concrete structs the user of the library would create.
// They must be copy-constructible and have the required methods.
// =============================================================================

struct LocationState
{
	std::string name;

	void AddValues(const LocationState& other) { name = other.name; /* Location is replaced */ }
	void SubtractValues(const LocationState& other) { if (name == other.name) name.clear(); }
	size_t GetHash() const { return std::hash<std::string>()(name); }
	bool IsEmpty() const { return name.empty(); }

	std::string ToString() const { return "Location(" + name + ")"; }
};

// Represents the agent's belief about a single resource in the world.
struct WorldItemInfo {
	std::string location;
	int quantity;
	std::string ToString() const { return "Location(" + location + ")" + " Quantity(" + std::to_string(quantity) + ")"; }
};

// Represents the agent's entire belief system about where world resources are.
struct WorldResourceState {
	std::map<std::string, WorldItemInfo> resources;

	// AddValues handles deltas. A delta can contain negative quantities.
	void AddValues(const WorldResourceState& other) {
		for (const auto& [item_name, item_info_delta] : other.resources) {
			auto it = resources.find(item_name);
			if (it != resources.end()) {
				it->second.quantity += item_info_delta.quantity;
				if (it->second.quantity <= 0) {
					resources.erase(it);
				}
			} else {
				// If the base state doesn't know about this item, just add the delta info
				// This would happen if an action reveals a new resource location.
				resources[item_name] = item_info_delta;
			}
		}
	}
    
	// For this model, subtracting doesn't make logical sense, so it's a no-op.
	void SubtractValues(const WorldResourceState&) {}
    
	size_t GetHash() const { return resources.size(); }
	bool IsEmpty() const { return resources.empty(); }
	std::string ToString() const
	{
		std::string result = "{";
		for (const auto& [key, value] : resources)
		{
			result += key + ":" + value.ToString() + ";\n";
		}
		return result + "}";
	}
};

struct InventoryState
{
	std::map<std::string, int> items;

	void AddValues(const InventoryState& other) {
		for (const auto& [item, quant] : other.items) {
			items[item] += quant;
			if (items[item] <= 0) {
				items.erase(item);
			}
		}
	}
	void SubtractValues(const InventoryState& other_delta) {
		// "other_delta" is the action's results.
		// We only want to subtract from items that are already part of our goal state.
		for (const auto& [item_name, delta_quant] : other_delta.items) {
        
			auto goal_item_it = this->items.find(item_name);

			// If the action's result affects an item that is in our goal...
			if (goal_item_it != this->items.end()) {
				// ...subtract the result's contribution from our goal.
				goal_item_it->second -= delta_quant;

				// If the goal for this specific item is now met (or exceeded), remove it.
				if (goal_item_it->second <= 0) {
					this->items.erase(goal_item_it);
				}
			}
		}
	}
	size_t GetHash() const {
		size_t seed = 0;
		for (const auto& [name, quant] : items) {
			seed ^= std::hash<std::string>()(name) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
			seed ^= std::hash<int>()(quant) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
		}
		return seed;
	}
	bool IsEmpty() const { return items.empty(); }

	std::string ToString() const {
		if (items.empty()) return "Inventory{}";
		std::string s = "Inventory{ ";
		for (const auto& [item, quant] : items) {
			s += item + ":" + std::to_string(quant) + " ";
		}
		s += "}";
		return s;
	}
};

struct Recipe {
	// Things that are consumed
	InventoryState ingredients;
	// Things that are required but not consumed
	InventoryState tools;
	// The output of the craft
	InventoryState products;
	// The location required for the craft (e.g., "Forge")
	std::string location;
	std::string ToString() const {return "";};
};

struct KnownRecipesState {
	std::vector<Recipe> recipes;

	// This state is just a list, so these methods are simple.
	void AddValues(const KnownRecipesState& other) { recipes.insert(recipes.end(), other.recipes.begin(), other.recipes.end()); }
	void SubtractValues(const KnownRecipesState&) { /* Can't subtract recipes in this model */ }
	size_t GetHash() const { return recipes.size(); /* Simple hash for testing */ }
	bool IsEmpty() const { return recipes.empty(); }
	std::string ToString() const {return "";}
};

// =============================================================================
// 2. DEFINE USER-SPECIFIC ACTIONS
// =============================================================================

class AddItemToInventoryAction : public BaseAction {
private:
    // These are configured at runtime based on the goal and agent's knowledge
    std::string itemToGet;
    std::string itemLocation;
    int quantityToGet = 1;

public:
    AddItemToInventoryAction() = default;

	std::string GetName() const override
	{
		return "Add " + std::to_string(quantityToGet) + " " + itemToGet + " to inventory";
	}
	
    float GetCost() const override { return 1.0f; }

    std::unique_ptr<BaseAction> Clone() const override {
        auto clone = std::make_unique<AddItemToInventoryAction>();
        clone->itemToGet = this->itemToGet;
        clone->itemLocation = this->itemLocation;
        clone->quantityToGet = this->quantityToGet;
        clone->requirements = this->requirements;
        clone->results = this->results;
        return clone;
    }

	static std::vector<std::unique_ptr<BaseAction>> GenerateInstances(const AgentState& currentState, const Goal& goal) {
		std::vector<std::unique_ptr<BaseAction>> instances;
		const auto* invGoal = Get<InventoryState>(goal);
		const auto* worldResources = Get<WorldResourceState>(currentState);

		if (!invGoal || !worldResources) return instances;

		for (const auto& [goal_item, goal_quant] : invGoal->items) {
			const auto* currentInv = Get<InventoryState>(currentState);
			int current_quant = (currentInv && currentInv->items.count(goal_item)) ? currentInv->items.at(goal_item) : 0;

			if (current_quant < goal_quant && worldResources->resources.count(goal_item)) {
				const auto& item_info = worldResources->resources.at(goal_item);
				if (item_info.quantity > 0) {
					auto action = std::make_unique<AddItemToInventoryAction>();
					// Configure this action instance for this specific item.
					action->itemToGet = goal_item;
					action->itemLocation = item_info.location;
					action->quantityToGet = std::min(goal_quant - current_quant, item_info.quantity);

					// Set requirements and results
					action->requirements = action->GenerateRequirements(currentState, goal);
					action->results = action->GenerateResults(currentState, goal);

					instances.push_back(std::move(action));
					// We only generate one "get item" action at a time to keep the search space smaller.
					// The planner can chain them if it needs more than one type of item.
					return instances;
				}
			}
		}
		return instances;
	}

    AgentState GenerateRequirements(const AgentState& currentState, const Goal& goal) override {
        const auto* invGoal = Get<InventoryState>(goal);
        const auto* worldResources = Get<WorldResourceState>(currentState);
        
        if (!invGoal || !worldResources) return {};

        for (const auto& [goal_item, goal_quant] : invGoal->items) {
            const auto* currentInv = Get<InventoryState>(currentState);
            int current_quant = (currentInv && currentInv->items.count(goal_item)) ? currentInv->items.at(goal_item) : 0;
            
            if (current_quant < goal_quant && worldResources->resources.count(goal_item)) {
                const auto& item_info = worldResources->resources.at(goal_item);
                if (item_info.quantity > 0) {
                    // Configure this action instance for this specific item.
                    this->itemToGet = goal_item;
                    this->itemLocation = item_info.location;
                    this->quantityToGet = goal_quant - current_quant;
                    
                    AgentState reqs;
                    Set(reqs, LocationState{this->itemLocation});
                    return reqs;
                }
            }
        }
        return {};
    }

	AgentState GenerateResults(const AgentState&, const Goal&) override {
		if (itemToGet.empty()) return {};
		AgentState results_delta;
		Set(results_delta, InventoryState{{{this->itemToGet, this->quantityToGet}}});
		WorldResourceState belief_delta;
		belief_delta.resources[this->itemToGet] = WorldItemInfo{this->itemLocation, -this->quantityToGet};
		Set(results_delta, belief_delta);
		return results_delta;
	}
};


class CraftAction : public BaseAction {
private:
    // This action instance, once configured, will represent one specific recipe.
    std::optional<Recipe> chosenRecipe;

public:
    // Now default-constructible!
    CraftAction() = default;

	std::string GetName() const override
	{
		return "CraftAction";
	}
	
    float GetCost() const override { return 1.0f; }

    std::unique_ptr<BaseAction> Clone() const override {
        auto clone = std::make_unique<CraftAction>();
        // CRITICAL: The clone must copy the configured state!
        clone->chosenRecipe = this->chosenRecipe;
        clone->requirements = this->requirements;
        clone->results = this->results;
        return clone;
    }

    static std::vector<std::unique_ptr<BaseAction>> GenerateInstances(const AgentState& currentState, const Goal& goal) {
        std::vector<std::unique_ptr<BaseAction>> instances;
        const auto* knownRecipes = Get<KnownRecipesState>(currentState);
        const auto* invGoal = Get<InventoryState>(goal);

        if (!knownRecipes || !invGoal) {
            return instances;
        }

        // For EACH known recipe...
        for (const auto& recipe : knownRecipes->recipes) {
            // ...check if it produces an item that is currently in our goal.
            for (const auto& [productName, quant] : recipe.products.items) {
                if (invGoal->items.count(productName)) {
                    // This recipe is relevant! Create a configured instance for it.
                    auto action = std::make_unique<CraftAction>();
                    action->chosenRecipe = recipe; // Pre-configure with this specific recipe

                    // Now populate the requirements and results based on this recipe
                    action->requirements = action->GenerateRequirements(currentState, goal);
                    action->results = action->GenerateResults(currentState, goal);
                    
                    instances.push_back(std::move(action));
                    
                    // Use a goto to break out of the inner loop once we've created an action
                    // for this recipe, to avoid creating duplicates if a recipe has multiple
                    // products that are in the goal.
                    goto next_recipe;
                }
            }
        next_recipe:;
        }
        return instances;
    }
	
    AgentState GenerateRequirements(const AgentState&, const Goal&) override {
        if (!chosenRecipe) return {}; // Should not happen with the new design

        AgentState reqs;
        Set(reqs, LocationState{this->chosenRecipe->location});
        InventoryState allRequiredItems;
        allRequiredItems.AddValues(this->chosenRecipe->ingredients);
        allRequiredItems.AddValues(this->chosenRecipe->tools);
        Set(reqs, allRequiredItems);
        return reqs;
    }
	
    AgentState GenerateResults(const AgentState&, const Goal&) override {
        if (!chosenRecipe) return {};
        InventoryState results_delta;
        results_delta.AddValues(chosenRecipe->products);
        InventoryState consumed_ingredients;
        for (const auto& [item, quant] : chosenRecipe->ingredients.items) {
            consumed_ingredients.items[item] = -quant;
        }
        results_delta.AddValues(consumed_ingredients);
        AgentState results;
        Set(results, results_delta);
        return results;
    }
};

class MoveToAction : public BaseAction {
public:
	std::string targetLocationName; // Configured at runtime
	float cost = 1.0f;

	std::string GetName() const override { return "MoveToAction"; }
	
	static std::vector<std::unique_ptr<BaseAction>> GenerateInstances(const AgentState& currentState, const Goal& goal) {
		std::vector<std::unique_ptr<BaseAction>> instances;
		const auto* currentLoc = Get<LocationState>(currentState);
		const auto* goalLoc = Get<LocationState>(goal);

		// Is this action relevant?
		if (goalLoc && (!currentLoc || currentLoc->name != goalLoc->name)) {
			auto action = std::make_unique<MoveToAction>();
			// Configure it
			action->targetLocationName = goalLoc->name;
			// Set requirements and results
			action->requirements = action->GenerateRequirements(currentState, goal);
			action->results = action->GenerateResults(currentState, goal);
			instances.push_back(std::move(action));
		}
		return instances;
	}

	AgentState GenerateRequirements(const AgentState& currentState, const Goal& goal) override {
		// Set this action's specific parameters from the goal
		if (const auto* goalLoc = Get<LocationState>(goal)) {
			this->targetLocationName = goalLoc->name;
		}
		return {}; // No preconditions for a simple move
	}

	AgentState GenerateResults(const AgentState& currentState, const Goal& /*goal*/) override {
		AgentState res;
		Set(res, LocationState{this->targetLocationName});
		return res;
	}

	std::unique_ptr<BaseAction> Clone() const override {
		auto clone = std::make_unique<MoveToAction>();
		clone->targetLocationName = this->targetLocationName;
		clone->requirements = this->requirements; // BaseAction state
		clone->results = this->results;         // BaseAction state
		return clone;
	}

	float GetCost() const override { return cost; }
};


class GetKeyInKeyRoomAction : public BaseAction {
public:
	std::string GetName() const override
	{
		return "GetKeyInRoom";
	}
	float GetCost() const override { return 1.0f; }

	static std::vector<std::unique_ptr<BaseAction>> GenerateInstances(const AgentState& currentState, const Goal& goal) {
		std::vector<std::unique_ptr<BaseAction>> instances;

		auto action = std::make_unique<GetKeyInKeyRoomAction>();
		// Set requirements and results
		action->requirements = action->GenerateRequirements(currentState, goal);
		action->results = action->GenerateResults(currentState, goal);
		instances.push_back(std::move(action));
		
		return instances;
	}

	AgentState GenerateRequirements(const AgentState& currentState, const Goal&) override {
		AgentState reqs;
		Set(reqs, LocationState{"KeyRoom"});
		return reqs;
	}

	AgentState GenerateResults(const AgentState& currentState, const Goal&) override {
		AgentState res;
		Set(res, InventoryState{{{"Key", 1}}});
		return res;
	}
    
	std::unique_ptr<BaseAction> Clone() const override {
		auto clone = std::make_unique<GetKeyInKeyRoomAction>();
		clone->requirements = this->requirements;
		clone->results = this->results;
		return clone;
	}
};


// =============================================================================
// TEST FIXTURE
// This fixture creates and configures the registry for all tests.
// =============================================================================
class SagoapTest : public ::testing::Test {
protected:
	StateTypeRegistry registry;

	// This is called before each test
	void SetUp() override {
		registry.RegisterType<LocationState>();
		registry.RegisterType<InventoryState>();
		registry.RegisterType<KnownRecipesState>();

		SAGOAP::Debug::SetRegistryForVisualization(&registry);
	}
};

// =============================================================================
// TESTS
// =============================================================================

TEST_F(SagoapTest, CombineStates_ReplacesLocation) {
    AgentState base;
    Set(base, LocationState{"Start"});

    AgentState delta;
    Set(delta, LocationState{"End"});

    AgentState result = CombineStates(base, delta, registry);

    const auto* resultLoc = Get<LocationState>(result);
    ASSERT_NE(resultLoc, nullptr);
    EXPECT_EQ(resultLoc->name, "End");
}

TEST_F(SagoapTest, CombineStates_AddsToInventory) {
    AgentState base;
    Set(base, InventoryState{{{"Wood", 5}}});

    AgentState delta;
    Set(delta, InventoryState{{{"Iron", 2}, {"Wood", 3}}});

    AgentState result = CombineStates(base, delta, registry);

    const auto* resultInv = Get<InventoryState>(result);
    ASSERT_NE(resultInv, nullptr);
    EXPECT_EQ(resultInv->items.size(), 2);
    EXPECT_EQ(resultInv->items.at("Wood"), 8);
    EXPECT_EQ(resultInv->items.at("Iron"), 2);
}

TEST_F(SagoapTest, CombineStates_RemovesFromInventoryWithNegativeDelta) {
    AgentState base;
    Set(base, InventoryState{{{"Arrows", 20}}});

    AgentState delta;
    Set(delta, InventoryState{{{"Arrows", -5}}}); // Fired 5 arrows

    AgentState result = CombineStates(base, delta, registry);
    const auto* resultInv = Get<InventoryState>(result);
    ASSERT_NE(resultInv, nullptr);
    EXPECT_EQ(resultInv->items.at("Arrows"), 15);

    // Test removing all arrows
    Set(delta, InventoryState{{{"Arrows", -20}}});
    result = CombineStates(base, delta, registry);
    resultInv = Get<InventoryState>(result);
    ASSERT_NE(resultInv, nullptr);
    EXPECT_TRUE(resultInv->items.empty());
}

TEST_F(SagoapTest, SubtractStates_RemovesMetGoal) {
    Goal goal;
    Set(goal, LocationState{"End"});
    Set(goal, InventoryState{{{"Key", 1}}});

    AgentState results_from_action;
    Set(results_from_action, LocationState{"End"}); // This action achieved the location goal

    Goal nextGoal = SubtractStates(goal, results_from_action, registry);

    EXPECT_EQ(nextGoal.properties.size(), 1);
    EXPECT_EQ(Get<LocationState>(nextGoal), nullptr); // Location goal should be gone
    ASSERT_NE(Get<InventoryState>(nextGoal), nullptr); // Inventory goal should remain
}

TEST_F(SagoapTest, ActionGenerator_GeneratesAndConfiguresAction) {
    AgentState currentState;
    Set(currentState, LocationState{"Start"});

    Goal goal;
    Set(goal, LocationState{"End"});

    ActionGenerator<MoveToAction> generator;
    auto actions = generator.GenerateActions(currentState, goal);

    ASSERT_EQ(actions.size(), 1);
    MoveToAction* moveAction = dynamic_cast<MoveToAction*>(actions[0].get());
    ASSERT_NE(moveAction, nullptr);

    // Check if Configure was called and set the internal state correctly
    EXPECT_EQ(moveAction->targetLocationName, "End");
    
    // Check if results were generated correctly
    const auto* resultLoc = Get<LocationState>(moveAction->results);
    ASSERT_NE(resultLoc, nullptr);
    EXPECT_EQ(resultLoc->name, "End");
}

TEST_F(SagoapTest, GoapPlanner_FindsSimplePlan) {
    // --- Define a simple heuristic ---
    HeuristicFunction heuristic = [](const AgentState& current, const Goal& goal) -> float {
        // A simple heuristic just counts the number of unsatisfied goal properties.
        float cost = 0.0f;
        for (const auto& [type, goal_prop] : goal.properties) {
            auto it = current.properties.find(type);
            if (it == current.properties.end()) {
                cost += 1.0; // Property doesn't exist in current state
            } else {
                // A more complex heuristic would compare the values. For now, we assume if
                // the type exists, it might be satisfied. This is a weak but functional heuristic.
            }
        }
        return cost;
    };

    // --- Set up the scenario ---
    AgentState initialState;
    Set(initialState, LocationState{"StartRoom"});
    Set(initialState, InventoryState{});

    Goal finalGoal;
    Set(finalGoal, InventoryState{{{"Key", 1}}});

    ActionGenerator<MoveToAction, GetKeyInKeyRoomAction> actionGenerator;

    // --- RUN THE PLANNER! ---
    auto plan = GoapPlanner::Plan(initialState, finalGoal, heuristic, actionGenerator, registry);

    // --- VERIFY THE PLAN ---
    ASSERT_FALSE(plan.empty());
    ASSERT_EQ(plan.size(), 2);

    // Step 1: MoveToAction
    MoveToAction* firstAction = dynamic_cast<MoveToAction*>(plan[0].get());
    ASSERT_NE(firstAction, nullptr);
    // Planner should have identified "KeyRoom" as a subgoal from GetKeyAction's requirements
    // and configured the generic MoveToAction to go there.
    EXPECT_EQ(firstAction->targetLocationName, "KeyRoom");

    // Step 2: GetKeyInKeyRoomAction
    GetKeyInKeyRoomAction* secondAction = dynamic_cast<GetKeyInKeyRoomAction*>(plan[1].get());
    ASSERT_NE(secondAction, nullptr);
}

TEST_F(SagoapTest, GoapPlanner_CraftsSword_WithFullyGenericActions) {
    // --- Register all our custom state types ---
    registry.RegisterType<KnownRecipesState>();
    registry.RegisterType<WorldResourceState>();

    // --- Define Recipes (data only) ---
    Recipe ironIngotRecipe{
        /* ingredients */ InventoryState{{{"IronOre", 1}, {"Wood", 1}}},
        /* tools */       InventoryState{},
        /* products */    InventoryState{{{"IronIngot", 1}}},
        /* location */    "Forge"
    };
    Recipe swordRecipe{
        /* ingredients */ InventoryState{{{"IronIngot", 2}}},
        /* tools */       InventoryState{{{"Hammer", 1}}},
        /* products */    InventoryState{{{"Sword", 1}}},
        /* location */    "Forge"
    };

    // --- Set up Initial State (agent's full knowledge) ---
    AgentState initialState;
    Set(initialState, LocationState{"Village"});
    Set(initialState, InventoryState{{{"Hammer", 1}, {"Wood", 2}}});
    Set(initialState, KnownRecipesState{{ironIngotRecipe, swordRecipe}});
    // The agent believes 2 Iron Ore are available at the Mines.
    Set(initialState, WorldResourceState{{
        {"IronOre", WorldItemInfo{"Mines", 2}}
    }});

    // --- Goal 
    Goal finalGoal;
    Set(finalGoal, InventoryState{{{"Sword", 1}}});

    // --- ActionGenerator 
    ActionGenerator<MoveToAction, CraftAction, AddItemToInventoryAction> actionGenerator;

    HeuristicFunction heuristic = [](const AgentState&, const Goal& goal) -> float {
        return static_cast<float>(goal.properties.size());
    };

    // --- Run Planner ---
    auto plan = GoapPlanner::Plan(initialState, finalGoal, heuristic, actionGenerator, registry);

    // --- Verify Plan ---
    ASSERT_FALSE(plan.empty());
    ASSERT_EQ(plan.size(), 5);

    // Step 1: MoveToAction to "Mines"
    MoveToAction* step1 = dynamic_cast<MoveToAction*>(plan[0].get());
    ASSERT_NE(step1, nullptr);
    EXPECT_EQ(step1->targetLocationName, "Mines");

    // Step 2: AddItemToInventoryAction (configured for IronOre)
    AddItemToInventoryAction* step2 = dynamic_cast<AddItemToInventoryAction*>(plan[1].get());
    ASSERT_NE(step2, nullptr);
    const auto* oreInvResult = Get<InventoryState>(step2->results);
    const auto* oreWorldResult = Get<WorldResourceState>(step2->results);
    ASSERT_NE(oreInvResult, nullptr);
    ASSERT_NE(oreWorldResult, nullptr);
    EXPECT_EQ(oreInvResult->items.at("IronOre"), 2); // Check inventory gain
    EXPECT_EQ(oreWorldResult->resources.at("IronOre").quantity, -2); // Check world depletion belief

    // Step 3: MoveToAction to "Forge"
    MoveToAction* step3 = dynamic_cast<MoveToAction*>(plan[2].get());
    ASSERT_NE(step3, nullptr);
    EXPECT_EQ(step3->targetLocationName, "Forge");

    // Step 4: CraftAction (configured for Ingots)
    CraftAction* step4 = dynamic_cast<CraftAction*>(plan[3].get());
    ASSERT_NE(step4, nullptr);
    const auto* ingotResult = Get<InventoryState>(step4->results);
    ASSERT_NE(ingotResult, nullptr);
    EXPECT_TRUE(ingotResult->items.count("IronIngot"));

	// Step 5: CraftAction (configured for ingot)
	CraftAction* step5 = dynamic_cast<CraftAction*>(plan[4].get());
	const auto* ingotResult2 = Get<InventoryState>(step5->results);
	EXPECT_TRUE(ingotResult2->items.count("Sword"));
	
    // Step 6: CraftAction (configured for Sword)
    CraftAction* step6 = dynamic_cast<CraftAction*>(plan[5].get());
    ASSERT_NE(step6, nullptr);
    const auto* swordResult = Get<InventoryState>(step6->results);
    ASSERT_NE(swordResult, nullptr);
    EXPECT_TRUE(swordResult->items.count("Sword"));
}