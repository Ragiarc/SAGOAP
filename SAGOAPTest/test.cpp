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
	void SubtractValues(const InventoryState& other) {
		// Subtracting is the same as adding a negative delta
		InventoryState negative_delta;
		for (const auto& [item, quant] : other.items) {
			negative_delta.items[item] = -quant;
		}
		AddValues(negative_delta);
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
};

struct KnownRecipesState {
	std::vector<Recipe> recipes;

	// This state is just a list, so these methods are simple.
	void AddValues(const KnownRecipesState& other) { recipes.insert(recipes.end(), other.recipes.begin(), other.recipes.end()); }
	void SubtractValues(const KnownRecipesState&) { /* Can't subtract recipes in this model */ }
	size_t GetHash() const { return recipes.size(); /* Simple hash for testing */ }
	bool IsEmpty() const { return recipes.empty(); }
};

// =============================================================================
// 2. DEFINE USER-SPECIFIC ACTIONS
// =============================================================================

class CraftAction : public BaseAction {
private:
    // This action instance, once configured, will represent one specific recipe.
    // std::optional is perfect for state that is set after construction.
    std::optional<Recipe> chosenRecipe;

public:
    // Now default-constructible!
    CraftAction() = default;

    float GetCost() const override { return 1.0f; }

    std::unique_ptr<BaseAction> Clone() const override {
        auto clone = std::make_unique<CraftAction>();
        // CRITICAL: The clone must copy the configured state!
        clone->chosenRecipe = this->chosenRecipe;
        clone->requirements = this->requirements;
        clone->results = this->results;
        return clone;
    }

    bool IsRelevant(const AgentState& currentState, const Goal& goal) const override {
        const auto* knownRecipes = Get<KnownRecipesState>(currentState);
        const auto* invGoal = Get<InventoryState>(goal);

        if (!knownRecipes || !invGoal) {
            return false; // Can't craft if we know no recipes or have no inventory goal.
        }

        // Is there at least one known recipe that can produce an item in the goal?
        for (const auto& recipe : knownRecipes->recipes) {
            for (const auto& [productName, quant] : recipe.products.items) {
                if (invGoal->items.count(productName)) {
                    return true; // Found a relevant recipe.
                }
            }
        }
        return false;
    }
    
    AgentState GenerateRequirements(const AgentState& currentState, const Goal& goal) override {
        const auto* knownRecipes = Get<KnownRecipesState>(currentState);
        const auto* invGoal = Get<InventoryState>(goal);
        
        // This should always be true if IsRelevant passed, but check for safety.
        if (!knownRecipes || !invGoal) return {};

        // Find the first recipe that satisfies the goal and configure this action instance.
        for (const auto& recipe : knownRecipes->recipes) {
            for (const auto& [productName, quant] : recipe.products.items) {
                if (invGoal->items.count(productName)) {
                    // This is the recipe we will use.
                    this->chosenRecipe = recipe;
                    
                    // Now generate requirements based on this chosen recipe.
                    AgentState reqs;
                    Set(reqs, LocationState{this->chosenRecipe->location});
                    InventoryState allRequiredItems;
                    allRequiredItems.AddValues(this->chosenRecipe->ingredients);
                    allRequiredItems.AddValues(this->chosenRecipe->tools);
                    Set(reqs, allRequiredItems);
                    return reqs;
                }
            }
        }
        return {}; // Should not be reached.
    }

    AgentState GenerateResults(const AgentState&, const Goal&) override {
        // If this action wasn't configured with a recipe, it has no results.
        if (!chosenRecipe) return {};

        InventoryState delta;
        delta.AddValues(chosenRecipe->products); 
        delta.SubtractValues(chosenRecipe->ingredients);
        
        AgentState results;
        Set(results, delta);
        return results;
    }
};

class MoveToAction : public BaseAction {
public:
	std::string targetLocationName; // Configured at runtime
	float cost = 1.0f;

	bool IsRelevant(const AgentState& currentState, const Goal& goal) const override {
		const auto* currentLoc = Get<LocationState>(currentState);
		const auto* goalLoc = Get<LocationState>(goal);
		if (goalLoc) {
			return !currentLoc || currentLoc->name != goalLoc->name;
		}
		return false;
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
	float GetCost() const override { return 1.0f; }

	bool IsRelevant(const AgentState& currentState, const Goal& goal) const override {
		const auto* goalInv = Get<InventoryState>(goal);
		if (goalInv && goalInv->items.count("Key")) {
			const auto* currentInv = Get<InventoryState>(currentState);
			return !currentInv || !currentInv->items.count("Key");
		}
		return false;
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

class PickupIronOreAction : public BaseAction { 
public:
	float GetCost() const override { return 1.0f; }
	bool IsRelevant(const AgentState&, const Goal& goal) const override {
		const auto* invGoal = Get<InventoryState>(goal); return invGoal && invGoal->items.count("IronOre");
	}
	AgentState GenerateRequirements(const AgentState&, const Goal&) override {
		AgentState reqs; Set(reqs, LocationState{"Mines"}); return reqs;
	}
	AgentState GenerateResults(const AgentState&, const Goal&) override {
		AgentState res; Set(res, InventoryState{{{"IronOre", 2}}}); return res;
	}
	std::unique_ptr<BaseAction> Clone() const override {
		auto c = std::make_unique<PickupIronOreAction>();
		c->requirements=this->requirements; c->results=this->results; return c;
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

    EXPECT_EQ(nextGoal.size(), 1);
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
        for (const auto& [type, goal_prop] : goal) {
            auto it = current.find(type);
            if (it == current.end()) {
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

TEST_F(SagoapTest, GoapPlanner_CraftsSword_WithLearnedRecipes) {
    // --- Define Recipes ---
    Recipe ironIngotRecipe{/* ... */}; // Same as before
    Recipe swordRecipe{/* ... */};     // Same as before
    ironIngotRecipe.ingredients = InventoryState{{{"IronOre", 1}, {"Wood", 1}}};
    ironIngotRecipe.products = InventoryState{{{"IronIngot", 1}}};
    ironIngotRecipe.location = "Forge";
    swordRecipe.ingredients = InventoryState{{{"IronIngot", 2}}};
    swordRecipe.tools = InventoryState{{{"Hammer", 1}}};
    swordRecipe.products = InventoryState{{{"Sword", 1}}};
    swordRecipe.location = "Forge";


    // --- Set up Initial State (with learned recipes) ---
    AgentState initialState;
    Set(initialState, LocationState{"Village"});
    Set(initialState, InventoryState{{{"Hammer", 1}, {"Wood", 1}}});
    // The agent "knows" these recipes. This could be updated dynamically.
    Set(initialState, KnownRecipesState{{ironIngotRecipe, swordRecipe}});

    // --- Goal is unchanged ---
    Goal finalGoal;
    Set(finalGoal, InventoryState{{{"Sword", 1}}});

    // --- ActionGenerator is simple again ---
    // CraftAction is now default-constructible.
    ActionGenerator<MoveToAction, PickupIronOreAction, CraftAction> actionGenerator;

    HeuristicFunction heuristic = [](const AgentState&, const Goal& goal) -> float {
        return static_cast<float>(goal.size());
    };

    // --- Run Planner ---
    auto plan = GoapPlanner::Plan(initialState, finalGoal, heuristic, actionGenerator, registry);

    // --- Verify Plan ---
    ASSERT_FALSE(plan.empty());
    ASSERT_EQ(plan.size(), 5);

    // Step 1 & 2 are unchanged
    MoveToAction* step1 = dynamic_cast<MoveToAction*>(plan[0].get());
    ASSERT_NE(step1, nullptr);
    EXPECT_EQ(step1->targetLocationName, "Mines");

    PickupIronOreAction* step2 = dynamic_cast<PickupIronOreAction*>(plan[1].get());
    ASSERT_NE(step2, nullptr);

    // Step 3 is unchanged
    MoveToAction* step3 = dynamic_cast<MoveToAction*>(plan[2].get());
    ASSERT_NE(step3, nullptr);
    EXPECT_EQ(step3->targetLocationName, "Forge");

    // Step 4 & 5 are now generic CraftActions, we verify them by their results
    CraftAction* step4 = dynamic_cast<CraftAction*>(plan[3].get());
    ASSERT_NE(step4, nullptr);
    const auto* ingotResult = Get<InventoryState>(step4->results);
    ASSERT_NE(ingotResult, nullptr);
    EXPECT_TRUE(ingotResult->items.count("IronIngot")); // It's the ingot-crafting action

    CraftAction* step5 = dynamic_cast<CraftAction*>(plan[4].get());
    ASSERT_NE(step5, nullptr);
    const auto* swordResult = Get<InventoryState>(step5->results);
    ASSERT_NE(swordResult, nullptr);
    EXPECT_TRUE(swordResult->items.count("Sword")); // It's the sword-crafting action
}