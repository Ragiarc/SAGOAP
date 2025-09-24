#include "gtest/gtest.h"
#include <type_traits> 
#include "SAHGOAP.h"
using namespace SAHGOAP;
using namespace SAHGOAP::Utils;

// STATE TYPES (LocationState, WorldResourceState, InventoryState, etc.)
namespace SAHGOAP
{
	struct LocationState
	{
		std::string name;

		void AddValues(const LocationState& other) { name = other.name; /* Location is replaced */ }
		void SubtractValues(const LocationState& other)
		{
			if (name == other.name) name.clear();
		}
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
		void SubtractValues(const InventoryState& state_to_subtract) {
			// This logic now correctly handles both subtracting a world state and an action's results.
			// We iterate through OUR items (the goal) and see if the other state can satisfy them.
			for (auto goal_it = this->items.begin(); goal_it != this->items.end(); /* no increment */) {
				const std::string& goal_item_name = goal_it->first;
				int& goal_quantity = goal_it->second;

				// Check if the state we're subtracting has this item
				auto sub_it = state_to_subtract.items.find(goal_item_name);
				if (sub_it != state_to_subtract.items.end()) {
					// It does, so subtract its quantity from our goal quantity.
					goal_quantity -= sub_it->second;
				}

				// If the goal for this item is now met (or exceeded), remove it from the goal.
				if (goal_quantity <= 0) {
					goal_it = this->items.erase(goal_it);
				} else {
					++goal_it;
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
	// 2. DEFINE USER-SPECIFIC ACTIONS (Now Primitive)
	// ===+==========================================================================

	class MoveToAction : public BaseAction {
		std::string target;
	public:
		MoveToAction(std::string t) : target(std::move(t)) {}

		static std::vector<std::unique_ptr<BaseAction>> GenerateInstances(const AgentState& currentState, const AgentState& goalState) {
			std::vector<std::unique_ptr<BaseAction>> instances;
			if (const auto* goalLoc = Get<LocationState>(goalState)) {
				const auto* currentLoc = Get<LocationState>(currentState);
				if (!currentLoc || currentLoc->name != goalLoc->name) {
					instances.push_back(std::make_unique<MoveToAction>(goalLoc->name));
				}
			}
			return instances;
		}
    
		AgentState GetRequirements(const AgentState&) const override
		{
			AgentState reqs;
			return reqs;
		}
		AgentState GetEffects(const AgentState&) const override {
			AgentState effects;
			Set(effects, LocationState{target});
			return effects;
		}
		float GetCost(const AgentState&) const override { return 1.0f; }
		std::string GetName() const override { return "MoveTo(" + target + ")"; }
		std::unique_ptr<BaseAction> Clone() const override { return std::make_unique<MoveToAction>(target); }
	};

	class GetItemAction : public BaseAction {
		std::string itemName;
		std::string itemLocation;
		int quantity;
	public:
		GetItemAction(std::string name, std::string loc, int quant) 
			: itemName(name), itemLocation(loc), quantity(quant) {}

		static std::vector<std::unique_ptr<BaseAction>> GenerateInstances(const AgentState& currentState, const AgentState& goalState) {
			std::vector<std::unique_ptr<BaseAction>> instances;
			const auto* invGoal = Get<InventoryState>(goalState);
			const auto* worldState = Get<WorldResourceState>(currentState);
			if (!invGoal || !worldState) return instances;
        
			for (const auto& [item_name, goal_quant] : invGoal->items) {
				if (goal_quant <= 0) continue;
				auto it = worldState->resources.find(item_name);
				if (it != worldState->resources.end() && it->second.quantity > 0) {
					instances.push_back(std::make_unique<GetItemAction>(item_name, it->second.location, goal_quant));
				}
			}
			return instances;
		}

		AgentState GetRequirements(const AgentState&) const override {
			AgentState reqs;
			Set(reqs, LocationState{itemLocation});
			return reqs;
		}
		AgentState GetEffects(const AgentState&) const override {
			AgentState effects;
			Set(effects, InventoryState{{{itemName, quantity}}});
			Set(effects, WorldResourceState{{{itemName, {itemLocation, -quantity}}}});
			return effects;
		}
		float GetCost(const AgentState&) const override { return 1.0f; }
		std::string GetName() const override { return "Get(" + itemName + ")"; }
		std::unique_ptr<BaseAction> Clone() const override { return std::make_unique<GetItemAction>(itemName, itemLocation, quantity); }
	};

	class CraftItemAction : public BaseAction {
		Recipe recipe;
	public:
		CraftItemAction(Recipe r) : recipe(std::move(r)) {}

		CraftItemAction(const CraftItemAction& other) : recipe(other.recipe) {}
	
		static std::vector<std::unique_ptr<BaseAction>> GenerateInstances(const AgentState& currentState, const AgentState& goalState) {
			std::vector<std::unique_ptr<BaseAction>> instances;
			const auto* invGoal = Get<InventoryState>(goalState);
			const auto* knownRecipes = Get<KnownRecipesState>(currentState);
			if (!invGoal || !knownRecipes) return instances;

			for (const auto& r : knownRecipes->recipes) {
				for (const auto& [product_name, quant] : r.products.items) {
					if (invGoal->items.count(product_name)) {
						instances.push_back(std::make_unique<CraftItemAction>(r));
						goto next_recipe;
					}
				}
				next_recipe:;
			}
			return instances;
		}

		AgentState GetRequirements(const AgentState&) const override {
			AgentState reqs;
			Set(reqs, LocationState{recipe.location});
			InventoryState allRequiredItems;
			allRequiredItems.AddValues(recipe.ingredients);
			allRequiredItems.AddValues(recipe.tools);
			Set(reqs, allRequiredItems);
			return reqs;
		}
		AgentState GetEffects(const AgentState&) const override {
			AgentState effects;
			InventoryState delta;
			delta.AddValues(recipe.products);
			InventoryState consumed;
			for (const auto& [item, quant] : recipe.ingredients.items) {
				consumed.items[item] = -quant;
			}
			delta.AddValues(consumed);
			Set(effects, delta);
			return effects;
		}
		float GetCost(const AgentState&) const override { return 1.0f; }
		std::string GetName() const override { return "Craft(" + recipe.products.items.begin()->first + ")"; }
		std::unique_ptr<BaseAction> Clone() const override { 
			return std::make_unique<CraftItemAction>(*this); 
		}
	};
}

// =============================================================================
// TEST FIXTURE AND TESTS
// =============================================================================

class SagoapTest : public ::testing::Test {
protected:
	StateTypeRegistry registry;

	// This is called before each test
	void SetUp() override {
		registry.RegisterType<LocationState>();
		registry.RegisterType<InventoryState>();
		registry.RegisterType<KnownRecipesState>();

		SAHGOAP::Debug::SetRegistryForVisualization(&registry);
	}

	void TearDown() override {
		// Clear the global pointer so the debugger doesn't try to use it
		// after the 'registry' object has been destroyed.
		SAHGOAP::Debug::SetRegistryForVisualization(nullptr);
	}
};

TEST_F(SagoapTest, HierarchicalPlanner_CraftsSword) {
    // --- Register WorldResourceState ---
    registry.RegisterType<WorldResourceState>();
    registry.RegisterType<KnownRecipesState>();

	// --- Define Recipes (data only) ---
	Recipe ironIngotRecipe {
		/* ingredients */ InventoryState{{{"IronOre", 1}, {"Wood", 1}}},
		/* tools */       InventoryState{},
		/* products */    InventoryState{{{"IronIngot", 1}}},
		/* location */    "Forge"
	};
	Recipe swordRecipe {
		/* ingredients */ InventoryState{{{"IronIngot", 2}}},
		/* tools */       InventoryState{{{"Hammer", 1}}},
		/* products */    InventoryState{{{"Sword", 1}}},
		/* location */    "Forge"
	};

    // --- Set up Initial State ---
    AgentState initialState;
    Set(initialState, LocationState{"Village"});
    Set(initialState, InventoryState{{{"Hammer", 1}, {"Wood", 2}}});
    Set(initialState, KnownRecipesState{{ironIngotRecipe, swordRecipe}});
    Set(initialState, WorldResourceState{{{"IronOre", WorldItemInfo{"Mines", 2}}}});

    // --- ActionGenerator ---
    ActionGenerator<MoveToAction, GetItemAction, CraftItemAction> actionGenerator;

    // --- Define the ultimate goal using the library's generic task ---
    AgentState finalWorldStateGoal;
    Set(finalWorldStateGoal, InventoryState{{{"Sword", 1}}});
 

    // --- Heuristic ---
    HierarchicalPlanner::HeuristicFunction heuristic = [](const AgentState& state, const Goal& goal) {
        return static_cast<float>(goal.size()); // Simple heuristic: number of tasks left
    };

    // --- Create and run the planner ---
    HierarchicalPlanner planner;
    auto plan = planner.Plan(initialState, finalWorldStateGoal, registry, actionGenerator, heuristic);

    // --- Verify Plan ---
    ASSERT_FALSE(plan.empty());
    ASSERT_EQ(plan.size(), 6);
    
    EXPECT_EQ(plan[0]->GetName(), "MoveTo(Mines)");
    EXPECT_EQ(plan[1]->GetName(), "Get(IronOre)");
    EXPECT_EQ(plan[2]->GetName(), "MoveTo(Forge)");
    EXPECT_EQ(plan[3]->GetName(), "Craft(IronIngot)");
    EXPECT_EQ(plan[4]->GetName(), "Craft(IronIngot)");
    EXPECT_EQ(plan[5]->GetName(), "Craft(Sword)");
}

TEST(MoveToActionTest, GetRequirements_AlwaysReturnsEmptyState) {
	// Arrange: Create a MoveToAction instance. The target location is arbitrary
	// and should not affect the outcome.
	MoveToAction action("Forge");

	// We can use any AgentState as the input, as the function should ignore it.
	// Let's create one with some data just to be certain.
	AgentState currentState;
	Set(currentState, LocationState{"Village"});
	Set(currentState, InventoryState{{{"Wood", 5}}});

	// Act: Call the function under test and store the result.
	AgentState requirements = action.GetRequirements(currentState);

	// Assert: The returned AgentState's properties map MUST be empty.
	// This directly targets the bug you observed, where this size was a huge
	// garbage number. The ASSERT_EQ will fail the test immediately if the size
	// is anything other than 0.
	ASSERT_EQ(requirements.properties.size(), 0) 
		<< "The properties map should have a size of 0.";
    
	// As a secondary, more idiomatic check, we can also test the .empty() method.
	EXPECT_TRUE(requirements.properties.empty())
		<< "The properties map should be empty.";
}

TEST_F(SagoapTest, Planner_FindsSimpleMovePlan)
{
	// This is a minimal test to verify the planner's core loop.
	// SCENARIO: The agent is in Room A, and the goal is to be in Room B.
	// EXPECTED: A one-step plan containing a single MoveToAction("Room B").

	// --- 1. ARRANGE ---
	AgentState initialState;
	Set(initialState, LocationState{"Room A"});
	
	AgentState goalState;
	Set(goalState, LocationState{"Room B"});

	// The set of actions available to the planner
	ActionGenerator<MoveToAction> actionGenerator;
	
	HierarchicalPlanner::HeuristicFunction heuristic = [](const AgentState& state, const Goal& goal) {
		return static_cast<float>(goal.size());
	};

	// --- 2. ACT ---

	HierarchicalPlanner planner;
	auto plan = planner.Plan(initialState, goalState, registry, actionGenerator, heuristic);

	// --- 3. ASSERT ---
	
	ASSERT_FALSE(plan.empty()) << "Planner failed to find any plan.";
	
	ASSERT_EQ(plan.size(), 1);
	
	auto* moveAction = dynamic_cast<MoveToAction*>(plan[0].get());
	ASSERT_NE(moveAction, nullptr) << "The action in the plan was not a MoveToAction.";
    
	EXPECT_EQ(moveAction->GetName(), "MoveTo(Room B)");
}

TEST_F(SagoapTest, Planner_FindsPlanWithOneSubgoal)
{
	// This test verifies the planner's ability to create a sub-goal.
	// SCENARIO: The agent needs "Wood", which is at the "Forest". The agent starts at the "Village".
	// EXPECTED: The planner must first create a plan to move to the Forest (sub-goal)
	// and then execute the action to get the wood.
	// Final plan should be: [MoveTo("Forest"), Get("Wood")]

	// --- 1. ARRANGE ---

	// The fixture registers LocationState and InventoryState. We also need WorldResourceState.
	registry.RegisterType<WorldResourceState>();

	// Initial state: Agent is at the village, has no items, but knows where wood is.
	AgentState initialState;
	Set(initialState, LocationState{"Village"});
	Set(initialState, InventoryState{});
	Set(initialState, WorldResourceState{{{"Wood", WorldItemInfo{"Forest", 5}}}});

	// Goal state: Agent wants 1 wood.
	AgentState goalState;
	Set(goalState, InventoryState{{{"Wood", 1}}});

	// Actions available: Move and Get.
	ActionGenerator<MoveToAction, GetItemAction> actionGenerator;

	HierarchicalPlanner::HeuristicFunction heuristic = [](const AgentState& state, const Goal& goal)
	{
		return static_cast<float>(goal.size());
	};

	// --- 2. ACT ---

	HierarchicalPlanner planner;
	auto plan = planner.Plan(initialState, goalState, registry, actionGenerator, heuristic);

	// --- 3. ASSERT ---

	ASSERT_FALSE(plan.empty()) << "Planner failed to find a plan with a sub-goal.";
	ASSERT_EQ(plan.size(), 2);

	// Step 1: Verify the sub-goal was to move to the forest.
	auto* step1 = dynamic_cast<MoveToAction*>(plan[0].get());
	ASSERT_NE(step1, nullptr);
	EXPECT_EQ(step1->GetName(), "MoveTo(Forest)");

	// Step 2: Verify the original goal action was to get the wood.
	auto* step2 = dynamic_cast<GetItemAction*>(plan[1].get());
	ASSERT_NE(step2, nullptr);
	EXPECT_EQ(step2->GetName(), "Get(Wood)");
}

TEST_F(SagoapTest, Planner_FindsPlanWithNestedSubgoals)
{
    // This is the most critical test. It verifies the planner can "recurse"
    // by solving a sub-goal that itself generates another sub-goal.
    // SCENARIO: Goal is "Dagger". To make a Dagger, you need an Ingot. To make an Ingot, you need Ore.
    // EXPECTED: [Move(Mines), Get(Ore), Move(Forge), Craft(Ingot), Craft(Dagger)]

    // --- 1. ARRANGE ---

    registry.RegisterType<WorldResourceState>();
    registry.RegisterType<KnownRecipesState>();

    // Define two simple, chained recipes.
    Recipe ingotRecipe {
        /* ingredients */ InventoryState{{{"Ore", 1}}},
        /* tools */       InventoryState{},
        /* products */    InventoryState{{{"Ingot", 1}}},
        /* location */    "Forge"
    };
    Recipe daggerRecipe {
        /* ingredients */ InventoryState{{{"Ingot", 1}}},
        /* tools */       InventoryState{},
        /* products */    InventoryState{{{"Dagger", 1}}},
        /* location */    "Forge"
    };

    // Initial state: Agent is at the village, has nothing, but knows recipes and where ore is.
    AgentState initialState;
    Set(initialState, LocationState{"Village"});
    Set(initialState, InventoryState{});
    Set(initialState, WorldResourceState{{{"Ore", WorldItemInfo{"Mines", 1}}}});
    Set(initialState, KnownRecipesState{{ingotRecipe, daggerRecipe}});

    // Goal state: Agent wants 1 Dagger.
    AgentState goalState;
    Set(goalState, InventoryState{{{"Dagger", 1}}});

    // Actions available: Move, Get, and Craft.
    ActionGenerator<MoveToAction, GetItemAction, CraftItemAction> actionGenerator;

    HierarchicalPlanner::HeuristicFunction heuristic = [](const AgentState&, const Goal& goal) {
        return static_cast<float>(goal.size());
    };

    // --- 2. ACT ---

    HierarchicalPlanner planner;
    auto plan = planner.Plan(initialState, goalState, registry, actionGenerator, heuristic);

    // --- 3. ASSERT ---

    ASSERT_FALSE(plan.empty()) << "Planner failed to find a plan with nested sub-goals.";
    ASSERT_EQ(plan.size(), 5);

    EXPECT_EQ(plan[0]->GetName(), "MoveTo(Mines)");
    EXPECT_EQ(plan[1]->GetName(), "Get(Ore)");
    EXPECT_EQ(plan[2]->GetName(), "MoveTo(Forge)");
    EXPECT_EQ(plan[3]->GetName(), "Craft(Ingot)");
    EXPECT_EQ(plan[4]->GetName(), "Craft(Dagger)");
}