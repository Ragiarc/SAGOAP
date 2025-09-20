#include "gtest/gtest.h"
#include "SAHGOAP.h"
using namespace SAHGOAP;
using namespace SAHGOAP::Utils;

// STATE TYPES (LocationState, WorldResourceState, InventoryState, etc.)
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
    std::unique_ptr<BaseAction> Clone() const override { return std::make_unique<CraftItemAction>(recipe); }
};


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
    Goal initialGoal;
    initialGoal.push_back(std::make_unique<AchieveStateTask>(std::move(finalWorldStateGoal)));

    // --- Heuristic ---
    HierarchicalPlanner::HeuristicFunction heuristic = [](const AgentState& state, const Goal& goal) {
        return static_cast<float>(goal.size()); // Simple heuristic: number of tasks left
    };

    // --- Create and run the planner ---
    HierarchicalPlanner planner;
    auto plan = planner.Plan(initialState, std::move(initialGoal), registry, actionGenerator, heuristic);

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