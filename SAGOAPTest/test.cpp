#include "gtest/gtest.h"
#include "SAGOAP.h"

// test components
class HealthStateComponent : public StateComponent {
public:
  int value;

  HealthStateComponent(int v) : value(v) {}

  std::unique_ptr<StateComponent> Clone() const override {
    return std::make_unique<HealthStateComponent>(value);
  }

  void AddValues(const StateComponent& otherComponent) override {
    const auto* other = dynamic_cast<const HealthStateComponent*>(&otherComponent);
    if (other) value += other->value;
  }

  void SubtractValues(const StateComponent& otherComponent) override {
    const auto* other = dynamic_cast<const HealthStateComponent*>(&otherComponent);
    if (other) value -= other->value;
  }

  bool IsEmpty() const override {
    return value <= 0;
  }

  bool operator==(const HealthStateComponent& other) const {
    return value == other.value;
  }

  size_t GetHash() const override
  {
    return std::hash<int>()(value) ^ std::hash<std::string>()("Health");
  }
};

class ManaStateComponent : public StateComponent {
public:
  int value;

  ManaStateComponent(int v) : value(v) {}

  std::unique_ptr<StateComponent> Clone() const override {
    return std::make_unique<ManaStateComponent>(value);
  }

  void AddValues(const StateComponent& otherComponent) override {
    const auto* other = dynamic_cast<const ManaStateComponent*>(&otherComponent);
    if (other) value += other->value;
  }

  void SubtractValues(const StateComponent& otherComponent) override {
    const auto* other = dynamic_cast<const ManaStateComponent*>(&otherComponent);
    if (other) value -= other->value;
  }

  bool IsEmpty() const override {
    return value <= 0;
  }

  bool operator==(const ManaStateComponent& other) const {
    return value == other.value;
  }

  size_t GetHash() const override {
    return std::hash<int>()(value) ^ std::hash<std::string>()("Mana"); 
  }
};

// test actions
class HealAction : public BaseAction {
public:
  float cost = 1.0f;
  HealAction(float c = 1.0f) : cost(c) {}

  bool IsRelevant(const std::vector<std::unique_ptr<StateComponent>>& /*currentState*/,
                  const std::vector<std::unique_ptr<StateComponent>>& goal) const override {
    return std::any_of(goal.begin(), goal.end(), [](const auto& comp) {
        return dynamic_cast<const HealthStateComponent*>(comp.get()) != nullptr;
    });
  }

  std::vector<std::unique_ptr<StateComponent>> GenerateRequirements(
      const std::vector<std::unique_ptr<StateComponent>>& /*goal*/) override {
    std::vector<std::unique_ptr<StateComponent>> reqs;
    reqs.push_back(std::make_unique<ManaStateComponent>(10));
    return reqs;
  }

  std::vector<std::unique_ptr<StateComponent>> GenerateResults(
      const std::vector<std::unique_ptr<StateComponent>>& /*goal*/) override {
    std::vector<std::unique_ptr<StateComponent>> res;
    res.push_back(std::make_unique<HealthStateComponent>(15));
    return res;
  }
    
  std::unique_ptr<BaseAction> Clone() const override {
    return std::make_unique<HealAction>(cost);
  }
    
  float GetCost() const override { return cost; }
};

class ManaBoostAction : public BaseAction {
public:
  float cost = 1.0f;
  ManaBoostAction(float c = 1.0f) : cost(c) {}

  bool IsRelevant(const std::vector<std::unique_ptr<StateComponent>>& /*currentState*/,
                  const std::vector<std::unique_ptr<StateComponent>>& goal) const override {
    return std::any_of(goal.begin(), goal.end(), [](const auto& comp) {
        return dynamic_cast<const ManaStateComponent*>(comp.get()) != nullptr;
    });
  }

  std::vector<std::unique_ptr<StateComponent>> GenerateRequirements(
      const std::vector<std::unique_ptr<StateComponent>>& /*goal*/) override {
    return {}; // No requirements
  }

  std::vector<std::unique_ptr<StateComponent>> GenerateResults(
      const std::vector<std::unique_ptr<StateComponent>>& /*goal*/) override {
    std::vector<std::unique_ptr<StateComponent>> res;
    res.push_back(std::make_unique<ManaStateComponent>(20)); // Was 15, changed to 20 for variety
    return res;
  }
    
  std::unique_ptr<BaseAction> Clone() const override {
    return std::make_unique<ManaBoostAction>(cost);
  }

  float GetCost() const override { return cost; }
};

class LocationStateComponent : public StateComponent {
public:
	std::string locationName;

	LocationStateComponent(std::string name) : locationName(std::move(name)) {}

	std::unique_ptr<StateComponent> Clone() const override {
		return std::make_unique<LocationStateComponent>(locationName);
	}

	void AddValues(const StateComponent& otherComponent) override {
		const auto* other = dynamic_cast<const LocationStateComponent*>(&otherComponent);
		if (other) {
			locationName = other->locationName; // Replace current location
		}
	}

	void SubtractValues(const StateComponent& otherComponent) override {
		const auto* other = dynamic_cast<const LocationStateComponent*>(&otherComponent);
		if (other && other->locationName == locationName) {
			locationName.clear(); // Or set to a specific "undefined" location
		}
	}

	bool IsEmpty() const override {
		return locationName.empty();
	}

	bool operator==(const LocationStateComponent& other) const {
		return locationName == other.locationName;
	}

	size_t GetHash() const override {
		return std::hash<std::string>()(locationName) ^ std::hash<std::string>()("LocationStateComponent");
	}
};

class ItemStateComponent : public StateComponent {
public:
	std::string itemName;
	bool hasItem;

	ItemStateComponent(std::string name, bool has = true) : itemName(std::move(name)), hasItem(has) {}

	std::unique_ptr<StateComponent> Clone() const override {
		return std::make_unique<ItemStateComponent>(itemName, hasItem);
	}

	void AddValues(const StateComponent& otherComponent) override {
		const auto* other = dynamic_cast<const ItemStateComponent*>(&otherComponent);
		if (other && other->itemName == itemName) {
			hasItem = other->hasItem; // Set to the new state
		}
	}

	void SubtractValues(const StateComponent& otherComponent) override {
		const auto* other = dynamic_cast<const ItemStateComponent*>(&otherComponent);
		// If we are asked to subtract a "hasItem=true" state, it means we lose the item.
		if (other && other->itemName == itemName && other->hasItem) {
			hasItem = false;
		}
	}

	bool IsEmpty() const override {
		return !hasItem; // Empty if we don't have the item
	}

	bool operator==(const ItemStateComponent& other) const {
		return itemName == other.itemName && hasItem == other.hasItem;
	}

	size_t GetHash() const override {
		return std::hash<std::string>()(itemName) ^ std::hash<bool>()(hasItem) ^ std::hash<std::string>()("ItemStateComponent");
	}
};

class MoveToAction : public BaseAction {
public:
    std::string targetLocationName; // Set by GenerateRequirements based on the goal
    float cost;

    MoveToAction(float c = 1.0f)
        : targetLocationName(""), cost(c) {}

    std::unique_ptr<BaseAction> Clone() const override {
        auto clonedAction = std::make_unique<MoveToAction>(cost);
        clonedAction->targetLocationName = this->targetLocationName; // Copy configured target

        for(const auto& req_comp : this->requirements) {
            if(req_comp) clonedAction->requirements.push_back(req_comp->Clone());
        }
        for(const auto& res_comp : this->results) {
            if(res_comp) clonedAction->results.push_back(res_comp->Clone());
        }
        return clonedAction;
    }

    bool IsRelevant(const std::vector<std::unique_ptr<StateComponent>>& currentState,
                    const std::vector<std::unique_ptr<StateComponent>>& goal) const override {
        std::string currentActualLocation = "";
        for (const auto& comp : currentState) {
            if (const auto* locComp = dynamic_cast<const LocationStateComponent*>(comp.get())) {
                currentActualLocation = locComp->locationName;
                break;
            }
        }

        for (const auto& comp : goal) {
            if (const auto* locGoal = dynamic_cast<const LocationStateComponent*>(comp.get())) {
                if (!locGoal->locationName.empty() && locGoal->locationName != currentActualLocation) {
                    return true; // Relevant if goal wants a location we're not at.
                }
            }
        }
        return false;
    }

    std::vector<std::unique_ptr<StateComponent>> GenerateRequirements(
        const std::vector<std::unique_ptr<StateComponent>>& goal) override {
        // --- Action-specific parameter setting happens here ---
        this->targetLocationName = ""; // Reset before finding a new target from the goal
        
        // Find the first suitable location goal to set as this action's target.
        // Ideally, IsRelevant has already ensured such a goal exists and we're not there.
        for (const auto& comp : goal) {
            if (const auto* locGoal = dynamic_cast<const LocationStateComponent*>(comp.get())) {
                if (!locGoal->locationName.empty()) {
                    this->targetLocationName = locGoal->locationName;
                    break; 
                }
            }
        }
        // --- End of action-specific parameter setting ---

        // For a simple move, requirements are typically empty.
        // If we needed to be at a "StartLocation" for this generic MoveToAction to be valid,
        // that logic would be more complex or part of a different action type.
        return {};
    }

    std::vector<std::unique_ptr<StateComponent>> GenerateResults(
        const std::vector<std::unique_ptr<StateComponent>>& /*goal*/) override {
        std::vector<std::unique_ptr<StateComponent>> res;
        if (!targetLocationName.empty()) { // targetLocationName should be set by GenerateRequirements
            res.push_back(std::make_unique<LocationStateComponent>(targetLocationName));
        }
        return res;
    }
    
    float GetCost() const override { return cost; }
};

class PickupItemAction : public BaseAction {
public:
    std::string itemName;           // This will be set by GenerateRequirements from the goal
    std::string itemLocationName;   // Location where this action can pick items up (fixed per instance type)
    float cost;

    // Constructor now takes the location where this specific pickup action operates.
    PickupItemAction(std::string fixedPickupLocation, float c = 1.0f)
        : itemName(""), itemLocationName(std::move(fixedPickupLocation)), cost(c) {}

    std::unique_ptr<BaseAction> Clone() const override {
        // Construct with the fixed parameters for this type.
        auto clonedAction = std::make_unique<PickupItemAction>(this->itemLocationName, this->cost);
        clonedAction->itemName = this->itemName; // Copy the itemName configured from the goal

        for(const auto& req : this->requirements) { if(req) clonedAction->requirements.push_back(req->Clone()); }
        for(const auto& res : this->results) { if(res) clonedAction->results.push_back(res->Clone()); }
        return clonedAction;
    }

    bool IsRelevant(const std::vector<std::unique_ptr<StateComponent>>& currentState,
                    const std::vector<std::unique_ptr<StateComponent>>& goal) const override {
        std::string targetItemNameInGoal = "";
        bool goalWantsAnItem = false;

        for (const auto& comp : goal) {
            if (const auto* itemGoal = dynamic_cast<const ItemStateComponent*>(comp.get())) {
                if (itemGoal->hasItem && !itemGoal->itemName.empty()) {
                    targetItemNameInGoal = itemGoal->itemName;
                    goalWantsAnItem = true;
                    break;
                }
            }
        }
        if (!goalWantsAnItem) return false;

        for (const auto& comp : currentState) {
            if (const auto* currentItem = dynamic_cast<const ItemStateComponent*>(comp.get())) {
                if (currentItem->itemName == targetItemNameInGoal && currentItem->hasItem) {
                    return false; // Already have the item
                }
            }
        }
        return true; // Relevant if goal wants an item we don't have.
    }

    std::vector<std::unique_ptr<StateComponent>> GenerateRequirements(
        const std::vector<std::unique_ptr<StateComponent>>& goal) override {
        // --- Action-specific parameter setting ---
        this->itemName = ""; // Reset
        for (const auto& comp : goal) {
            if (const auto* itemGoal = dynamic_cast<const ItemStateComponent*>(comp.get())) {
                if (itemGoal->hasItem && !itemGoal->itemName.empty()) {
                    this->itemName = itemGoal->itemName; // Set the item to pick up
                    break;
                }
            }
        }
        // --- End of action-specific parameter setting ---

        std::vector<std::unique_ptr<StateComponent>> reqs;
        if (!this->itemName.empty()) { // Only add requirements if an item was identified
            // Requirement: Be at the location where this action can pick up items.
            reqs.push_back(std::make_unique<LocationStateComponent>(this->itemLocationName));
        }
        return reqs;
    }

    std::vector<std::unique_ptr<StateComponent>> GenerateResults(
        const std::vector<std::unique_ptr<StateComponent>>& /*goal*/) override {
        std::vector<std::unique_ptr<StateComponent>> res;
        if (!itemName.empty()) { // itemName should be set by GenerateRequirements
            res.push_back(std::make_unique<ItemStateComponent>(itemName, true));
        }
        return res;
    }
    
    float GetCost() const override { return cost; }
};

// tests
TEST(StateComponentUtilsTest, CombineHealthAndMana) {
    std::vector<std::unique_ptr<StateComponent>> listA;
    listA.push_back(std::make_unique<HealthStateComponent>(5));

    std::vector<std::unique_ptr<StateComponent>> listB;
    listB.push_back(std::make_unique<HealthStateComponent>(10));

    auto result = StateComponentUtils::CombineComponentLists(listA, listB);
    ASSERT_EQ(result.size(), 1);
    auto* healthComp = dynamic_cast<HealthStateComponent*>(result[0].get());
    ASSERT_NE(healthComp, nullptr);
    EXPECT_EQ(healthComp->value, 15);
}

TEST(StateComponentUtilsTest, RemoveHealth) {
    std::vector<std::unique_ptr<StateComponent>> base;
    base.push_back(std::make_unique<HealthStateComponent>(15));

    std::vector<std::unique_ptr<StateComponent>> toRemove;
    toRemove.push_back(std::make_unique<HealthStateComponent>(10));

    auto result = StateComponentUtils::RemoveComponentList(base, toRemove);
    ASSERT_EQ(result.size(), 1);
    auto* healthComp = dynamic_cast<HealthStateComponent*>(result[0].get());
    ASSERT_NE(healthComp, nullptr);
    EXPECT_EQ(healthComp->value, 5); // 15 - 10 = 5, not empty

    std::vector<std::unique_ptr<StateComponent>> toRemoveAll;
    toRemoveAll.push_back(std::make_unique<HealthStateComponent>(15));
    result = StateComponentUtils::RemoveComponentList(base, toRemoveAll);
    ASSERT_EQ(result.size(), 0); // 15 - 15 = 0, so empty and removed
}


TEST(ActionGeneratorTest, GeneratesRelevantActions_HealthMana) {
    std::vector<std::unique_ptr<StateComponent>> currentState;
    currentState.push_back(std::make_unique<HealthStateComponent>(10));
    currentState.push_back(std::make_unique<ManaStateComponent>(5));

    std::vector<std::unique_ptr<StateComponent>> goalState;
    goalState.push_back(std::make_unique<HealthStateComponent>(30)); // Goal is more health

    ActionGenerator<HealAction, ManaBoostAction> generator;
    auto actions = generator.GenerateActions(currentState, goalState);

    ASSERT_EQ(actions.size(), 1); // Only HealAction is relevant to a Health goal
    EXPECT_NE(dynamic_cast<HealAction*>(actions[0].get()), nullptr);
}

TEST(ActionGeneratorTest, GeneratedActionHasCorrectRequirementsAndResults_HealthMana) {
    std::vector<std::unique_ptr<StateComponent>> currentState; // Not strictly needed for this test part
    std::vector<std::unique_ptr<StateComponent>> goalState;
    goalState.push_back(std::make_unique<HealthStateComponent>(30));

    ActionGenerator<HealAction> generator;
    auto actions = generator.GenerateActions(currentState, goalState);
    ASSERT_EQ(actions.size(), 1);

    BaseAction* action = actions[0].get();
    ASSERT_NE(action, nullptr);

    // Action should have been configured, populating requirements and results
    ASSERT_FALSE(action->requirements.empty());
    ASSERT_FALSE(action->results.empty());

    const auto& reqs = action->requirements;
    ASSERT_EQ(reqs.size(), 1);
    const auto* manaReq = dynamic_cast<ManaStateComponent*>(reqs[0].get());
    ASSERT_NE(manaReq, nullptr);
    EXPECT_EQ(manaReq->value, 10);

    const auto& results = action->results;
    ASSERT_EQ(results.size(), 1);
    const auto* healthRes = dynamic_cast<HealthStateComponent*>(results[0].get());
    ASSERT_NE(healthRes, nullptr);
    EXPECT_EQ(healthRes->value, 15);
}

TEST(StateComponentUtilsTest, LocationCombine_ReplacesLocation) {
    std::vector<std::unique_ptr<StateComponent>> base;
    base.push_back(std::make_unique<LocationStateComponent>("StartPoint"));

    std::vector<std::unique_ptr<StateComponent>> add;
    add.push_back(std::make_unique<LocationStateComponent>("EndPoint"));

    auto result = StateComponentUtils::CombineComponentLists(base, add);
    ASSERT_EQ(result.size(), 1);
    auto* locComp = dynamic_cast<LocationStateComponent*>(result[0].get());
    ASSERT_NE(locComp, nullptr);
    EXPECT_EQ(locComp->locationName, "EndPoint");
}

TEST(StateComponentUtilsTest, ItemCombine_UpdatesPossession) {
    std::vector<std::unique_ptr<StateComponent>> base;
    base.push_back(std::make_unique<ItemStateComponent>("Key", false)); // Initially don't have key

    std::vector<std::unique_ptr<StateComponent>> add;
    add.push_back(std::make_unique<ItemStateComponent>("Key", true)); // Add state "has key"

    auto result = StateComponentUtils::CombineComponentLists(base, add);
    ASSERT_EQ(result.size(), 1);
    auto* itemComp = dynamic_cast<ItemStateComponent*>(result[0].get());
    ASSERT_NE(itemComp, nullptr);
    EXPECT_EQ(itemComp->itemName, "Key");
    EXPECT_TRUE(itemComp->hasItem);
}

TEST(StateComponentUtilsTest, LocationRemove_ClearsLocationIfMatch) {
    std::vector<std::unique_ptr<StateComponent>> base;
    base.push_back(std::make_unique<LocationStateComponent>("SomePlace"));

    std::vector<std::unique_ptr<StateComponent>> remove;
    remove.push_back(std::make_unique<LocationStateComponent>("SomePlace"));

    auto result = StateComponentUtils::RemoveComponentList(base, remove);
    ASSERT_EQ(result.size(), 0); // Location becomes empty, so component is removed
}

TEST(StateComponentUtilsTest, ItemRemove_SetsNotPossessed) {
    std::vector<std::unique_ptr<StateComponent>> base;
    base.push_back(std::make_unique<ItemStateComponent>("Sword", true));

    std::vector<std::unique_ptr<StateComponent>> remove;
    // To remove "Sword", we subtract a component that says "Sword is possessed".
    // The SubtractValues logic should then set hasItem to false.
    remove.push_back(std::make_unique<ItemStateComponent>("Sword", true)); 

    auto result = StateComponentUtils::RemoveComponentList(base, remove);
    // ItemStateComponent("Sword", false) is !IsEmpty(), so it should remain if not empty.
    // IsEmpty for ItemStateComponent means !hasItem. If after subtraction hasItem is false,
    // it should be removed if IsEmpty() returns true.
    // Current ItemStateComponent::IsEmpty() is `return !hasItem;`
    // So if hasItem becomes false, IsEmpty() is true, and it's removed.
    ASSERT_EQ(result.size(), 0); 
}

TEST(ActionGeneratorTest, MoveToAction_ConfiguresForGoalLocation)
{
    std::vector<std::unique_ptr<StateComponent>> currentState;
    currentState.push_back(std::make_unique<LocationStateComponent>("Start"));

    std::vector<std::unique_ptr<StateComponent>> goalState;
    goalState.push_back(std::make_unique<LocationStateComponent>("End"));

    ActionGenerator<MoveToAction> generator;
    auto actions = generator.GenerateActions(currentState, goalState);

    ASSERT_EQ(actions.size(), 1);
    MoveToAction* moveAction = dynamic_cast<MoveToAction*>(actions[0].get());
    ASSERT_NE(moveAction, nullptr);

    // targetLocationName is set inside GenerateRequirements, which is called by BaseAction::Configure
    EXPECT_EQ(moveAction->targetLocationName, "End"); 

    ASSERT_EQ(moveAction->results.size(), 1);
    LocationStateComponent* resultLoc = dynamic_cast<LocationStateComponent*>(moveAction->results[0].get());
    ASSERT_NE(resultLoc, nullptr);
    EXPECT_EQ(resultLoc->locationName, "End");
    EXPECT_TRUE(moveAction->requirements.empty());

    std::vector<std::unique_ptr<StateComponent>> currentStateAtEnd;
    currentStateAtEnd.push_back(std::make_unique<LocationStateComponent>("End"));
    actions = generator.GenerateActions(currentStateAtEnd, goalState);
    EXPECT_EQ(actions.size(), 0);
}