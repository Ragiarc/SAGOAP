#include "gtest/gtest.h"
#include "../SAGOAP/SAGOAP.cpp"

// test components
class HealthStateComponent : public StateComponent {
public:
  int value;

  HealthStateComponent(int v) : value(v) {}

  std::unique_ptr<StateComponent> Clone() const override {
    return std::make_unique<HealthStateComponent>(value);
  }

  void AddValues(const StateComponent& otherComponent) override {
    const auto& other = dynamic_cast<const HealthStateComponent&>(otherComponent);
    value += other.value;
  }

  void SubtractValues(const StateComponent& otherComponent) override {
    const auto& other = dynamic_cast<const HealthStateComponent&>(otherComponent);
    value -= other.value;
  }

  bool IsEmpty() const override {
    return value <= 0;
  }

  bool operator==(const HealthStateComponent& other) const {
    return value == other.value;
  }

  size_t GetHash() const override
  {
    return std::hash<int>()(value);
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
    const auto& other = dynamic_cast<const ManaStateComponent&>(otherComponent);
    value += other.value;
  }

  void SubtractValues(const StateComponent& otherComponent) override {
    const auto& other = dynamic_cast<const ManaStateComponent&>(otherComponent);
    value -= other.value;
  }

  bool IsEmpty() const override {
    return value <= 0;
  }

  bool operator==(const ManaStateComponent& other) const {
    return value == other.value;
  }

  size_t GetHash() const override 
  {
    return std::hash<int>()(value);
  }
};

// test actions
class HealAction : public BaseAction {
public:
  bool IsRelevant(const std::unique_ptr<StateComponent>& goal) const override {
    // Relevant if the goal is to have more health
    const auto* healthGoal = dynamic_cast<const HealthStateComponent*>(goal.get());
    return healthGoal != nullptr;
  }

  std::vector<std::unique_ptr<StateComponent>> GenerateRequirements(const std::unique_ptr<StateComponent>& goal) override {
    // Requires 10 mana
    std::vector<std::unique_ptr<StateComponent>> requirements;
    auto manaRequirement = new ManaStateComponent(10);
    requirements.push_back(std::move(manaRequirement->Clone()));
    return requirements;
  }

  std::vector<std::unique_ptr<StateComponent>> GenerateResults(const std::unique_ptr<StateComponent>& goal) override {
    // Heals 15 health
    std::vector<std::unique_ptr<StateComponent>> results;
    auto healthResult = new HealthStateComponent(15);
    results.push_back(std::move(healthResult->Clone()));
    return results;
  }
};

class ManaBoostAction : public BaseAction {
public:
  bool IsRelevant(const std::unique_ptr<StateComponent>& goal) const override {
    // Relevant if the goal is to have more mana
    const auto* manaGoal = dynamic_cast<const ManaStateComponent*>(goal.get());
    return manaGoal != nullptr;
  }

  std::vector<std::unique_ptr<StateComponent>> GenerateRequirements(const std::unique_ptr<StateComponent>& goal) override {
    // No requirements
    return {};
  }

  std::vector<std::unique_ptr<StateComponent>> GenerateResults(const std::unique_ptr<StateComponent>& goal) override {
    // Boosts 20 mana
    std::vector<std::unique_ptr<StateComponent>> results;
    auto manaResult = new ManaStateComponent(15);
    results.push_back(std::move(manaResult->Clone()));
    return results;
  }
};
// tests
// component tests
TEST(CombineComponentListsTest, MatchingComponentTypesAddAsExpected) {
  std::vector<std::unique_ptr<StateComponent>> listA;
  HealthStateComponent a(5);
  listA.push_back(a.Clone());

  std::vector<std::unique_ptr<StateComponent>> listB;
  listB.push_back(HealthStateComponent(10).Clone());

  auto result = StateComponentUtils::CombineComponentLists(listA, listB);

  ASSERT_EQ(result.size(), 1);  
  EXPECT_EQ(dynamic_cast<HealthStateComponent*>(result[0].get())->value, 15);

  auto result2 = StateComponentUtils::CombineComponentLists(listB, listA);
  ASSERT_EQ(result2.size(), 1);  
  EXPECT_EQ(dynamic_cast<HealthStateComponent*>(result2[0].get())->value, 15);

  auto result3 = StateComponentUtils::CombineComponentLists(result2, listA);
  ASSERT_EQ(result3.size(), 1);  
  EXPECT_EQ(dynamic_cast<HealthStateComponent*>(result3[0].get())->value, 20);
}

TEST(CombineComponentListsTest, DifferentComponentTypesAddAsExpected) {
  std::vector<std::unique_ptr<StateComponent>> listA;
  HealthStateComponent a(5);
  listA.push_back(a.Clone());

  std::vector<std::unique_ptr<StateComponent>> listB;
  listB.push_back(ManaStateComponent(10).Clone());

  auto result = StateComponentUtils::CombineComponentLists(listA, listB);

  ASSERT_EQ(result.size(), 2);  
  EXPECT_EQ(dynamic_cast<HealthStateComponent*>(result[0].get())->value, 5);
  EXPECT_EQ(dynamic_cast<ManaStateComponent*>(result[1].get())->value, 10);
  
}

TEST(RemoveComponentListsTest, MatchingComponentTypesSubtractAsExpected) {
  std::vector<std::unique_ptr<StateComponent>> listA;
  HealthStateComponent a(5);
  listA.push_back(a.Clone());

  std::vector<std::unique_ptr<StateComponent>> listB;
  listB.push_back(HealthStateComponent(10).Clone());

  auto result = StateComponentUtils::RemoveComponentList(listA, listB);

  ASSERT_EQ(result.size(), 0);  

  auto result2 = StateComponentUtils::RemoveComponentList(listB, listA);
  ASSERT_EQ(result2.size(), 1);  
  EXPECT_EQ(dynamic_cast<HealthStateComponent*>(result2[0].get())->value, 5);

  auto result3 = StateComponentUtils::RemoveComponentList(result2, listA);
  ASSERT_EQ(result3.size(), 0);  
}

TEST(RemoveComponentListsTest, DifferentComponentTypesSubtractAsExpected) {
  std::vector<std::unique_ptr<StateComponent>> listA;
  HealthStateComponent a(5);
  listA.push_back(a.Clone());

  std::vector<std::unique_ptr<StateComponent>> listB;
  listB.push_back(ManaStateComponent(10).Clone());

  auto result = StateComponentUtils::RemoveComponentList(listA, listB);

  ASSERT_EQ(result.size(), 1);  
  EXPECT_EQ(dynamic_cast<HealthStateComponent*>(result[0].get())->value, 5);
  
}

// action generation tests


TEST(ActionGeneratorTest, GeneratesRelevantActions) {
    // Create a goal: have 20 health
    HealthStateComponent healthGoal(20);

    // Create an ActionGenerator that can generate HealAction and ManaBoostAction
    ActionGenerator<HealAction, ManaBoostAction> generator;

    // Generate actions
    auto actions = generator.GenerateActions(healthGoal.Clone());

    // We expect only the HealAction to be generated (ManaBoost is not relevant)
    ASSERT_EQ(actions.size(), 1);
    EXPECT_TRUE(dynamic_cast<HealAction*>(actions[0].get()) != nullptr);
}

TEST(ActionGeneratorTest, GeneratedActionHasCorrectRequirementsAndResults) {
    // Goal: 30 health
    HealthStateComponent healthGoal(30);
    ActionGenerator<HealAction> generator;

    auto actions = generator.GenerateActions(healthGoal.Clone());
    ASSERT_EQ(actions.size(), 1);

    HealAction* healAction = dynamic_cast<HealAction*>(actions[0].get());
    ASSERT_NE(healAction, nullptr);

    // Verify requirements

    const auto& requirements = healAction->requirements;
    ASSERT_EQ(requirements.size(), 1);
    const auto* manaRequirement = dynamic_cast<ManaStateComponent*>(requirements[0].get());
    ASSERT_NE(manaRequirement, nullptr);
    EXPECT_EQ(manaRequirement->value, 10);

    // Verify results
    const auto& results = healAction->results;
    ASSERT_EQ(results.size(), 1);
    const auto* healthResult = dynamic_cast<HealthStateComponent*>(results[0].get());
    ASSERT_NE(healthResult, nullptr);
    EXPECT_EQ(healthResult->value, 15);
}
