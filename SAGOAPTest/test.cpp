#include "gtest/gtest.h"
#include "../SAGOAP/SAGOAP.cpp"

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

  bool operator==(const HealthStateComponent& other) const {
    return value == other.value;
  }
};


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
