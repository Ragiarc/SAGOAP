#include "gtest/gtest.h"
#include "../SAGOAP/SAGOAP.cpp"

class TestStateComponent : public StateComponent {
public:
  int value;

  TestStateComponent(int v) : value(v) {}

  std::unique_ptr<StateComponent> Clone() const override {
    return std::make_unique<TestStateComponent>(value);
  }

  void AddValues(const StateComponent& otherComponent) override {
    const auto& other = dynamic_cast<const TestStateComponent&>(otherComponent);
    value += other.value;
  }

  /*void SubtractValues(std::unique_ptr<StateComponent> otherComponent) override {
    const auto& other = dynamic_cast<const TestStateComponent&>(otherComponent);
    value -= other.value;
  }*/

  bool IsEmpty() const override {
    return value <= 0;
  }

  bool operator==(const TestStateComponent& other) const {
    return value == other.value;
  }
};


TEST(CombineComponentListsTest, CombinesWithoutOverlap) {
  std::vector<std::unique_ptr<StateComponent>> listA;
  TestStateComponent a(5);
  listA.push_back(a.Clone());

  std::vector<std::unique_ptr<StateComponent>> listB;
  listB.push_back(TestStateComponent(10).Clone());

  auto result = StateComponentUtils::CombineComponentLists(listA, listB);

  ASSERT_EQ(result.size(), 1);  // One from each list
  EXPECT_EQ(dynamic_cast<TestStateComponent*>(result[0].get())->value, 15);
  //EXPECT_EQ(dynamic_cast<TestStateComponent&>(result[1]).value, 10);
}

TEST(TestCaseName, TestName)
{
  EXPECT_EQ(1, 1);
  EXPECT_TRUE(true);
  
}
