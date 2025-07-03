/*
#pragma once
#include "SAGOAP.h"

struct Recipe {
	std::vector<std::unique_ptr<StateComponent>> requirements; // e.g., Wood(2), Stone(1)
	std::vector<std::unique_ptr<StateComponent>> yield; // e.g., Axe(1)
};

struct HarvestInfo {
	std::unique_ptr<StateComponent> requiredTool; // e.g., AxeComponent() (value might not matter here, just type)
	std::vector<std::unique_ptr<StateComponent>> yield;       // e.g., Wood(5)
	std::unique_ptr<StateComponent> depletedState; // Optional: What it turns into, e.g., StumpComponent()
	// float harvestTime = 2.0f; // Optional
};

class LocationComponent : public StateComponent {
public:
	int locationId = -1; // Or use x, y coordinates, etc.

	LocationComponent(int id) : locationId(id) {}

	// --- StateComponent Overrides ---
	void AddValues(const StateComponent& otherComponent) override { /* Location likely doesn't add #1# }
	void SubtractValues(const StateComponent& otherComponent) override { /* Location likely doesn't subtract #1# }
	std::unique_ptr<StateComponent> Clone() const override { return std::make_unique<LocationComponent>(locationId); }
	bool IsEmpty() const override;
	// Example: Invalid ID is empty
	size_t GetHash() const override;
	bool operator==(const LocationComponent& other) const;
};


class SurvivalStateComponent : public StateComponent
{
	public:
	~SurvivalStateComponent() override = default;

	virtual const Recipe* GetRecipeInfo() const { return nullptr; }
	virtual const HarvestInfo* GetHarvestInfo() const { return nullptr; }
	virtual const LocationComponent* GetLocationComponent() const { return nullptr; }
	
};



class WoodSC : public SurvivalStateComponent
{
public:
	void AddValues(const StateComponent& otherComponent) override;
	void SubtractValues(const StateComponent& otherComponent) override;
	std::unique_ptr<StateComponent> Clone() const override;
	bool IsEmpty() const override;
	size_t GetHash() const override;
	~WoodSC() override;
	const Recipe* GetRecipeInfo() const override;
	const HarvestInfo* GetHarvestInfo() const override;
	const LocationComponent* GetLocationComponent() const override;
};
*/


