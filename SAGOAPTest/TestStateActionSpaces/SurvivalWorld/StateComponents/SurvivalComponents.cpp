/*
#include "SurvivalComponents.h"

bool LocationComponent::IsEmpty() const
{ return locationId < 0; }

size_t LocationComponent::GetHash() const
{
	size_t typeHash = typeid(LocationComponent).hash_code();
	size_t valueHash = std::hash<int>()(locationId);
	return typeHash ^ (valueHash + 0x9e3779b9 + (typeHash << 6) + (typeHash >> 2));
}

bool LocationComponent::operator==(const LocationComponent& other) const
{ return locationId == other.locationId; }
*/
