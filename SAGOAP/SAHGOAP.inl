#pragma once
#include <queue>
#include <unordered_set>
#include <algorithm>

namespace SAHGOAP
{

	template<typename ComponentType>
	void WorldModel::RegisterComponentHasher(std::function<size_t(const ComponentType&)> hasher)
	{
		int id = internal::ComponentTypeID<ComponentType>::Get();
		
		// Type-erase the specific hasher into a generic std::any hasher
		HasherFunction erased_hasher = [hasher](const std::any& component) {
			return hasher(*std::any_cast<const ComponentType>(&component));
		};

		if (id >= registered_hashers.size()) {
			registered_hashers.resize(id + 1);
		}
		registered_hashers[id] = erased_hasher;
	}
}
