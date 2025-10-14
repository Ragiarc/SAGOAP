#pragma once
#include <queue>
#include <unordered_set>
#include <algorithm>

namespace SAHGOAP
{
	template<typename T>
    T* AgentState::AddComponent()
	{
		auto& prop = components[std::type_index(typeid(T))];
		if (!prop.has_value()) {
			prop = T{};
		}
		return std::any_cast<T>(&prop);
	}

	template<typename T>
	const T* AgentState::GetComponent() const
	{
		auto it = components.find(std::type_index(typeid(T)));
		if (it == components.end()) {
			return nullptr;
		}
		return std::any_cast<const T>(&it->second);
	}

	template<typename ComponentType>
	void WorldModel::RegisterComponentHasher(std::function<size_t(const ComponentType&)> hasher)
	{
		std::type_index type_idx = std::type_index(typeid(ComponentType));

		// Create a type-erased lambda that captures the user's strongly-typed hasher.
		// It takes a const std::any&, attempts to cast it to the correct component type,
		// and then calls the user's hasher.
		registered_hashers[type_idx] = [hasher](const std::any& component_any) -> size_t {
			// Attempt to cast the std::any to the specific component type.
			if (const auto* comp = std::any_cast<ComponentType>(&component_any)) {
				// If successful, call the user's provided hasher function.
				return hasher(*comp);
			}
			// Return 0 if the cast fails (should not happen in a correctly structured state).
			return 0;
		};
	}
}
