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
}
