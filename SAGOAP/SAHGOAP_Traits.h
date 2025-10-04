#pragma once
#include <string>
#include <map>

namespace SAHGOAP::Traits
{
	// --- The Core Concept: A Trait Struct ---
	// The library looks for a specialization of this struct for the user's type.
	template<typename T>
	struct StateTypeTrait; // Intentionally left undefined. User must specialize.

	// ========================================================================
	// Library-Provided Trait Implementations (for developer to inherit from)
	// ========================================================================

	// A pre-made trait for structs that have a single std::string member.
	template<auto MemberPtr>
	struct SymbolicValueTrait {
		static const std::string& GetValue(const auto& instance) {
			return instance.*MemberPtr;
		}
	};

	// A pre-made trait for structs that have a std::map<string, int> member.
	template<auto MemberPtr>
	struct SymbolicMapTrait {
		static const std::map<std::string, int>& GetMap(const auto& instance) {
			return instance.*MemberPtr;
		}
	};

	struct ComplexMapTrait {
		// Developer must provide:
		// using KeyType = std::string;
		// using ValueType = YourStructType;
		// static const std::map<KeyType, ValueType>& GetMap(const MyType& instance);
		//
		// struct FieldProvider {
		//    static void ForEachField(auto functor);
		// };
	};
	
	// ========================================================================
	// User-Facing Macros (to reduce boilerplate)
	// ========================================================================

	// Macro for the most common case: a struct with a single std::string member.
#define SAHGOAP_SYMBOLIC_VALUE_TRAIT(StructType, MemberName) \
template<> \
struct SAHGOAP::Traits::StateTypeTrait<StructType> \
: SAHGOAP::Traits::SymbolicValueTrait<&StructType::MemberName> {}

	// Macro for the second most common case: a struct with a std::map<string, int> member.
#define SAHGOAP_SYMBOLIC_MAP_TRAIT(StructType, MemberName) \
template<> \
struct SAHGOAP::Traits::StateTypeTrait<StructType> \
: SAHGOAP::Traits::SymbolicMapTrait<&StructType::MemberName> {}

// Begins the trait definition for a complex map.
#define SAHGOAP_BEGIN_COMPLEX_MAP_TRAIT(StructType, MapMember, ValueStructType) \
template<> \
struct SAHGOAP::Traits::StateTypeTrait<StructType> : SAHGOAP::Traits::ComplexMapTrait { \
using KeyType = std::string; \
using ValueType = ValueStructType; \
static const std::map<KeyType, ValueType>& GetMap(const StructType& instance) { \
return instance.MapMember; \
} \
struct FieldProvider { \
static void ForEachField(auto& functor) { \
using ParentStruct = StructType; \
using CurrentValueStruct = ValueStructType;

// Declares an integer field.
// IT NOW PASSES THE TYPES TO VISIT.
#define SAHGOAP_COMPLEX_MAP_FIELD_INT(FieldName) \
functor.template visit<ParentStruct, CurrentValueStruct, int>(#FieldName, &CurrentValueStruct::FieldName);

// Declares a symbolic field.
// IT NOW PASSES THE TYPES TO VISIT.
#define SAHGOAP_COMPLEX_MAP_FIELD_SYMBOL(FieldName, SymbolicTypeName) \
functor.template visit<ParentStruct, CurrentValueStruct, std::string>(#FieldName, &CurrentValueStruct::FieldName, SymbolicTypeName);

// Ends the trait definition.
#define SAHGOAP_END_COMPLEX_MAP_TRAIT() \
} \
}; \
};
	
} // namespace SAHGOAP::Traits