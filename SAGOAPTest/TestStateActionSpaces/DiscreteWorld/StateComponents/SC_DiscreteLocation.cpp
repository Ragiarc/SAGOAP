#include "SC_DiscreteLocation.h"

void SC_DiscreteLocation::AddValues(const StateComponent& otherComponent)
{
    const auto& other = dynamic_cast<const SC_DiscreteLocation&>(otherComponent);
    this->x_location += other.x_location;
    this->y_location += other.y_location;
}

void SC_DiscreteLocation::SubtractValues(const StateComponent& otherComponent)
{
    const auto& other = dynamic_cast<const SC_DiscreteLocation&>(otherComponent);
    this->x_location -= other.x_location;
    this->y_location -= other.y_location;
}

bool SC_DiscreteLocation::IsEmpty() const
{
    return false;
}
