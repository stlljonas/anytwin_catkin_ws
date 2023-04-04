#pragma once

// anymal description
#include <anymal_description/AnymalDescription.hpp>

namespace anymal_model_ros {

// ANYmal description abbreviations
// Note: These typedefs reduce the amount of code in the anymal_model_ros package.
//       If you need any of these types, depend on the anymal_description package directly!
using CAD = anymal_description::ConcreteAnymalDescription;
using AD = anymal_description::AnymalDescription;
using AT = anymal_description::AnymalTopology;

}  // namespace anymal_model_ros
