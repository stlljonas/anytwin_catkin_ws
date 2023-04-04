// c++
#include <algorithm>

// series elastic actuator
#include "series_elastic_actuator/common.hpp"


namespace series_elastic_actuator {


double saturate(double value, double min, double max)
{
  return std::min(std::max(value, min), max);
}


} // series_elastic_actuator
