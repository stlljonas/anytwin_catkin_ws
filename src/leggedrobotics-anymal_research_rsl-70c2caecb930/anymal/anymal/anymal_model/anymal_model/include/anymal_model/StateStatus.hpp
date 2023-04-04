#pragma once

namespace anymal_model {

enum class StateStatus : int { STATUS_ERROR_SENSOR = -3, STATUS_ERROR_ESTIMATOR = -2, STATUS_ERROR_UNKNOWN = -1, STATUS_OK = 0 };

}  // namespace anymal_model
