#pragma once

#include "esp_err.h"

namespace actuators::safe_outputs {

[[nodiscard]] esp_err_t initialize();
[[nodiscard]] esp_err_t motorCoast();
[[nodiscard]] esp_err_t setAux5v(bool enabled);
[[nodiscard]] esp_err_t setParaPower(bool enabled);
[[nodiscard]] bool initialized();

} // namespace actuators::safe_outputs
