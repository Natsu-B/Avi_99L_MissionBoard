#include "control/zero_hold.hpp"

#include <algorithm>
#include <cmath>

namespace control {
namespace {

bool validConfig(const ZeroHoldConfig &config) {
  return std::isfinite(config.kp_nm_per_rad) && config.kp_nm_per_rad >= 0.0 &&
         std::isfinite(config.kd_nm_s_per_rad) &&
         config.kd_nm_s_per_rad >= 0.0 &&
         std::isfinite(config.torque_limit_nm) && config.torque_limit_nm > 0.0 &&
         std::isfinite(config.angle_dead_zone_rad) &&
         config.angle_dead_zone_rad >= 0.0 &&
         std::isfinite(config.rate_dead_zone_rad_s) &&
         config.rate_dead_zone_rad_s >= 0.0;
}

double continuousDeadZone(double value, double width) {
  const double residual = std::max(std::abs(value) - width, 0.0);
  if (residual == 0.0)
    return 0.0;
  return value < 0.0 ? -residual : residual;
}

} // namespace

ZeroHoldOutput computeZeroHold(const ZeroHoldInput &input,
                               const ZeroHoldConfig &config) {
  if (!validConfig(config) || !std::isfinite(input.angle_rad) ||
      !std::isfinite(input.rate_rad_s))
    return {};

  const double angle =
      continuousDeadZone(input.angle_rad, config.angle_dead_zone_rad);
  const double rate =
      continuousDeadZone(input.rate_rad_s, config.rate_dead_zone_rad_s);
  const double raw =
      -config.kp_nm_per_rad * angle - config.kd_nm_s_per_rad * rate;
  if (!std::isfinite(raw))
    return {};

  const double limited =
      std::clamp(raw, -config.torque_limit_nm, config.torque_limit_nm);
  return {raw, limited, limited != raw, true};
}

} // namespace control
