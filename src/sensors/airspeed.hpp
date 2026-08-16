#pragma once

#include <array>
#include <cstddef>

namespace sensors {

struct AirspeedResult {
  double airspeed_mps{};
  bool valid{};
};

[[nodiscard]] AirspeedResult
computeSaintVenantAirspeed(double static_pressure_pa,
                          double differential_pressure_pa,
                          double temperature_celsius,
                          double pitot_pressure_correction_coefficient);

class DifferentialPressureFilter {
public:
  static constexpr std::size_t kMaximumSamples = 32;

  DifferentialPressureFilter(double zero_offset_pa,
                             double negative_tolerance_pa,
                             std::size_t moving_average_samples);

  void reset();
  [[nodiscard]] bool update(double raw_pressure_pa, double &filtered_pressure_pa);

private:
  double zero_offset_pa_{};
  double negative_tolerance_pa_{};
  std::size_t moving_average_samples_{};
  std::array<double, kMaximumSamples> samples_{};
  std::size_t head_{};
  std::size_t count_{};
  double sum_{};
};

} // namespace sensors
