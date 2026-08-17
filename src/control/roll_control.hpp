#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace control {

struct GainPoint {
  double airspeed_mps{};
  std::array<double, 4> gain{};
};

struct RollControlInput {
  double roll_deviation_rad{};
  double fin_angle_rad{};
  double roll_rate_rad_s{};
  double fin_rate_rad_s{};
  double airspeed_mps{};
};

struct RollControlOutput {
  double torque_nm{};
  bool saturated{};
  bool valid{};
};

struct ControlInputHealth {
  bool attitude_available{};
  bool fin_available{};
  bool lps_available{};
  bool ssc_available{};
  bool airspeed_available{};
};

[[nodiscard]] bool
allControlInputsAvailable(const ControlInputHealth &health);

class RollController {
public:
  explicit RollController(const std::array<GainPoint, 7> &schedule);

  [[nodiscard]] RollControlOutput
  compute(const RollControlInput &input) const;

private:
  std::array<GainPoint, 7> schedule_{};
};

class RollEstimator {
public:
  explicit RollEstimator(uint64_t maximum_gap_us);

  void reset();
  [[nodiscard]] bool start(uint64_t timestamp_us, double roll_rate_rad_s);
  [[nodiscard]] bool observe(uint64_t timestamp_us, bool sample_valid,
                             double roll_rate_rad_s);

  [[nodiscard]] bool started() const { return started_; }
  [[nodiscard]] bool valid() const { return valid_; }
  [[nodiscard]] double deviationRad() const { return deviation_rad_; }
  [[nodiscard]] uint64_t lastSampleUs() const { return last_sample_us_; }

private:
  uint64_t maximum_gap_us_{};
  uint64_t last_sample_us_{};
  double previous_rate_rad_s_{};
  double deviation_rad_{};
  bool started_{};
  bool valid_{};
};

class FlightControlSession {
public:
  FlightControlSession(uint64_t gyro_maximum_gap_us,
                       double permanent_stop_airspeed_mps);

  void reset();
  [[nodiscard]] bool evaluateEligibility(bool required_inputs_available,
                                         bool zero_hold_achieved);
  // Control reference capture後の必須input喪失は同一flight内での
  // 再entryを禁止する。entry前の状態には適用しない。
  void enforcePostEntryHealth(bool required_inputs_available);
  void observeAirspeed(bool valid, double airspeed_mps);
  [[nodiscard]] bool startReference(uint64_t timestamp_us,
                                    double roll_rate_rad_s);
  [[nodiscard]] bool observeGyro(uint64_t timestamp_us, bool sample_valid,
                                 double roll_rate_rad_s);
  void disablePermanently();

  [[nodiscard]] bool gateEvaluated() const { return gate_evaluated_; }
  [[nodiscard]] bool permanentlyDisabled() const {
    return permanently_disabled_;
  }
  [[nodiscard]] bool referenceStarted() const { return estimator_.started(); }
  [[nodiscard]] bool estimatorValid() const { return estimator_.valid(); }
  [[nodiscard]] double rollDeviationRad() const {
    return estimator_.deviationRad();
  }

private:
  RollEstimator estimator_;
  double permanent_stop_airspeed_mps_{};
  bool gate_evaluated_{};
  bool permanently_disabled_{};
};

} // namespace control
