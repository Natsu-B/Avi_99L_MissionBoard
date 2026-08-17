#pragma once

#include <cstdint>

namespace actuators {

struct FinTorqueMapperConfig {
  double motor_resistance_ohm{};
  double motor_torque_constant_nm_per_a{};
  double motor_speed_constant_rpm_per_v{};
  double total_gear_ratio{};
  double drivetrain_efficiency{};
  double motor_bus_voltage_v{};
  double motor_max_current_a{};
  double motor_hard_speed_rpm{};
  double gearbox_continuous_speed_rpm{};
  double maximum_duty{};
  double outward_angle_limit_rad{};
  uint16_t command_full_scale{};
  uint16_t minimum_active_command{};
  bool positive_torque_uses_in1{};
};

struct FinTorqueMapperInput {
  // TorqueMapper基準のrequested/effective出力軸torque座標。実測torqueではない。
  double requested_torque_nm{};
  double angle_rad{};
  double rate_rad_s{};
  bool motion_requested{};
};

struct FinTorqueMapperOutput {
  double requested_torque_nm{};
  double effective_torque_nm{};
  // 駆動候補の最終dutyから計算した値。実測currentではない。
  // current_limit_unrealizable時はdriveを0へ落とす直前の候補値を保持する。
  double estimated_motor_current_a{};
  double motor_speed_rpm{};
  double raw_duty_signed{};
  double duty_signed{};
  uint16_t command_magnitude{};
  bool minimum_command_applied{};
  bool minimum_command_limited_by_current{};
  bool minimum_command_rejected_torque_direction{};
  bool current_limited{};
  bool current_limit_unrealizable{};
  bool torque_direction_unrealizable{};
  bool duty_limited{};
  bool outward_inhibited{};
  bool motor_speed_inhibited{};
  bool gearbox_speed_exceeded{};
  // safety/unrealizableによる0はRoll drive/brakeの0 commandではなく
  // 両入力LOWのdrive/coastを必要とする。
  bool coast_required{};
  bool valid{};

  [[nodiscard]] bool antiWindupRequired() const {
    return current_limited || current_limit_unrealizable ||
           torque_direction_unrealizable || duty_limited ||
           outward_inhibited || motor_speed_inhibited ||
           gearbox_speed_exceeded;
  }
};

// requested torqueを共通の電気modelでPWMへ写像する。actual current/torqueの
// 計測値ではなく、設定値から計算したeffective座標を返す。
[[nodiscard]] FinTorqueMapperOutput
mapFinOutputTorque(const FinTorqueMapperInput &input,
                   const FinTorqueMapperConfig &config);

// FIN0003/FIN0004のMotorDriverと同じ整数floorで10 bit commandを
// LEDC countへ変換する。full scale 1024はcount 1023に対応する。
[[nodiscard]] uint32_t finCommandToPwmCount(uint16_t command_magnitude,
                                            uint16_t command_full_scale,
                                            uint32_t maximum_duty_count);

} // namespace actuators
