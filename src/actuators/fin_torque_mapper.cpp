#include "actuators/fin_torque_mapper.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace actuators {
namespace {

constexpr double kTwoPi = 6.28318530717958647692;

bool validConfig(const FinTorqueMapperConfig &config) {
  return std::isfinite(config.motor_resistance_ohm) &&
         config.motor_resistance_ohm > 0.0 &&
         std::isfinite(config.motor_torque_constant_nm_per_a) &&
         config.motor_torque_constant_nm_per_a > 0.0 &&
         std::isfinite(config.motor_speed_constant_rpm_per_v) &&
         config.motor_speed_constant_rpm_per_v > 0.0 &&
         std::isfinite(config.total_gear_ratio) &&
         config.total_gear_ratio > 0.0 &&
         std::isfinite(config.drivetrain_efficiency) &&
         config.drivetrain_efficiency > 0.0 &&
         config.drivetrain_efficiency <= 1.0 &&
         std::isfinite(config.motor_bus_voltage_v) &&
         config.motor_bus_voltage_v > 0.0 &&
         std::isfinite(config.motor_max_current_a) &&
         config.motor_max_current_a > 0.0 &&
         std::isfinite(config.motor_hard_speed_rpm) &&
         config.motor_hard_speed_rpm > 0.0 &&
         std::isfinite(config.gearbox_continuous_speed_rpm) &&
         config.gearbox_continuous_speed_rpm > 0.0 &&
         config.gearbox_continuous_speed_rpm <= config.motor_hard_speed_rpm &&
         std::isfinite(config.maximum_duty) && config.maximum_duty > 0.0 &&
         config.maximum_duty <= 1.0 &&
         std::isfinite(config.outward_angle_limit_rad) &&
         config.outward_angle_limit_rad > 0.0 &&
         config.command_full_scale > 0U &&
         config.minimum_active_command <= config.command_full_scale;
}

bool differs(double left, double right) {
  const double scale = std::max({1.0, std::abs(left), std::abs(right)});
  return std::abs(left - right) >
         8.0 * std::numeric_limits<double>::epsilon() * scale;
}

bool exceedsMagnitude(double value, double limit) {
  const double tolerance =
      16.0 * std::numeric_limits<double>::epsilon() *
      std::max({1.0, std::abs(value), std::abs(limit)});
  return std::abs(value) > limit + tolerance;
}

void inhibitDrive(FinTorqueMapperOutput &output) {
  output.effective_torque_nm = 0.0;
  output.duty_signed = 0.0;
  output.command_magnitude = 0U;
  output.minimum_command_applied = false;
  output.coast_required = true;
}

} // namespace

uint32_t finCommandToPwmCount(uint16_t command_magnitude,
                              uint16_t command_full_scale,
                              uint32_t maximum_duty_count) {
  if (command_full_scale == 0U)
    return 0U;
  const uint32_t clamped =
      std::min<uint32_t>(command_magnitude, command_full_scale);
  return static_cast<uint32_t>(
      (static_cast<uint64_t>(clamped) * maximum_duty_count) /
      command_full_scale);
}

FinTorqueMapperOutput
mapFinOutputTorque(const FinTorqueMapperInput &input,
                   const FinTorqueMapperConfig &config) {
  FinTorqueMapperOutput output{};
  output.requested_torque_nm = input.requested_torque_nm;
  if (!validConfig(config) || !std::isfinite(input.requested_torque_nm) ||
      !std::isfinite(input.angle_rad) || !std::isfinite(input.rate_rad_s))
    return output;

  const double torque = input.requested_torque_nm;
  output.motor_speed_rpm =
      input.rate_rad_s * config.total_gear_ratio * 60.0 / kTwoPi;
  output.gearbox_speed_exceeded =
      std::abs(output.motor_speed_rpm) > config.gearbox_continuous_speed_rpm;

  // 9800 rpm以上では、回転をさらに加速するtorqueだけを禁止する。
  // requested torqueで判定し、逆向きbrakingは許可する。
  output.outward_inhibited =
      (input.angle_rad >= config.outward_angle_limit_rad && torque > 0.0) ||
      (input.angle_rad <= -config.outward_angle_limit_rad && torque < 0.0);
  output.motor_speed_inhibited =
      std::abs(output.motor_speed_rpm) >= config.motor_hard_speed_rpm &&
      torque * input.rate_rad_s > 0.0;
  if (output.outward_inhibited || output.motor_speed_inhibited) {
    inhibitDrive(output);
    output.valid = true;
    return output;
  }

  const bool requested_motoring_or_stall =
      torque * input.rate_rad_s >= 0.0;
  const double requested_output_torque_per_motor_amp =
      config.motor_torque_constant_nm_per_a * config.total_gear_ratio *
      (requested_motoring_or_stall ? config.drivetrain_efficiency
                                   : 1.0 / config.drivetrain_efficiency);
  const double requested_current =
      torque / requested_output_torque_per_motor_amp;
  const double back_emf_v =
      output.motor_speed_rpm / config.motor_speed_constant_rpm_per_v;
  const double requested_voltage_v =
      requested_current * config.motor_resistance_ohm + back_emf_v;
  const double requested_voltage_duty =
      requested_voltage_v / config.motor_bus_voltage_v;
  const double voltage_limited_duty =
      std::clamp(requested_voltage_duty, -config.maximum_duty,
                 config.maximum_duty);
  output.duty_limited =
      differs(requested_voltage_duty, voltage_limited_duty);
  output.raw_duty_signed = config.positive_torque_uses_in1
                               ? voltage_limited_duty
                               : -voltage_limited_duty;

  // Spica固定版と同じく、まずbus voltageで実現されるcurrentを求め、次に
  // current制約へclampして必要terminal voltageを再構成する。
  const double voltage_limited_current =
      (voltage_limited_duty * config.motor_bus_voltage_v - back_emf_v) /
      config.motor_resistance_ohm;
  const double base_current_command =
      std::clamp(voltage_limited_current, -config.motor_max_current_a,
                 config.motor_max_current_a);
  output.current_limited =
      differs(voltage_limited_current, base_current_command);
  const double base_current_limited_voltage_v =
      back_emf_v + base_current_command * config.motor_resistance_ohm;
  const double base_duty = std::clamp(
      base_current_limited_voltage_v / config.motor_bus_voltage_v,
      -config.maximum_duty, config.maximum_duty);

  // 電源電圧内へclampした最終候補からcurrentを計算する。この値をさらに
  // 数値上clampして「制約成立」と見せない。電圧範囲内でcurrent制約を実現
  // できない場合はdriveを0へ落とす。
  const double base_electrical_current =
      (base_duty * config.motor_bus_voltage_v - back_emf_v) /
      config.motor_resistance_ohm;
  if (exceedsMagnitude(base_electrical_current,
                       config.motor_max_current_a)) {
    output.estimated_motor_current_a = base_electrical_current;
    output.current_limited = true;
    output.current_limit_unrealizable = true;
    inhibitDrive(output);
    output.valid = std::isfinite(base_electrical_current);
    return output;
  }

  double applied_voltage_duty = base_duty;
  const double raw_command =
      std::abs(base_duty) * config.command_full_scale;
  if (input.motion_requested && torque != 0.0 && raw_command > 0.0 &&
      raw_command < config.minimum_active_command) {
    applied_voltage_duty =
        std::copysign(static_cast<double>(config.minimum_active_command) /
                          config.command_full_scale,
                      base_duty);
    output.minimum_command_applied = true;
  }

  // minimum commandによる増分にもcurrent制約を再適用する。必要なdutyが
  // 電源範囲内なら70未満へ戻してcurrent制約を優先する。
  double post_compensation_current =
      (applied_voltage_duty * config.motor_bus_voltage_v - back_emf_v) /
      config.motor_resistance_ohm;
  if (output.minimum_command_applied && torque != 0.0 &&
      post_compensation_current * torque < 0.0) {
    // back-EMFでterminal voltage符号とtorque符号が異なる領域では、total
    // dutyを70へ増幅するとtorque方向が逆転し得る。その場合は補償前へ戻す。
    applied_voltage_duty = base_duty;
    post_compensation_current = base_electrical_current;
    output.minimum_command_applied = false;
    output.minimum_command_rejected_torque_direction = true;
  }
  if (exceedsMagnitude(post_compensation_current,
                       config.motor_max_current_a)) {
    output.current_limited = true;
    const double post_compensation_limited_current =
        std::clamp(post_compensation_current, -config.motor_max_current_a,
                   config.motor_max_current_a);
    applied_voltage_duty =
        (back_emf_v + post_compensation_limited_current *
                          config.motor_resistance_ohm) /
        config.motor_bus_voltage_v;
  }

  const double limited_voltage_duty =
      std::clamp(applied_voltage_duty, -config.maximum_duty,
                 config.maximum_duty);
  output.duty_limited =
      output.duty_limited ||
      differs(applied_voltage_duty, limited_voltage_duty);

  const double electrical_current =
      (limited_voltage_duty * config.motor_bus_voltage_v - back_emf_v) /
      config.motor_resistance_ohm;
  output.estimated_motor_current_a = electrical_current;
  if (exceedsMagnitude(electrical_current, config.motor_max_current_a)) {
    output.current_limited = true;
    output.current_limit_unrealizable = true;
    inhibitDrive(output);
    output.valid = std::isfinite(electrical_current);
    return output;
  }
  if (torque != 0.0 && electrical_current * torque < 0.0) {
    // bus/back-EMF制約後にもtorque方向が反転する候補は駆動しない。
    output.torque_direction_unrealizable = true;
    inhibitDrive(output);
    output.valid = true;
    return output;
  }

  output.duty_signed = config.positive_torque_uses_in1
                           ? limited_voltage_duty
                           : -limited_voltage_duty;
  output.command_magnitude = static_cast<uint16_t>(std::lround(
      std::abs(limited_voltage_duty) * config.command_full_scale));

  if (output.minimum_command_applied) {
    const double minimum_duty =
        static_cast<double>(config.minimum_active_command) /
        config.command_full_scale;
    if (std::abs(limited_voltage_duty) +
            16.0 * std::numeric_limits<double>::epsilon() <
        minimum_duty) {
      output.minimum_command_applied = false;
      output.minimum_command_limited_by_current = true;
    }
  }

  // duty/current制約後にcurrent符号がrequestと反転し得るため、effective側の
  // power-flow branchは最終計算currentから選び直す。
  const bool effective_motoring_or_stall =
      electrical_current * input.rate_rad_s >= 0.0;
  const double effective_output_torque_per_motor_amp =
      config.motor_torque_constant_nm_per_a * config.total_gear_ratio *
      (effective_motoring_or_stall ? config.drivetrain_efficiency
                                   : 1.0 / config.drivetrain_efficiency);
  output.effective_torque_nm =
      electrical_current * effective_output_torque_per_motor_amp;
  output.valid = std::isfinite(output.effective_torque_nm) &&
                 std::isfinite(output.duty_signed) &&
                 std::isfinite(output.estimated_motor_current_a);
  return output;
}

} // namespace actuators
