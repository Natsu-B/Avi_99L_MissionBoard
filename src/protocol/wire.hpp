#pragma once

#include <array>
#include <cstdint>

namespace protocol {

enum class CanId : uint16_t {
  liftoff_detection_emergency_stop = 0x002,
  generic_command_request = 0x010,
  command_result = 0x011,
  mission_event = 0x020,
  kinematics_telemetry = 0x100,
  control_telemetry = 0x101,
  mission_status_telemetry = 0x102,
  power_time_telemetry = 0x103,
  descent_core_telemetry = 0x104,
  attitude_tilt_telemetry = 0x107,
  lps_telemetry = 0x108,
  airspeed_telemetry = 0x109,
  control_roll_telemetry_v2 = 0x10A,
};

enum class WireMissionState : uint8_t {
  command_receive = 0,
  liftoff_detection = 1,
  engine_burn = 2,
  control = 3,
  descent = 4,
};

enum class CommandCode : uint8_t {
  start_sequence = 0x01,
  fin_free = 0x10,
  fin_hold_current = 0x13,
  para_open = 0x25,
  para_close = 0x26,
  liftoff_emergency_result = 0xF1,
};

enum class CommandPhase : uint8_t {
  accepted = 0,
  completed = 1,
  rejected = 2,
  failed = 3
};
enum class CommandReason : uint8_t {
  none = 0,
  busy = 1,
  invalid_state = 2,
  invalid_argument = 3,
  not_configured = 4,
  device_unavailable = 5,
  timeout = 6,
  stall = 7,
  protocol_error = 8,
  interrupted_by_emergency = 9,
  persistence_error = 10,
  internal_error = 11,
  not_supported = 12,
  safety_interlock = 13,
  already_satisfied = 14,
};
enum class FinMode : uint8_t {
  free = 0,
  brake = 1,
  position_hold = 2,
  zero_hold = 3,
  relative_move = 4,
  roll_control = 5,
  unknown = 15
};
enum class ParaMode : uint8_t {
  free = 0,
  hold = 1,
  relative_move = 2,
  opening_or_retrying = 3,
  closing = 4,
  powered_off = 5,
  unknown = 15
};

struct GenericCommandRequest {
  uint8_t transaction_id{};
  uint8_t command{};
  std::array<uint8_t, 6> arguments{};
};
struct CommandResult {
  uint8_t transaction_id{};
  uint8_t command{};
  CommandPhase phase{};
  CommandReason reason{};
  uint32_t detail{};
};
struct MissionStatus {
  uint8_t sequence{};
  WireMissionState state{WireMissionState::command_receive};
  uint16_t flight_status{};
  uint8_t config_flags{};
  FinMode fin_mode{FinMode::free};
  ParaMode para_mode{ParaMode::powered_off};
  uint8_t parachute_angle_raw{255};
};
struct Kinematics {
  uint8_t sequence{};
  uint16_t roll_raw{0x8000};
  uint16_t roll_rate_raw{0x8000};
  uint8_t fin_angle_raw{241};
  uint16_t fin_rate_raw{0x8000};
};
struct ControlTelemetry {
  uint8_t sequence{};
  uint16_t requested_torque_raw{0x0800};
  uint8_t flight_elapsed_raw{0xF1};
};
struct LpsTelemetry {
  uint8_t sequence{};
  uint16_t pressure_raw{2047};
  uint8_t temperature_raw{255};
};
struct AirspeedTelemetry {
  uint8_t sequence{};
  uint8_t airspeed_raw{249};
};
struct ControlRollTelemetryV2 {
  static constexpr uint8_t schema_version = 2;
  static constexpr uint8_t reference_valid = 1U << 0U;
  static constexpr uint8_t reference_captured_since_previous_frame = 1U << 1U;
  static constexpr uint8_t control_active = 1U << 2U;
  static constexpr uint8_t reference_out_of_range = 1U << 3U;
  static constexpr uint8_t deviation_out_of_range = 1U << 4U;

  uint8_t sequence{};
  uint16_t control_roll_reference_unwrapped_raw{0x8000};
  uint16_t roll_deviation_unwrapped_raw{0x8000};
  uint8_t flags{};
  uint8_t reference_capture_event_sequence{};
};
struct CanFrame {
  uint32_t identifier{};
  uint8_t data_length{};
  std::array<uint8_t, 8> data{};
  bool extended{};
  bool remote{};
};

[[nodiscard]] bool decodeGenericCommand(const CanFrame &frame,
                                        GenericCommandRequest &request);
[[nodiscard]] bool decodeEmergency(const CanFrame &frame,
                                   uint8_t &transaction_id);
[[nodiscard]] CanFrame encode(const CommandResult &result);
[[nodiscard]] CanFrame encode(const MissionStatus &status);
[[nodiscard]] CanFrame encode(const Kinematics &kinematics);
[[nodiscard]] CanFrame encode(const ControlTelemetry &telemetry);
[[nodiscard]] CanFrame encode(const LpsTelemetry &telemetry);
[[nodiscard]] CanFrame encode(const AirspeedTelemetry &telemetry);
[[nodiscard]] CanFrame encode(const ControlRollTelemetryV2 &telemetry);
[[nodiscard]] uint8_t encodeFinAngle(double degrees, bool valid);
[[nodiscard]] uint16_t encodeFinRate(double degrees_per_second, bool valid);
[[nodiscard]] uint16_t encodeRoll(double degrees, bool valid);
[[nodiscard]] uint16_t encodeRollRate(double degrees_per_second, bool valid);
[[nodiscard]] uint16_t encodeRequestedTorque(double torque_nm, bool valid);
[[nodiscard]] uint8_t encodeFlightElapsed(double seconds, bool valid);
[[nodiscard]] uint16_t encodeLpsPressure(double hectopascals, bool valid);
[[nodiscard]] uint8_t encodeLpsTemperature(double degrees_celsius, bool valid);
[[nodiscard]] uint8_t encodeAirspeed(double metres_per_second, bool valid);
[[nodiscard]] uint8_t encodeParachuteAngle(double degrees, bool valid);

} // namespace protocol
