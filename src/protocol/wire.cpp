#include "protocol/wire.hpp"

#include <algorithm>
#include <cmath>

namespace protocol {
namespace {

void putU16(std::array<uint8_t, 8> &data, std::size_t offset, uint16_t value) {
  data[offset] = static_cast<uint8_t>(value);
  data[offset + 1] = static_cast<uint8_t>(value >> 8U);
}
void putU32(std::array<uint8_t, 8> &data, std::size_t offset, uint32_t value) {
  data[offset] = static_cast<uint8_t>(value);
  data[offset + 1] = static_cast<uint8_t>(value >> 8U);
  data[offset + 2] = static_cast<uint8_t>(value >> 16U);
  data[offset + 3] = static_cast<uint8_t>(value >> 24U);
}
CanFrame makeFrame(CanId id, uint8_t length) {
  CanFrame frame{};
  frame.identifier = static_cast<uint16_t>(id);
  frame.data_length = length;
  return frame;
}
uint16_t signed16(double value, double resolution, uint16_t invalid) {
  if (!std::isfinite(value))
    return invalid;
  const long count = std::lround(value / resolution);
  if (count < -32752L || count > 32767L)
    return invalid;
  return static_cast<uint16_t>(static_cast<int16_t>(count));
}

} // namespace

bool decodeGenericCommand(const CanFrame &frame,
                          GenericCommandRequest &request) {
  if (frame.extended || frame.remote ||
      frame.identifier != static_cast<uint16_t>(CanId::generic_command_request) ||
      frame.data_length != 8)
    return false;
  request.transaction_id = frame.data[0];
  request.command = frame.data[1];
  std::copy_n(frame.data.begin() + 2, request.arguments.size(),
              request.arguments.begin());
  return true;
}
bool decodeEmergency(const CanFrame &frame, uint8_t &transaction_id) {
  if (frame.extended || frame.remote ||
      frame.identifier !=
          static_cast<uint16_t>(CanId::liftoff_detection_emergency_stop) ||
      frame.data_length != 1)
    return false;
  transaction_id = frame.data[0];
  return true;
}
CanFrame encode(const CommandResult &result) {
  auto frame = makeFrame(CanId::command_result, 8);
  frame.data[0] = result.transaction_id;
  frame.data[1] = result.command;
  frame.data[2] = static_cast<uint8_t>(result.phase);
  frame.data[3] = static_cast<uint8_t>(result.reason);
  putU32(frame.data, 4, result.detail);
  return frame;
}
CanFrame encode(const MissionStatus &status) {
  auto frame = makeFrame(CanId::mission_status_telemetry, 8);
  frame.data[0] = status.sequence;
  frame.data[1] = static_cast<uint8_t>(status.state);
  putU16(frame.data, 2, status.flight_status);
  frame.data[4] = status.config_flags;
  frame.data[5] = static_cast<uint8_t>(status.fin_mode);
  frame.data[6] = static_cast<uint8_t>(status.para_mode);
  frame.data[7] = status.parachute_angle_raw;
  return frame;
}
CanFrame encode(const Kinematics &kinematics) {
  auto frame = makeFrame(CanId::kinematics_telemetry, 8);
  frame.data[0] = kinematics.sequence;
  putU16(frame.data, 1, kinematics.roll_raw);
  putU16(frame.data, 3, kinematics.roll_rate_raw);
  frame.data[5] = kinematics.fin_angle_raw;
  putU16(frame.data, 6, kinematics.fin_rate_raw);
  return frame;
}
CanFrame encode(const ControlTelemetry &telemetry) {
  auto frame = makeFrame(CanId::control_telemetry, 4);
  frame.data[0] = telemetry.sequence;
  putU16(frame.data, 1,
         static_cast<uint16_t>(telemetry.requested_torque_raw & 0x0FFFU));
  frame.data[3] = telemetry.flight_elapsed_raw;
  return frame;
}
CanFrame encode(const LpsTelemetry &telemetry) {
  auto frame = makeFrame(CanId::lps_telemetry, 4);
  frame.data[0] = telemetry.sequence;
  putU16(frame.data, 1, static_cast<uint16_t>(telemetry.pressure_raw & 0x07FFU));
  frame.data[3] = telemetry.temperature_raw;
  return frame;
}
CanFrame encode(const AirspeedTelemetry &telemetry) {
  auto frame = makeFrame(CanId::airspeed_telemetry, 2);
  frame.data[0] = telemetry.sequence;
  frame.data[1] = telemetry.airspeed_raw;
  return frame;
}
CanFrame encode(const ControlRollTelemetryV2 &telemetry) {
  auto frame = makeFrame(CanId::control_roll_telemetry_v2, 8);
  frame.data[0] = telemetry.sequence;
  frame.data[1] = ControlRollTelemetryV2::schema_version;
  putU16(frame.data, 2, telemetry.control_roll_reference_unwrapped_raw);
  putU16(frame.data, 4, telemetry.roll_deviation_unwrapped_raw);
  frame.data[6] = static_cast<uint8_t>(telemetry.flags & 0x1FU);
  frame.data[7] = telemetry.reference_capture_event_sequence;
  return frame;
}
CanFrame encode(const DeviceHealthTelemetry &telemetry) {
  auto frame = makeFrame(CanId::device_health_telemetry, 6);
  frame.data[0] = telemetry.sequence;
  frame.data[1] = static_cast<uint8_t>(telemetry.icm42688);
  frame.data[2] = static_cast<uint8_t>(telemetry.as5047d);
  frame.data[3] = static_cast<uint8_t>(telemetry.lps25hb);
  frame.data[4] = static_cast<uint8_t>(telemetry.ssc);
  frame.data[5] = static_cast<uint8_t>(telemetry.mission_sd);
  return frame;
}
uint8_t encodeFinAngle(double degrees, bool valid) {
  if (!valid || !std::isfinite(degrees))
    return 241;
  if (degrees < -15.0 || degrees > 15.0)
    return 254;
  return static_cast<uint8_t>(std::lround((degrees + 15.0) / 0.125));
}
uint16_t encodeFinRate(double value, bool valid) {
  return valid ? signed16(value, 0.02, 0x8008U) : 0x8000U;
}
uint16_t encodeRoll(double value, bool valid) {
  return valid ? signed16(value, 0.5, 0x800AU) : 0x8000U;
}
uint16_t encodeRollRate(double value, bool valid) {
  return valid ? signed16(value, 0.1, 0x800AU) : 0x8000U;
}
uint16_t encodeRequestedTorque(double value, bool valid) {
  if (!valid || !std::isfinite(value))
    return 0x0800U;
  const long count = std::lround(value / 0.002);
  if (count < -2032L || count > 2047L)
    return 0x0804U;
  return static_cast<uint16_t>(static_cast<int16_t>(count)) & 0x0FFFU;
}
uint8_t encodeFlightElapsed(double seconds, bool valid) {
  if (!valid || !std::isfinite(seconds) || seconds < 0.0)
    return 0xF1U;
  if (seconds >= 24.0)
    return 0xFBU;
  return static_cast<uint8_t>(std::floor(seconds * 10.0));
}
uint16_t encodeLpsPressure(double value, bool valid) {
  if (!valid || !std::isfinite(value))
    return 2047;
  if (value < 800.0)
    return 2041;
  if (value > 1206.2)
    return 2042;
  return static_cast<uint16_t>(std::lround((value - 800.0) / 0.2));
}
uint8_t encodeLpsTemperature(double value, bool valid) {
  if (!valid || !std::isfinite(value))
    return 255;
  if (value < -50.0)
    return 250;
  if (value > 150.0)
    return 251;
  return static_cast<uint8_t>(std::lround(value + 50.0));
}
uint8_t encodeAirspeed(double value, bool valid) {
  if (!valid || !std::isfinite(value))
    return 249;
  if (value < 0.0)
    return 246;
  if (value > 245.0)
    return 247;
  return static_cast<uint8_t>(std::lround(value));
}
uint8_t encodeParachuteAngle(double degrees, bool valid) {
  if (!valid || !std::isfinite(degrees))
    return 255;
  double normalized = std::fmod(degrees, 360.0);
  if (normalized < 0.0)
    normalized += 360.0;
  return static_cast<uint8_t>(std::lround(normalized / 1.5));
}

} // namespace protocol
