#include <cassert>
#include <cmath>
#include <limits>

#include "actuators/fin_kinematics.hpp"
#include "config/flight.hpp"
#include "control/roll_control.hpp"
#include "control/zero_hold.hpp"
#include "mission/flight_detectors.hpp"
#include "mission/state_machine.hpp"
#include "protocol/command_cache.hpp"
#include "protocol/wire.hpp"
#include "sensors/airspeed.hpp"

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
}

int main() {
  mission::StateMachine state;
  assert(state.snapshot().phase == mission::Phase::command_receive);
  assert(state.startSequence());
  assert(state.snapshot().phase == mission::Phase::liftoff_detection);
  assert(state.reportLiftoff(5'000'000));
  auto flight = state.snapshot();
  assert(flight.phase == mission::Phase::flight);
  assert(flight.liftoff_us == 4'000'000);
  const uint32_t first_generation = flight.generation;
  assert(first_generation != 0);
  assert(state.liftoffEmergencyRollback());
  assert(state.snapshot().phase == mission::Phase::liftoff_detection);
  assert(!state.requestDescent(first_generation));
  assert(state.reportLiftoff(8'000'000));
  flight = state.snapshot();
  assert(flight.generation != first_generation);
  assert(state.requestDescent(flight.generation));
  assert(state.snapshot().phase == mission::Phase::descent);
  assert(!state.liftoffEmergencyRollback());
  state.latchPowerCutoff();
  assert(state.snapshot().power_cutoff);

  mission::ImuLiftoffDetector imu;
  bool detected = false;
  for (int i = 0; i < 80; ++i)
    detected = imu.update(0.0, 0.0, 2.1, true) || detected;
  assert(detected);

  mission::PressureApexDetector apex;
  bool apex_detected = false;
  for (int i = 0; i < 40; ++i)
    apex_detected =
        apex.update(1000.0 + 0.01 * i, true, 11'000'000) || apex_detected;
  assert(apex_detected);

  protocol::CommandCache cache;
  protocol::GenericCommandRequest request{};
  request.transaction_id = 1;
  request.command = static_cast<uint8_t>(protocol::CommandCode::para_open);
  assert(cache.lookup(request).kind == protocol::CommandCache::Lookup::miss);
  cache.rememberAccepted(request);
  assert(cache.lookup(request).kind == protocol::CommandCache::Lookup::replay);
  protocol::CommandResult final{1, request.command,
                                protocol::CommandPhase::completed,
                                protocol::CommandReason::none, 0};
  cache.finish(final);
  assert(cache.lookup(request).result.phase == protocol::CommandPhase::completed);
  auto conflict = request;
  conflict.command = static_cast<uint8_t>(protocol::CommandCode::para_close);
  assert(cache.lookup(conflict).kind == protocol::CommandCache::Lookup::conflict);

  // CommandReceiveのFin command codeを固定する。
  assert(static_cast<uint8_t>(protocol::CommandCode::fin_free) == 0x10);
  assert(static_cast<uint8_t>(protocol::CommandCode::fin_zero) == 0x11);
  assert(static_cast<uint8_t>(protocol::CommandCode::fin_hold) == 0x13);

  protocol::CanFrame frame{};
  frame.identifier =
      static_cast<uint16_t>(protocol::CanId::generic_command_request);
  frame.data_length = 8;
  frame.data[0] = 7;
  frame.data[1] = static_cast<uint8_t>(protocol::CommandCode::fin_hold);
  protocol::GenericCommandRequest decoded{};
  assert(protocol::decodeGenericCommand(frame, decoded));
  assert(decoded.transaction_id == 7);
  assert(decoded.command == frame.data[1]);

  // AS5047Dの0/360度境界を跨いでも連続差分を維持する。
  const double wrap_forward = actuators::fin_kinematics::unwrapEncoderDelta(
      10.0 * kDegToRad, 350.0 * kDegToRad);
  const double wrap_reverse = actuators::fin_kinematics::unwrapEncoderDelta(
      350.0 * kDegToRad, 10.0 * kDegToRad);
  assert(std::abs(wrap_forward - 20.0 * kDegToRad) < 1.0e-12);
  assert(std::abs(wrap_reverse + 20.0 * kDegToRad) < 1.0e-12);

  // 1回転角だけを観測しても、隣接sampleの差分からmulti-turnを積算できる。
  double previous_raw = 350.0 * kDegToRad;
  double unwrapped = previous_raw;
  for (int i = 0; i < 6; ++i) {
    const double next_raw =
        std::fmod(previous_raw + 120.0 * kDegToRad, 2.0 * kPi);
    unwrapped += actuators::fin_kinematics::unwrapEncoderDelta(next_raw,
                                                               previous_raw);
    previous_raw = next_raw;
  }
  const double accumulated = unwrapped - 350.0 * kDegToRad;
  assert(std::abs(accumulated - 4.0 * kPi) < 1.0e-12);

  // Encoder軸が何周しても、gear ratioで出力軸Fin角へ変換する。
  const double two_encoder_turns = 4.0 * kPi;
  const double fin_angle = actuators::fin_kinematics::encoderToFinRadians(
      two_encoder_turns, flight_config::kTotalGearRatio);
  assert(std::abs(fin_angle -
                  two_encoder_turns / flight_config::kTotalGearRatio) <
         1.0e-15);

  // +8秒gateはICMまたはFin zeroが欠ければそのflightで永久停止する。
  control::FlightControlSession gate_ok{
      flight_config::kGyroIntegrationMaximumGapUs,
      flight_config::kAirspeedPermanentStopMps};
  assert(gate_ok.evaluateEligibility(true, true));
  assert(!gate_ok.permanentlyDisabled());

  control::FlightControlSession gate_imu_ng{
      flight_config::kGyroIntegrationMaximumGapUs,
      flight_config::kAirspeedPermanentStopMps};
  assert(!gate_imu_ng.evaluateEligibility(false, true));
  assert(gate_imu_ng.permanentlyDisabled());

  control::FlightControlSession gate_zero_ng{
      flight_config::kGyroIntegrationMaximumGapUs,
      flight_config::kAirspeedPermanentStopMps};
  assert(!gate_zero_ng.evaluateEligibility(true, false));
  assert(gate_zero_ng.permanentlyDisabled());

  // unavailableは永久停止しないが、valid <= 60 m/sは永久停止する。
  gate_ok.observeAirspeed(false, 0.0);
  assert(!gate_ok.permanentlyDisabled());
  gate_ok.observeAirspeed(true, 60.1);
  assert(!gate_ok.permanentlyDisabled());
  gate_ok.observeAirspeed(true, 60.0);
  assert(gate_ok.permanentlyDisabled());

  // Control開始時点をroll偏差0として、gyroを台形積分する。
  control::FlightControlSession integration{
      flight_config::kGyroIntegrationMaximumGapUs,
      flight_config::kAirspeedPermanentStopMps};
  assert(integration.evaluateEligibility(true, true));
  const double ten_dps = 10.0 * kPi / 180.0;
  assert(integration.startReference(1'000'000, ten_dps));
  assert(std::abs(integration.rollDeviationRad()) < 1.0e-12);
  for (uint64_t time = 1'001'000; time <= 1'100'000; time += 1'000)
    assert(integration.observeGyro(time, true, ten_dps));
  assert(std::abs(integration.rollDeviationRad() -
                  1.0 * kPi / 180.0) <
         1.0e-9);

  // 短い欠落は復帰可能、上限を超えたgapは永久停止。
  control::FlightControlSession short_gap{
      flight_config::kGyroIntegrationMaximumGapUs,
      flight_config::kAirspeedPermanentStopMps};
  assert(short_gap.evaluateEligibility(true, true));
  assert(short_gap.startReference(2'000'000, 0.0));
  assert(short_gap.observeGyro(2'003'000, false, 0.0));
  assert(short_gap.observeGyro(2'004'000, true, 0.0));
  assert(!short_gap.permanentlyDisabled());
  assert(!short_gap.observeGyro(2'010'000, true, 0.0));
  assert(short_gap.permanentlyDisabled());

  // gain scheduleはstate feedbackを出力し、torque limitを守る。
  control::RollController controller{flight_config::kRollGainSchedule,
                                     flight_config::kRollControlTorqueLimitNm};
  const auto control_output =
      controller.compute({1.0, 0.0, 0.0, 0.0, 100.0});
  assert(control_output.valid);
  assert(std::abs(control_output.torque_nm + 0.08) < 1.0e-12);

  // 本番ZeroHold定数とSpicaのcontinuous rate dead-zoneを固定する。
  assert(std::abs(flight_config::kFinZeroHoldKpNmPerRad - 65.390941574) <
         1.0e-12);
  assert(std::abs(flight_config::kFinZeroHoldKdNmPerRadS - 3.269547079) <
         1.0e-12);
  assert(std::abs(flight_config::kFinZeroHoldTorqueLimitNm - 0.80) < 1.0e-12);
  assert(std::abs(flight_config::kFinZeroHoldRateDeadZoneDegS - 1.0) <
         1.0e-12);
  assert(std::abs(flight_config::kMotorMaxCurrentA - 2.2) < 1.0e-12);

  const control::ZeroHoldConfig zero_hold_config{
      flight_config::kFinZeroHoldKpNmPerRad,
      flight_config::kFinZeroHoldKdNmPerRadS,
      flight_config::kFinZeroHoldTorqueLimitNm,
      0.0,
      flight_config::kFinZeroHoldRateDeadZoneDegS * kDegToRad};

  const auto below_rate_dead_zone = control::computeZeroHold(
      {0.1 * kDegToRad, 0.5 * kDegToRad}, zero_hold_config);
  const double angle_only_expected =
      -flight_config::kFinZeroHoldKpNmPerRad * 0.1 * kDegToRad;
  assert(below_rate_dead_zone.valid);
  assert(!below_rate_dead_zone.saturated);
  assert(std::abs(below_rate_dead_zone.raw_torque_nm - angle_only_expected) <
         1.0e-12);

  const auto above_rate_dead_zone = control::computeZeroHold(
      {0.1 * kDegToRad, 2.0 * kDegToRad}, zero_hold_config);
  const double continuous_dead_zone_expected =
      angle_only_expected - flight_config::kFinZeroHoldKdNmPerRadS *
                                1.0 * kDegToRad;
  assert(above_rate_dead_zone.valid);
  assert(!above_rate_dead_zone.saturated);
  assert(std::abs(above_rate_dead_zone.raw_torque_nm -
                  continuous_dead_zone_expected) <
         1.0e-12);

  const auto saturated_zero_hold =
      control::computeZeroHold({5.0 * kDegToRad, 0.0}, zero_hold_config);
  assert(saturated_zero_hold.valid);
  assert(saturated_zero_hold.saturated);
  assert(std::abs(saturated_zero_hold.torque_nm + 0.80) < 1.0e-12);

  const auto invalid_zero_hold = control::computeZeroHold(
      {std::numeric_limits<double>::quiet_NaN(), 0.0}, zero_hold_config);
  assert(!invalid_zero_hold.valid);

  // SSC差圧filterはcompile-time zeroを引き、負圧許容範囲内を0へclipする。
  sensors::DifferentialPressureFilter dp_filter{10.0, 5.0, 4};
  double filtered = 0.0;
  assert(dp_filter.update(9.0, filtered));
  assert(filtered == 0.0);
  assert(!dp_filter.update(0.0, filtered));

  const auto zero_speed =
      sensors::computeSaintVenantAirspeed(100000.0, 0.0, 20.0, 0.92);
  assert(zero_speed.valid && zero_speed.airspeed_mps == 0.0);
  const auto positive_speed =
      sensors::computeSaintVenantAirspeed(100000.0, 1000.0, 20.0, 0.92);
  assert(positive_speed.valid && positive_speed.airspeed_mps > 0.0);

  // wire互換: Control packetとairspeed packetのID/lengthを固定する。
  protocol::ControlTelemetry control_message{};
  control_message.requested_torque_raw =
      protocol::encodeRequestedTorque(0.5, true);
  const auto control_frame = protocol::encode(control_message);
  assert(control_frame.identifier ==
         static_cast<uint16_t>(protocol::CanId::control_telemetry));
  assert(control_frame.data_length == 4);
  const auto airspeed_frame = protocol::encode(protocol::AirspeedTelemetry{
      1, protocol::encodeAirspeed(123.0, true)});
  assert(airspeed_frame.identifier ==
         static_cast<uint16_t>(protocol::CanId::airspeed_telemetry));
  assert(airspeed_frame.data_length == 2);

  return 0;
}
