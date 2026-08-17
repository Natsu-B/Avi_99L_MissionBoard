#include <cassert>
#include <cmath>
#include <limits>
#include <string_view>

#include "actuators/fin_drive_interlock.hpp"
#include "actuators/fin_kinematics.hpp"
#include "actuators/fin_torque_mapper.hpp"
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

  // +8秒gateは必須inputまたはZeroHold成立が欠ければ永久停止する。
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

  control::FlightControlSession gate_hold_ng{
      flight_config::kGyroIntegrationMaximumGapUs,
      flight_config::kAirspeedPermanentStopMps};
  assert(!gate_hold_ng.evaluateEligibility(true, false));
  assert(gate_hold_ng.permanentlyDisabled());

  // Control entry後はattitude/Fin/LPS/SSC/airspeedのどれか1つでも
  // unavailableになれば、同一flight内の再entryをlatchして禁止する。
  constexpr std::array<control::ControlInputHealth, 5> input_dropouts{{
      {false, true, true, true, true},
      {true, false, true, true, true},
      {true, true, false, true, true},
      {true, true, true, false, true},
      {true, true, true, true, false},
  }};
  assert(control::allControlInputsAvailable({true, true, true, true, true}));
  for (const auto &dropout : input_dropouts) {
    assert(!control::allControlInputsAvailable(dropout));
    control::FlightControlSession no_reentry{
        flight_config::kGyroIntegrationMaximumGapUs,
        flight_config::kAirspeedPermanentStopMps};
    no_reentry.enforcePostEntryHealth(false);
    assert(!no_reentry.permanentlyDisabled());
    assert(no_reentry.evaluateEligibility(true, true));
    assert(no_reentry.startReference(900'000, 0.0));
    no_reentry.enforcePostEntryHealth(
        control::allControlInputsAvailable(dropout));
    assert(no_reentry.permanentlyDisabled());
    no_reentry.enforcePostEntryHealth(true);
    assert(no_reentry.permanentlyDisabled());
  }

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

  // gain scheduleはstate feedbackをrequested torqueとして出力する。
  // software authority clampは持たず、共通actuator mapperが物理制約を課す。
  constexpr std::array<control::GainPoint, 7> expected_roll_schedule{{
      {60.0,
       {14.142135623750997, 17.848362242491291, 2.7408752200006115,
        0.5733297870290188}},
      {80.0,
       {14.142135623728176, 21.122267798503998, 2.1934392351804677,
        0.6100282346806772}},
      {100.0,
       {14.142135623726388, 24.290707004734234, 1.8456407122409573,
        0.6444682755503087}},
      {120.0,
       {14.142135623726164, 27.363474239193827, 1.6035209983579835,
        0.6769427908755344}},
      {140.0,
       {14.14213562373258, 30.362532251737875, 1.423327710512041,
        0.7078278812966763}},
      {160.0,
       {14.142135623732253, 33.3812090635748, 1.276489149551201,
        0.7381654653819836}},
      {180.0,
       {14.142135623730802, 36.368817904885404, 1.1577597738460124,
        0.7675013930465681}},
  }};
  for (std::size_t point = 0; point < expected_roll_schedule.size();
       ++point) {
    assert(flight_config::kRollGainSchedule[point].airspeed_mps ==
           expected_roll_schedule[point].airspeed_mps);
    for (std::size_t state_index = 0; state_index < 4; ++state_index)
      assert(flight_config::kRollGainSchedule[point].gain[state_index] ==
             expected_roll_schedule[point].gain[state_index]);
  }
  // FIN0009/FIN0010の不成立を隠さず、ユーザ指定で固定した
  // Brake simulation候補とproduction選定状態を分離する。
  static_assert(std::string_view{flight_config::kRollGainCandidateTopology} ==
                "drive_brake");
  static_assert(std::string_view{flight_config::kRollGainFitRun} ==
                "FIN0007");
  static_assert(std::string_view{flight_config::kRollGainStrictHoldoutRun} ==
                "FIN0009");
  static_assert(std::string_view{flight_config::kRollGainConfirmationRun} ==
                "FIN0010");
  static_assert(
      std::string_view{flight_config::kRollGainSelectionStatus} ==
      "PROVISIONAL_BRAKE_FIXED_BY_USER_DIRECTION_VALIDATION_GATE_NOT_MET");
  static_assert(
      std::string_view{flight_config::kRollGainSourceArtifactSha256} ==
      "2eb8ac6f6b94fb99b22417546ef437c557b676c56545e0721e4b15c94dff25b1");
  static_assert(
      std::string_view{flight_config::kRollGainSourceConsumedMarkerSha256} ==
      "4cedffcdb8f389116bfe10c8a6126ca34e64bff2b6cc04898889b79f73fa09c2");
  static_assert(!flight_config::kRollGainStrictHoldoutPassed);
  static_assert(!flight_config::kRollGainConfirmationPassed);
  static_assert(flight_config::kRollGainValidationNoGo);
  static_assert(!flight_config::kRollGainProductionSelectable);
  control::RollController controller{flight_config::kRollGainSchedule};
  const auto control_output =
      controller.compute({1.0, 0.0, 0.0, 0.0, 100.0});
  assert(control_output.valid);
  assert(std::abs(control_output.torque_nm +
                  flight_config::kRollGainSchedule[2].gain[0]) <
         1.0e-12);
  const auto below_schedule =
      controller.compute({1.0, 0.0, 0.0, 0.0, 59.9});
  const auto above_schedule =
      controller.compute({1.0, 0.0, 0.0, 0.0, 220.0});
  assert(below_schedule.valid && above_schedule.valid);
  assert(std::abs(below_schedule.torque_nm +
                  flight_config::kRollGainSchedule.front().gain[0]) <
         1.0e-12);
  assert(std::abs(above_schedule.torque_nm +
                  flight_config::kRollGainSchedule.back().gain[0]) <
         1.0e-12);
  for (const double boundary_speed :
       std::array<double, 8>{59.9, 60.0, 60.1, 179.9, 180.0, 180.1,
                             200.0, 220.0}) {
    const auto boundary =
        controller.compute({1.0, 0.0, 0.0, 0.0, boundary_speed});
    assert(boundary.valid && std::isfinite(boundary.torque_nm));
  }
  const auto midpoint =
      controller.compute({0.0, 1.0, 0.0, 0.0, 70.0});
  const double expected_midpoint_fin_gain =
      0.5 * (expected_roll_schedule[0].gain[1] +
             expected_roll_schedule[1].gain[1]);
  assert(midpoint.valid);
  assert(std::abs(midpoint.torque_nm + expected_midpoint_fin_gain) <
         1.0e-12);

  const auto unclamped_control_output =
      controller.compute({100.0, 0.0, 0.0, 0.0, 220.0});
  assert(unclamped_control_output.valid);
  assert(std::abs(unclamped_control_output.torque_nm +
                  100.0 * flight_config::kRollGainSchedule.back().gain[0]) <
         1.0e-12);

  // FIN0003/FIN0004取得時のPIDと30 kHz用command scaleを固定する。
  assert(std::abs(flight_config::kFinZeroHoldKpNmPerRad - 65.390941574) <
         1.0e-12);
  assert(std::abs(flight_config::kFinZeroHoldKiNmPerRadTimesS - 4.577365910) <
         1.0e-12);
  assert(std::abs(flight_config::kFinZeroHoldKdNmPerRadS - 3.269547079) <
         1.0e-12);
  assert(std::abs(flight_config::kFinZeroHoldNmPerCommand -
                  0.0022825744628906255) < 1.0e-15);
  assert(std::abs(flight_config::kFinZeroHoldIntegralLimitRadTimesS -
                  0.034906585) < 1.0e-12);
  assert(std::abs(flight_config::kFinZeroHoldRateFilterTauS - 0.020) <
         1.0e-12);
  static_assert(flight_config::kFinZeroHoldSimulationGatePassed);
  static_assert(!flight_config::kFinZeroHoldProductionSelectable);
  assert(flight_config::kMotorCommandFullScale == 1024);
  assert(flight_config::kMotorMinimumActiveCommand == 70);
  assert(std::abs(flight_config::kMotorMaxCurrentA - 2.2) < 1.0e-12);
  // FIN0003 MotorDriverの整数floor量子化。70 commandは69 LEDC count、
  // full scale 1024 commandは1023 countに対応する。
  assert(actuators::finCommandToPwmCount(70, 1024, 1023) == 69);
  assert(actuators::finCommandToPwmCount(1024, 1024, 1023) == 1023);
  assert(actuators::finCommandToPwmCount(2048, 1024, 1023) == 1023);
  assert(actuators::finCommandToPwmCount(70, 0, 1023) == 0);

  // cutoff latch後は古いrealtime snapshotの非0 commandを拒否し、
  // coast用の0 commandと明示的なnew epoch resetだけを許可する。
  actuators::FinDriveInterlock drive_interlock;
  assert(drive_interlock.allows(70));
  drive_interlock.latch();
  assert(drive_interlock.inhibited());
  assert(!drive_interlock.allows(70));
  assert(!drive_interlock.allows(-70));
  assert(drive_interlock.allows(0));
  drive_interlock.noteSuccessfulRecovery();
  assert(drive_interlock.inhibited());
  drive_interlock.clearForNewEpoch();
  assert(!drive_interlock.inhibited());
  assert(drive_interlock.allows(1024));

  // boot初期化faultはrecoverable safe stateでありpower-cutoff latchではない。
  // recovery後はFinZero/ZeroHoldに必要な非0 driveを再度許可する。
  actuators::FinDriveInterlock boot_recovery_interlock;
  boot_recovery_interlock.noteRecoverableFault();
  assert(boot_recovery_interlock.inhibited());
  assert(!boot_recovery_interlock.allows(70));
  boot_recovery_interlock.noteSuccessfulRecovery();
  assert(!boot_recovery_interlock.inhibited());
  assert(boot_recovery_interlock.allows(70));
  actuators::FinDriveInterlock combined_interlock;
  combined_interlock.noteRecoverableFault();
  combined_interlock.latch();
  combined_interlock.clearForNewEpoch();
  assert(combined_interlock.inhibited());
  combined_interlock.noteSuccessfulRecovery();
  assert(combined_interlock.allows(70));

  const control::ZeroHoldConfig zero_hold_config{
      flight_config::kFinZeroHoldKpNmPerRad,
      flight_config::kFinZeroHoldKiNmPerRadTimesS,
      flight_config::kFinZeroHoldKdNmPerRadS,
      flight_config::kFinZeroHoldIntegralLimitRadTimesS,
      flight_config::kFinZeroHoldRateFilterTauS,
      flight_config::kFinZeroHoldAngleDeadbandDeg * kDegToRad,
      flight_config::kFinZeroHoldRateDeadbandDegS * kDegToRad,
      flight_config::kFinZeroHoldMinimumActiveErrorDeg * kDegToRad,
      flight_config::kFinControlMaximumDtS,
      flight_config::kFinZeroHoldIntegralDecay,
      flight_config::kFinZeroHoldIntegralZeroThresholdRadTimesS};
  control::ZeroHoldState zero_hold_state{};

  const auto first_zero_hold = control::computeZeroHold(
      {0.1 * kDegToRad, 2.0 * kDegToRad, 0.001, true}, zero_hold_config,
      zero_hold_state);
  const double error = -0.1 * kDegToRad;
  const double first_expected =
      flight_config::kFinZeroHoldKpNmPerRad * error +
      flight_config::kFinZeroHoldKiNmPerRadTimesS * error * 0.001;
  assert(first_zero_hold.valid && first_zero_hold.motion_requested);
  assert(first_zero_hold.filtered_rate_rad_s == 0.0);
  assert(std::abs(first_zero_hold.requested_torque_nm - first_expected) <
         1.0e-12);

  const auto filtered_zero_hold = control::computeZeroHold(
      {0.1 * kDegToRad, 2.0 * kDegToRad, 0.001, true}, zero_hold_config,
      zero_hold_state);
  const double filtered_rate =
      (0.001 / 0.021) * 2.0 * kDegToRad;
  assert(filtered_zero_hold.valid);
  assert(std::abs(filtered_zero_hold.filtered_rate_rad_s - filtered_rate) <
         1.0e-12);

  const double integral_before_freeze =
      zero_hold_state.integral_error_rad_times_s;
  const auto frozen_zero_hold = control::computeZeroHold(
      {1.0 * kDegToRad, 0.0, 0.001, false}, zero_hold_config,
      zero_hold_state);
  assert(frozen_zero_hold.valid && frozen_zero_hold.integral_frozen);
  assert(zero_hold_state.integral_error_rad_times_s == integral_before_freeze);

  const double integral_before_limit_tick =
      zero_hold_state.integral_error_rad_times_s;
  const auto limit_tick = control::computeZeroHold(
      {10.0 * kDegToRad, 5.0 * kDegToRad, 0.001, true}, zero_hold_config,
      zero_hold_state);
  assert(limit_tick.valid);
  const double filtered_rate_after_limit_tick =
      zero_hold_state.filtered_rate_rad_s;
  assert(zero_hold_state.integral_error_rad_times_s !=
         integral_before_limit_tick);
  control::applyZeroHoldActuatorFeedback(
      true, integral_before_limit_tick, limit_tick, zero_hold_state);
  assert(zero_hold_state.integral_error_rad_times_s ==
         integral_before_limit_tick);
  assert(zero_hold_state.filtered_rate_rad_s ==
         filtered_rate_after_limit_tick);

  control::resetZeroHold(zero_hold_state);
  zero_hold_state.integral_error_rad_times_s = 0.01;
  zero_hold_state.rate_filter_initialized = true;
  const auto deadband_zero_hold = control::computeZeroHold(
      {0.01 * kDegToRad, 0.0, 0.001, true}, zero_hold_config,
      zero_hold_state);
  assert(deadband_zero_hold.valid && deadband_zero_hold.in_hold_deadband);
  assert(deadband_zero_hold.requested_torque_nm == 0.0);
  assert(std::abs(zero_hold_state.integral_error_rad_times_s - 0.0095) <
         1.0e-12);

  const auto invalid_zero_hold = control::computeZeroHold(
      {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.001, true},
      zero_hold_config, zero_hold_state);
  assert(!invalid_zero_hold.valid);
  const auto invalid_dt_zero_hold = control::computeZeroHold(
      {0.0, 0.0, 0.0, true}, zero_hold_config, zero_hold_state);
  assert(!invalid_dt_zero_hold.valid);

  control::resetZeroHold(zero_hold_state);
  bool integral_limit_reached = false;
  for (int tick = 0; tick < 100; ++tick) {
    const auto limited_integral = control::computeZeroHold(
        {1.0, 0.0, 0.001, true}, zero_hold_config, zero_hold_state);
    assert(limited_integral.valid);
    integral_limit_reached =
        integral_limit_reached || limited_integral.integral_frozen;
  }
  assert(integral_limit_reached);
  assert(std::abs(zero_hold_state.integral_error_rad_times_s) <=
         flight_config::kFinZeroHoldIntegralLimitRadTimesS);

  control::resetZeroHold(zero_hold_state);
  const auto subthreshold_zero_hold = control::computeZeroHold(
      {0.079 * kDegToRad, 0.0, 0.001, true}, zero_hold_config,
      zero_hold_state);
  assert(subthreshold_zero_hold.valid &&
         !subthreshold_zero_hold.motion_requested);

  control::ZeroHoldAchievementState achievement{};
  const control::ZeroHoldAchievementConfig achievement_config{
      1.0 * kDegToRad, 2.0 * kDegToRad, 200'000,
      flight_config::kFinFreshUs};
  assert(!control::updateZeroHoldAchievement(
      1'000'000, true, 0.5 * kDegToRad, 1.0 * kDegToRad,
      achievement_config, achievement));
  // 1 sample後の長時間欠測を200 ms連続成立として数えない。
  assert(!control::updateZeroHoldAchievement(
      1'200'000, true, 0.5 * kDegToRad, 1.0 * kDegToRad,
      achievement_config, achievement));
  for (uint64_t timestamp = 1'201'000; timestamp < 1'400'000;
       timestamp += 1'000)
    assert(!control::updateZeroHoldAchievement(
        timestamp, true, 0.5 * kDegToRad, 1.0 * kDegToRad,
        achievement_config, achievement));
  assert(control::updateZeroHoldAchievement(
      1'400'000, true, 0.5 * kDegToRad, 1.0 * kDegToRad,
      achievement_config, achievement));
  assert(!control::updateZeroHoldAchievement(
      1'401'000, true, 1.1 * kDegToRad, 0.0, achievement_config,
      achievement));
  assert(!achievement.achieved);
  assert(!control::updateZeroHoldAchievement(
      1'400'000, true, 0.0, 0.0, achievement_config, achievement));

  const actuators::FinTorqueMapperConfig mapper_config{
      flight_config::kMotorResistanceOhm,
      flight_config::kMotorTorqueConstantNmPerA,
      flight_config::kMotorSpeedConstantRpmPerV,
      flight_config::kTotalGearRatio,
      flight_config::kDrivetrainEfficiency,
      flight_config::kMotorBusVoltageV,
      flight_config::kMotorMaxCurrentA,
      flight_config::kMotorHardSpeedRpm,
      flight_config::kGearboxContinuousSpeedRpm,
      flight_config::kMotorMaximumDuty,
      flight_config::kFinOutwardCommandLimitDeg * kDegToRad,
      flight_config::kMotorCommandFullScale,
      flight_config::kMotorMinimumActiveCommand,
      flight_config::kPositiveTorqueUsesIn1};

  // Spica Scripts/mission_99l_torque_mapper.m
  // SHA256=c93eda5f5ba3c6002085300cb55c290cfdb4b0c03c581337e2ad41dbb9a9ff2b
  // から得た代表/境界vectorと、realizable dutyを固定する。
  struct MapperGoldenCase {
    double requested_torque_nm;
    double rate_rad_s;
    double angle_rad;
    bool motion_requested;
    double duty_signed;
  };
  constexpr std::array<MapperGoldenCase, 10> mapper_golden_cases{{
      {0.001, 0.0, 0.0, true, 0.068359375},
      {-0.001, 0.0, 0.0, true, -0.068359375},
      {0.001, -0.1, 0.0, true, -0.016535933463469304},
      {10.0, -5.0, 0.0, true, 0.016168985142420784},
      {100.0, 0.0, 0.0, true, 0.85066666666666668},
      {100.0, 5.0, 0.0, true, 1.0},
      {1.0, -5.0, 15.0 * kDegToRad, true, 0.0},
      {-1.0, 5.0, 15.0 * kDegToRad, true, 0.68047751450862737},
      {-0.1, 20.0, 0.0, true, 0.0},
      {0.1, 7.0, 0.0, true, 0.0},
  }};
  for (const auto &golden : mapper_golden_cases) {
    const auto mapped = actuators::mapFinOutputTorque(
        {golden.requested_torque_nm, golden.angle_rad, golden.rate_rad_s,
         golden.motion_requested},
        mapper_config);
    assert(mapped.valid);
    assert(std::abs(mapped.duty_signed - golden.duty_signed) < 1.0e-12);
  }

  // 70 commandはrequested torque clampでなくmapper後のPWM補償である。
  const auto minimum_command = actuators::mapFinOutputTorque(
      {0.001, 0.0, 0.0, true}, mapper_config);
  assert(minimum_command.valid && minimum_command.minimum_command_applied);
  assert(minimum_command.command_magnitude == 70);
  const auto no_minimum_command = actuators::mapFinOutputTorque(
      {0.001, 0.0, 0.0, false}, mapper_config);
  assert(no_minimum_command.valid &&
         !no_minimum_command.minimum_command_applied);
  assert(no_minimum_command.command_magnitude < 70);
  const auto negative_minimum_command = actuators::mapFinOutputTorque(
      {-0.001, 0.0, 0.0, true}, mapper_config);
  assert(negative_minimum_command.valid &&
         negative_minimum_command.minimum_command_applied);
  assert(negative_minimum_command.command_magnitude == 70);
  assert(negative_minimum_command.duty_signed < 0.0);
  const auto minimum_direction_reversal = actuators::mapFinOutputTorque(
      {0.001, 0.0, -0.1, true}, mapper_config);
  assert(minimum_direction_reversal.valid);
  assert(!minimum_direction_reversal.minimum_command_applied);
  assert(minimum_direction_reversal
             .minimum_command_rejected_torque_direction);
  assert(minimum_direction_reversal.effective_torque_nm > 0.0);
  const auto zero_command = actuators::mapFinOutputTorque(
      {0.0, 0.0, 0.0, true}, mapper_config);
  assert(zero_command.valid && zero_command.command_magnitude == 0);
  assert(!zero_command.coast_required);

  const auto minimum_limited_by_current = actuators::mapFinOutputTorque(
      {10.0, 0.0, -5.0, true}, mapper_config);
  assert(minimum_limited_by_current.valid);
  assert(minimum_limited_by_current.current_limited);
  assert(minimum_limited_by_current.minimum_command_limited_by_current);
  assert(std::abs(minimum_limited_by_current.estimated_motor_current_a) <=
         flight_config::kMotorMaxCurrentA + 1.0e-12);

  // 大要求でもsoftware torque clampではなくcurrent/duty制約で制限する。
  const auto current_limited = actuators::mapFinOutputTorque(
      {100.0, 0.0, 0.0, true}, mapper_config);
  assert(current_limited.valid && current_limited.current_limited);
  assert(current_limited.duty_limited);
  assert(std::abs(current_limited.estimated_motor_current_a) <=
         flight_config::kMotorMaxCurrentA + 1.0e-12);
  assert(std::abs(current_limited.duty_signed) <= 1.0);
  const auto duty_limited = actuators::mapFinOutputTorque(
      {100.0, 0.0, 5.0, true}, mapper_config);
  assert(duty_limited.valid && duty_limited.duty_limited);
  assert(std::abs(duty_limited.duty_signed) == 1.0);

  // back-EMFで電圧符号が反転してもoutward判定はrequested torque符号で行う。
  const auto outward_blocked = actuators::mapFinOutputTorque(
      {1.0, 15.0 * kDegToRad, -5.0, true}, mapper_config);
  assert(outward_blocked.valid && outward_blocked.outward_inhibited);
  assert(outward_blocked.coast_required);
  assert(outward_blocked.duty_signed == 0.0);
  assert(outward_blocked.effective_torque_nm == 0.0);
  const auto negative_outward_blocked = actuators::mapFinOutputTorque(
      {-1.0, -15.0 * kDegToRad, 5.0, true}, mapper_config);
  assert(negative_outward_blocked.valid &&
         negative_outward_blocked.outward_inhibited);
  assert(negative_outward_blocked.duty_signed == 0.0);
  assert(negative_outward_blocked.effective_torque_nm == 0.0);
  const auto inward_braking = actuators::mapFinOutputTorque(
      {-1.0, 15.0 * kDegToRad, 5.0, true}, mapper_config);
  assert(inward_braking.valid && !inward_braking.outward_inhibited);
  assert(!inward_braking.coast_required);
  assert(inward_braking.duty_signed > 0.0);
  assert(inward_braking.effective_torque_nm < 0.0);

  // 高back-EMFでbus voltage内にcurrent制約を実現できない場合、
  // 計算currentをclampして隠さずdriveを0へ落とす。
  const auto current_unrealizable = actuators::mapFinOutputTorque(
      {-0.1, 0.0, 20.0, true}, mapper_config);
  assert(current_unrealizable.valid);
  assert(current_unrealizable.current_limit_unrealizable);
  assert(current_unrealizable.coast_required);
  assert(std::abs(current_unrealizable.estimated_motor_current_a) >
         flight_config::kMotorMaxCurrentA);
  assert(current_unrealizable.duty_signed == 0.0);
  assert(current_unrealizable.command_magnitude == 0);
  assert(current_unrealizable.effective_torque_nm == 0.0);

  // current値自体が範囲内でも、bus/back-EMF制約後のtorque方向を実現
  // できない候補はdriveしない。
  auto high_speed_test_config = mapper_config;
  high_speed_test_config.motor_hard_speed_rpm = 100'000.0;
  const auto torque_direction_unrealizable =
      actuators::mapFinOutputTorque(
          {0.1, 0.0, 7.0, true}, high_speed_test_config);
  assert(torque_direction_unrealizable.valid);
  assert(!torque_direction_unrealizable.current_limit_unrealizable);
  assert(torque_direction_unrealizable.torque_direction_unrealizable);
  assert(torque_direction_unrealizable.coast_required);
  assert(torque_direction_unrealizable.estimated_motor_current_a < 0.0);
  assert(torque_direction_unrealizable.duty_signed == 0.0);
  assert(torque_direction_unrealizable.command_magnitude == 0);
  assert(torque_direction_unrealizable.effective_torque_nm == 0.0);

  const double hard_speed_rate_rad_s =
      1.001 * flight_config::kMotorHardSpeedRpm * 2.0 * kPi /
      (flight_config::kTotalGearRatio * 60.0);
  const auto hard_speed_acceleration = actuators::mapFinOutputTorque(
      {1.0, 0.0, hard_speed_rate_rad_s, true}, mapper_config);
  assert(hard_speed_acceleration.valid);
  assert(hard_speed_acceleration.motor_speed_inhibited);
  assert(hard_speed_acceleration.coast_required);
  assert(hard_speed_acceleration.gearbox_speed_exceeded);
  assert(hard_speed_acceleration.duty_signed == 0.0);
  assert(hard_speed_acceleration.effective_torque_nm == 0.0);
  const auto angle_and_speed_inhibited = actuators::mapFinOutputTorque(
      {1.0, 15.0 * kDegToRad, hard_speed_rate_rad_s, true}, mapper_config);
  assert(angle_and_speed_inhibited.valid);
  assert(angle_and_speed_inhibited.outward_inhibited);
  assert(angle_and_speed_inhibited.motor_speed_inhibited);
  assert(!angle_and_speed_inhibited.minimum_command_applied);
  assert(angle_and_speed_inhibited.duty_signed == 0.0);
  const auto hard_speed_braking = actuators::mapFinOutputTorque(
      {-1.0, 0.0, hard_speed_rate_rad_s, true}, mapper_config);
  assert(hard_speed_braking.valid);
  assert(!hard_speed_braking.motor_speed_inhibited);
  assert(!hard_speed_braking.coast_required);
  assert(hard_speed_braking.gearbox_speed_exceeded);
  assert(hard_speed_braking.effective_torque_nm < 0.0);
  const auto negative_hard_speed_acceleration =
      actuators::mapFinOutputTorque(
          {-1.0, 0.0, -hard_speed_rate_rad_s, true}, mapper_config);
  assert(negative_hard_speed_acceleration.valid);
  assert(negative_hard_speed_acceleration.motor_speed_inhibited);
  assert(negative_hard_speed_acceleration.duty_signed == 0.0);
  const auto negative_hard_speed_braking = actuators::mapFinOutputTorque(
      {1.0, 0.0, -hard_speed_rate_rad_s, true}, mapper_config);
  assert(negative_hard_speed_braking.valid);
  assert(!negative_hard_speed_braking.motor_speed_inhibited);
  assert(negative_hard_speed_braking.effective_torque_nm > 0.0);

  const auto invalid_mapper = actuators::mapFinOutputTorque(
      {std::numeric_limits<double>::infinity(), 0.0, 0.0, true},
      mapper_config);
  assert(!invalid_mapper.valid);

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
