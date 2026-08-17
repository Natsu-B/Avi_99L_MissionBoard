#include "runtime/runtime.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>

#include "esp_timer.h"
#include "freertos/task.h"
#include "protocol/wire.hpp"

namespace runtime {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;

bool allZero(const std::array<uint8_t, 6> &args) {
  return std::all_of(args.begin(), args.end(),
                     [](uint8_t value) { return value == 0; });
}

uint64_t elapsed(const mission::Snapshot &snapshot, uint64_t now_us) {
  if (!snapshot.liftoff_valid || now_us < snapshot.liftoff_us)
    return 0;
  return now_us - snapshot.liftoff_us;
}

bool deviceHealthy(diagnostics::DeviceState state) {
  return state == diagnostics::DeviceState::healthy;
}

} // namespace

void Runtime::canTask() {
  uint8_t status_sequence = 0;
  uint8_t kinematics_sequence = 0;
  uint8_t control_sequence = 0;
  uint8_t control_roll_sequence = 0;
  uint8_t lps_sequence = 0;
  uint8_t airspeed_sequence = 0;
  uint8_t device_health_sequence = 0;
  uint8_t last_reference_event_sequence = 0;

  uint64_t next_status = 0;
  uint64_t next_kinematics = 0;
  uint64_t next_control = 0;
  uint64_t next_control_roll = 0;
  uint64_t next_lps = 0;
  uint64_t next_airspeed = 0;
  uint64_t next_device_health = 0;

  TickType_t wake = xTaskGetTickCount();
  uint64_t next_can_retry = 0;
  uint64_t next_can_status = 0;
  bool can_healthy = false;

  for (;;) {
    const uint64_t now = static_cast<uint64_t>(esp_timer_get_time());

    if (!can_.initialized() && now >= next_can_retry) {
      can_healthy = initializeCan() == ESP_OK;
      next_can_retry = now + 1'000'000ULL;
    }
    if (can_.initialized() && now >= next_can_status) {
      CANCREATE::Status status{};
      const esp_err_t status_result = can_.getStatus(status);
      can_healthy =
          status_result == ESP_OK && status.state != CANCREATE::State::bus_off;
      if (status_result == ESP_OK &&
          status.state == CANCREATE::State::bus_off) {
        can_healthy =
            can_.recover(avi::Timeout::milliseconds(10)) == ESP_OK;
      }
      next_can_status = now + 100'000ULL;
    }

    CANCREATE::Frame raw{};
    while (can_.read(raw, avi::Timeout::noWait()) == ESP_OK) {
      protocol::CanFrame frame{};
      frame.identifier = raw.identifier;
      frame.data_length = raw.data_length;
      frame.extended = raw.extended;
      frame.remote = raw.remote;
      std::copy(std::begin(raw.data), std::end(raw.data), frame.data.begin());

      protocol::GenericCommandRequest request{};
      if (protocol::decodeGenericCommand(frame, request)) {
        if (request.transaction_id == 0 || !allZero(request.arguments)) {
          sendCanFrame(protocol::encode(
              {request.transaction_id, request.command,
               protocol::CommandPhase::rejected,
               protocol::CommandReason::invalid_argument, 0}));
          continue;
        }

        const auto lookup = command_cache_.lookup(request);
        if (lookup.kind != protocol::CommandCache::Lookup::miss) {
          sendCanFrame(protocol::encode(lookup.result));
          continue;
        }

        const auto code = static_cast<protocol::CommandCode>(request.command);
        const auto phase = state_.snapshot().phase;

        if (code == protocol::CommandCode::start_sequence) {
          command_cache_.rememberAccepted(request);
          sendCanFrame(protocol::encode(
              {request.transaction_id, request.command,
               protocol::CommandPhase::accepted, protocol::CommandReason::none,
               0}));

          const bool ok = state_.startSequence();
          const protocol::CommandResult final{
              request.transaction_id,
              request.command,
              ok ? protocol::CommandPhase::completed
                 : protocol::CommandPhase::failed,
              ok ? protocol::CommandReason::none
                 : protocol::CommandReason::invalid_state,
              0};
          command_cache_.finish(final);
          sendCanFrame(protocol::encode(final));
          continue;
        }

        if (code == protocol::CommandCode::cancel_sequence) {
          if (phase != mission::Phase::liftoff_detection) {
            const protocol::CommandResult rejected{
                request.transaction_id, request.command,
                protocol::CommandPhase::rejected,
                protocol::CommandReason::invalid_state, 0};
            command_cache_.rememberAccepted(request);
            command_cache_.finish(rejected);
            sendCanFrame(protocol::encode(rejected));
            continue;
          }

          command_cache_.rememberAccepted(request);
          sendCanFrame(protocol::encode(
              {request.transaction_id, request.command,
               protocol::CommandPhase::accepted, protocol::CommandReason::none,
               0}));

          const bool ok = state_.cancelSequence();
          const protocol::CommandResult final{
              request.transaction_id,
              request.command,
              ok ? protocol::CommandPhase::completed
                 : protocol::CommandPhase::failed,
              ok ? protocol::CommandReason::none
                 : protocol::CommandReason::invalid_state,
              0};
          command_cache_.finish(final);
          sendCanFrame(protocol::encode(final));
          continue;
        }

        const bool is_fin =
            code == protocol::CommandCode::fin_free ||
            code == protocol::CommandCode::fin_zero ||
            code == protocol::CommandCode::fin_hold;
        const bool is_para = code == protocol::CommandCode::para_open ||
                             code == protocol::CommandCode::para_close;

        if ((is_fin || is_para) &&
            phase != mission::Phase::command_receive) {
          const protocol::CommandResult rejected{
              request.transaction_id, request.command,
              protocol::CommandPhase::rejected,
              protocol::CommandReason::invalid_state, 0};
          command_cache_.rememberAccepted(request);
          command_cache_.finish(rejected);
          sendCanFrame(protocol::encode(rejected));
          continue;
        }

        if (is_fin) {
          bool expected = false;
          if (!fin_command_pending_.compare_exchange_strong(expected, true)) {
            const protocol::CommandResult rejected{
                request.transaction_id, request.command,
                protocol::CommandPhase::rejected,
                protocol::CommandReason::busy, 0};
            command_cache_.rememberAccepted(request);
            command_cache_.finish(rejected);
            sendCanFrame(protocol::encode(rejected));
            continue;
          }

          const ActuatorCommand command{request.transaction_id,
                                        request.command};
          if (xQueueSend(fin_command_queue_, &command, 0) != pdTRUE) {
            fin_command_pending_.store(false, std::memory_order_release);
            sendCanFrame(protocol::encode(
                {request.transaction_id, request.command,
                 protocol::CommandPhase::rejected,
                 protocol::CommandReason::busy, 0}));
            continue;
          }

          command_cache_.rememberAccepted(request);
          sendCanFrame(protocol::encode(
              {request.transaction_id, request.command,
               protocol::CommandPhase::accepted, protocol::CommandReason::none,
               0}));
          continue;
        }

        if (is_para) {
          bool expected = false;
          if (!para_command_pending_.compare_exchange_strong(expected, true)) {
            const protocol::CommandResult rejected{
                request.transaction_id, request.command,
                protocol::CommandPhase::rejected,
                protocol::CommandReason::busy, 0};
            command_cache_.rememberAccepted(request);
            command_cache_.finish(rejected);
            sendCanFrame(protocol::encode(rejected));
            continue;
          }

          const ActuatorCommand command{request.transaction_id,
                                        request.command};
          if (xQueueSend(para_command_queue_, &command, 0) != pdTRUE) {
            para_command_pending_.store(false, std::memory_order_release);
            sendCanFrame(protocol::encode(
                {request.transaction_id, request.command,
                 protocol::CommandPhase::rejected,
                 protocol::CommandReason::busy, 0}));
            continue;
          }

          command_cache_.rememberAccepted(request);
          sendCanFrame(protocol::encode(
              {request.transaction_id, request.command,
               protocol::CommandPhase::accepted, protocol::CommandReason::none,
               0}));
          continue;
        }

        const protocol::CommandResult rejected{
            request.transaction_id, request.command,
            protocol::CommandPhase::rejected,
            protocol::CommandReason::not_supported, 0};
        command_cache_.rememberAccepted(request);
        command_cache_.finish(rejected);
        sendCanFrame(protocol::encode(rejected));
        continue;
      }

      uint8_t emergency_transaction = 0;
      if (protocol::decodeEmergency(frame, emergency_transaction)) {
        protocol::GenericCommandRequest replay_key{};
        replay_key.transaction_id = emergency_transaction;
        replay_key.command = static_cast<uint8_t>(
            protocol::CommandCode::liftoff_emergency_result);

        const auto lookup = command_cache_.lookup(replay_key);
        if (lookup.kind != protocol::CommandCache::Lookup::miss) {
          sendCanFrame(protocol::encode(lookup.result));
          continue;
        }

        if (emergency_transaction != 0)
          command_cache_.rememberAccepted(replay_key);

        const bool accepted = emergency_transaction != 0 &&
                              state_.liftoffEmergencyRollback();
        const protocol::CommandResult result{
            emergency_transaction,
            static_cast<uint8_t>(
                protocol::CommandCode::liftoff_emergency_result),
            accepted ? protocol::CommandPhase::completed
                     : protocol::CommandPhase::rejected,
            accepted ? protocol::CommandReason::none
                     : (emergency_transaction == 0
                            ? protocol::CommandReason::invalid_argument
                            : protocol::CommandReason::invalid_state),
            0};

        if (emergency_transaction != 0)
          command_cache_.finish(result);
        sendCanFrame(protocol::encode(result));
      }
    }

    protocol::CommandResult task_result{};
    while (xQueueReceive(result_queue_, &task_result, 0) == pdTRUE) {
      command_cache_.finish(task_result);
      sendCanFrame(protocol::encode(task_result));
    }

    if (now >= next_kinematics) {
      const auto fin = fin_.telemetry();
      const bool reference_valid =
          control_reference_valid_.load(std::memory_order_acquire);
      const double roll_deviation_deg =
          control_roll_deviation_rad_.load(std::memory_order_acquire) *
          kRadToDeg;

      protocol::Kinematics message{};
      message.sequence = kinematics_sequence++;
      message.roll_raw =
          protocol::encodeRoll(roll_deviation_deg, reference_valid);
      message.roll_rate_raw = protocol::encodeRollRate(
          gyro_roll_rate_dps_.load(std::memory_order_acquire),
          imu_valid_.load(std::memory_order_acquire));
      message.fin_angle_raw =
          protocol::encodeFinAngle(fin.angle_deg,
                                   fin.encoder_valid &&
                                       fin.zero_reference_valid);
      message.fin_rate_raw =
          protocol::encodeFinRate(fin.rate_deg_s,
                                  fin.encoder_valid && fin.rate_valid);
      sendCanFrame(protocol::encode(message));
      next_kinematics = now + 10'000ULL;
    }

    if (now >= next_control) {
      const auto snapshot = state_.snapshot();
      protocol::ControlTelemetry message{};
      message.sequence = control_sequence++;
      message.requested_torque_raw = protocol::encodeRequestedTorque(
          requested_control_torque_nm_.load(std::memory_order_acquire),
          control_active_.load(std::memory_order_acquire));
      message.flight_elapsed_raw = protocol::encodeFlightElapsed(
          static_cast<double>(elapsed(snapshot, now)) * 1.0e-6,
          snapshot.liftoff_valid);
      sendCanFrame(protocol::encode(message));
      next_control = now + 10'000ULL;
    }

    if (now >= next_status) {
      const auto fin = fin_.telemetry();
      const auto para = para_.telemetry();

      protocol::MissionStatus message{};
      message.sequence = status_sequence++;
      message.state = wireState();
      // Vault 04aのFlightStatus bit割当へ合わせる。
      // bit5/6 battery, bit7 ComBoard SD, bit9..14 event系はMission単独で
      // 正確に生成できないためここでは0とし、個別device状態は0x10Bへ送る。
      message.flight_status = static_cast<uint16_t>(
          (lps_liftoff_detected_.load(std::memory_order_acquire) ? 1U << 0U
                                                                  : 0U) |
          (imu_liftoff_detected_.load(std::memory_order_acquire) ? 1U << 1U
                                                                  : 0U) |
          (deviceHealthy(imu_health_.state()) ? 1U << 2U : 0U) |
          (para.ready ? 1U << 3U : 0U) |
          (control_active_.load(std::memory_order_acquire) ? 1U << 4U : 0U) |
          (can_healthy ? 1U << 8U : 0U) |
          (control_permanently_disabled_.load(std::memory_order_acquire)
               ? 1U << 15U
               : 0U));

      message.fin_mode =
          fin.state == actuators::FinState::zero_hold
              ? protocol::FinMode::zero_hold
          : fin.state == actuators::FinState::roll_control
              ? protocol::FinMode::roll_control
          : fin.state == actuators::FinState::free
              ? protocol::FinMode::free
              : protocol::FinMode::unknown;
      message.para_mode = para.mode;
      message.parachute_angle_raw =
          protocol::encodeParachuteAngle(para.position_deg,
                                         para.position_valid);
      sendCanFrame(protocol::encode(message));
      next_status = now + 100'000ULL;
    }

    if (now >= next_lps) {
      protocol::LpsTelemetry message{};
      message.sequence = lps_sequence++;
      const bool valid = lps_valid_.load(std::memory_order_acquire);
      message.pressure_raw = protocol::encodeLpsPressure(
          lps_pressure_hpa_.load(std::memory_order_acquire), valid);
      message.temperature_raw = protocol::encodeLpsTemperature(
          lps_temperature_c_.load(std::memory_order_acquire), valid);
      sendCanFrame(protocol::encode(message));
      next_lps = now + 40'000ULL;
    }

    if (now >= next_airspeed) {
      protocol::AirspeedTelemetry message{};
      message.sequence = airspeed_sequence++;
      const bool valid = airspeed_valid_.load(std::memory_order_acquire);
      message.airspeed_raw = protocol::encodeAirspeed(
          airspeed_mps_.load(std::memory_order_acquire), valid);
      sendCanFrame(protocol::encode(message));
      next_airspeed = now + 10'000ULL;
    }

    if (now >= next_control_roll) {
      const bool reference_valid =
          control_reference_valid_.load(std::memory_order_acquire);
      const bool control_active =
          control_active_.load(std::memory_order_acquire);
      const uint8_t reference_event =
          reference_capture_event_sequence_.load(std::memory_order_acquire);
      const double deviation_deg =
          control_roll_deviation_rad_.load(std::memory_order_acquire) *
          kRadToDeg;

      protocol::ControlRollTelemetryV2 message{};
      message.sequence = control_roll_sequence++;
      message.control_roll_reference_unwrapped_raw =
          protocol::encodeRoll(0.0, reference_valid);
      message.roll_deviation_unwrapped_raw =
          protocol::encodeRoll(deviation_deg, reference_valid);
      if (reference_valid)
        message.flags |= protocol::ControlRollTelemetryV2::reference_valid;
      if (reference_event != last_reference_event_sequence)
        message.flags |=
            protocol::ControlRollTelemetryV2::
                reference_captured_since_previous_frame;
      if (control_active)
        message.flags |= protocol::ControlRollTelemetryV2::control_active;
      if (reference_valid &&
          message.control_roll_reference_unwrapped_raw == 0x800AU)
        message.flags |= protocol::ControlRollTelemetryV2::reference_out_of_range;
      if (reference_valid &&
          message.roll_deviation_unwrapped_raw == 0x800AU)
        message.flags |=
            protocol::ControlRollTelemetryV2::deviation_out_of_range;
      message.reference_capture_event_sequence = reference_event;
      last_reference_event_sequence = reference_event;

      sendCanFrame(protocol::encode(message));
      next_control_roll = now + 100'000ULL;
    }

    if (now >= next_device_health) {
      const auto fin = fin_.telemetry();
      protocol::DeviceHealthTelemetry message{};
      message.sequence = device_health_sequence++;
      message.icm42688 = imu_health_.state();
      message.as5047d = fin.encoder_state;
      message.lps25hb = lps_health_.state();
      message.ssc = ssc_health_.state();
      message.mission_sd = logger_.sdState();
      sendCanFrame(protocol::encode(message));
      next_device_health = now + 100'000ULL;
    }

    vTaskDelayUntil(&wake, pdMS_TO_TICKS(1));
  }
}

} // namespace runtime
