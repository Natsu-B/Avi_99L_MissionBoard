#include "runtime/runtime.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "actuators/safe_outputs.hpp"
#include "config/flight.hpp"
#include "esp_timer.h"
#include "freertos/task.h"

namespace runtime {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

protocol::CommandReason reasonForEsp(esp_err_t result) {
  if (result == ESP_OK)
    return protocol::CommandReason::none;
  if (result == ESP_ERR_TIMEOUT)
    return protocol::CommandReason::timeout;
  if (result == ESP_ERR_INVALID_STATE)
    return protocol::CommandReason::busy;
  return protocol::CommandReason::device_unavailable;
}

uint64_t elapsed(const mission::Snapshot &snapshot, uint64_t now_us) {
  if (!snapshot.liftoff_valid || now_us < snapshot.liftoff_us)
    return 0;
  return now_us - snapshot.liftoff_us;
}

bool fresh(uint64_t sample_us, uint64_t now_us, uint64_t maximum_age_us) {
  return sample_us != 0 && now_us >= sample_us &&
         now_us - sample_us <= maximum_age_us;
}

} // namespace

void Runtime::safetyTask() {
  TickType_t wake = xTaskGetTickCount();
  bool cutoff_applied = false;
  for (;;) {
    const uint64_t now = static_cast<uint64_t>(esp_timer_get_time());
    const auto snapshot = state_.snapshot();
    if ((snapshot.phase == mission::Phase::flight ||
         snapshot.phase == mission::Phase::descent) &&
        snapshot.liftoff_valid) {
      const uint64_t flight_elapsed = elapsed(snapshot, now);
      if (snapshot.phase == mission::Phase::flight &&
          flight_elapsed >= flight_config::kDeploymentFallbackUs)
        (void)state_.requestDescent(snapshot.generation);
      if (flight_elapsed >= flight_config::kAbsolutePowerCutoffUs &&
          !cutoff_applied) {
        state_.latchPowerCutoff();
        (void)actuators::safe_outputs::setParaPower(false);
        (void)actuators::safe_outputs::setAux5v(false);
        (void)actuators::safe_outputs::motorCoast();
        cutoff_applied = true;
      }
    }
    if (snapshot.phase == mission::Phase::liftoff_detection)
      cutoff_applied = false;
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(1));
  }
}

void Runtime::realtimeTask() {
  TickType_t wake = xTaskGetTickCount();
  mission::Phase previous_phase = mission::Phase::command_receive;
  uint64_t next_imu_retry = 0;
  bool local_logger_finished = false;

  for (;;) {
    const uint64_t now = static_cast<uint64_t>(esp_timer_get_time());
    auto snapshot = state_.snapshot();

    if (snapshot.phase != previous_phase) {
      if (snapshot.phase == mission::Phase::liftoff_detection) {
        imu_liftoff_.reset();
        resetControlSession();
        if (fin_.telemetry().zero_valid)
          (void)fin_.zeroHold();
      }
      if (snapshot.phase == mission::Phase::flight)
        resetControlSession();
      if (snapshot.phase == mission::Phase::descent) {
        control_active_.store(false, std::memory_order_release);
        requested_control_torque_nm_.store(0.0, std::memory_order_release);
        if (fin_.telemetry().zero_valid)
          (void)fin_.zeroHold();
      }
      if (previous_phase == mission::Phase::flight &&
          snapshot.phase == mission::Phase::liftoff_detection) {
        logger_.finishFlight();
        logger_started_.store(false, std::memory_order_release);
      }
      previous_phase = snapshot.phase;
    }

    ICM42688::Data imu_data{};
    bool imu_sample_valid = false;
    double corrected_roll_rate_dps = 0.0;
    if (!imu_.initialized() && now >= next_imu_retry) {
      const esp_err_t result = initializeImu();
      next_imu_retry = now + 1'000'000ULL;
      if (result != ESP_OK)
        imu_valid_.store(false, std::memory_order_release);
    }
    if (imu_.initialized() && imu_.read(imu_data) == ESP_OK) {
      imu_sample_valid = std::isfinite(imu_data.acceleration_g[0]) &&
                         std::isfinite(imu_data.acceleration_g[1]) &&
                         std::isfinite(imu_data.acceleration_g[2]) &&
                         std::isfinite(imu_data.angular_velocity_dps[2]);
      if (imu_sample_valid) {
        corrected_roll_rate_dps =
            -imu_data.angular_velocity_dps[2] -
            flight_config::kGyroRollBiasDps;
        imu_sample_valid = std::isfinite(corrected_roll_rate_dps);
      }
      imu_valid_.store(imu_sample_valid, std::memory_order_release);
      if (imu_sample_valid) {
        gyro_roll_rate_dps_.store(corrected_roll_rate_dps,
                                  std::memory_order_release);
        imu_sample_us_.store(now, std::memory_order_release);
      }
    } else {
      imu_valid_.store(false, std::memory_order_release);
    }

    if (snapshot.phase == mission::Phase::liftoff_detection &&
        !imu_sample_valid)
      (void)imu_liftoff_.update(0.0, 0.0, 0.0, false);

    if (snapshot.phase == mission::Phase::liftoff_detection &&
        imu_sample_valid) {
      if (imu_liftoff_.update(imu_data.acceleration_g[0],
                              imu_data.acceleration_g[1],
                              imu_data.acceleration_g[2], true)) {
        if (state_.reportLiftoff(now)) {
          snapshot = state_.snapshot();
          if (logger_.startFlight(snapshot.generation) == ESP_OK) {
            logger_started_.store(true, std::memory_order_release);
            logger_finished_.store(false, std::memory_order_release);
          }
          local_logger_finished = false;
        }
      }
    }

    fin_.update(now);

    ActuatorCommand fin_command{};
    while (xQueueReceive(fin_command_queue_, &fin_command, 0) == pdTRUE) {
      esp_err_t result = ESP_ERR_INVALID_ARG;
      if (fin_command.command ==
          static_cast<uint8_t>(protocol::CommandCode::fin_zero)) {
        result = fin_.setZero();
      } else if (fin_command.command ==
                 static_cast<uint8_t>(protocol::CommandCode::fin_hold)) {
        result = fin_.zeroHold();
      } else if (fin_command.command ==
                 static_cast<uint8_t>(protocol::CommandCode::fin_free)) {
        result = fin_.free();
      }

      pushResult({fin_command.transaction_id, fin_command.command,
                  result == ESP_OK ? protocol::CommandPhase::completed
                                   : protocol::CommandPhase::failed,
                  reasonForEsp(result), 0});
      fin_command_pending_.store(false, std::memory_order_release);
    }

    snapshot = state_.snapshot();
    const auto fin_telemetry = fin_.telemetry();

    if (snapshot.phase == mission::Phase::flight && snapshot.liftoff_valid &&
        !snapshot.power_cutoff) {
      const uint64_t flight_elapsed = elapsed(snapshot, now);
      const double roll_rate_rad_s = corrected_roll_rate_dps * kDegToRad;

      if (flight_elapsed < flight_config::kControlStartUs) {
        control_active_.store(false, std::memory_order_release);
        requested_control_torque_nm_.store(0.0, std::memory_order_release);
        if (fin_telemetry.zero_valid)
          (void)fin_.zeroHold();
      } else {
        if (!control_session_.gateEvaluated())
          (void)control_session_.evaluateEligibility(
              imu_sample_valid, fin_telemetry.zero_valid);

        const bool airspeed_valid =
            airspeed_valid_.load(std::memory_order_acquire) &&
            fresh(airspeed_sample_us_.load(std::memory_order_acquire), now,
                  flight_config::kAirspeedFreshUs);
        const double airspeed =
            airspeed_mps_.load(std::memory_order_acquire);
        control_session_.observeAirspeed(airspeed_valid, airspeed);

        if (control_session_.referenceStarted())
          (void)control_session_.observeGyro(now, imu_sample_valid,
                                             roll_rate_rad_s);
        if (!fin_telemetry.zero_valid)
          control_session_.disablePermanently();

        const bool imu_fresh =
            imu_sample_valid &&
            fresh(imu_sample_us_.load(std::memory_order_acquire), now,
                  flight_config::kImuFreshUs);
        const bool fin_fresh =
            fin_telemetry.encoder_valid && fin_telemetry.rate_valid &&
            fin_telemetry.zero_valid &&
            fresh(fin_telemetry.sample_timestamp_us, now,
                  flight_config::kFinFreshUs);
        const bool lps_fresh =
            lps_valid_.load(std::memory_order_acquire) &&
            fresh(lps_sample_us_.load(std::memory_order_acquire), now,
                  flight_config::kLpsFreshUs);
        const bool ssc_fresh =
            ssc_valid_.load(std::memory_order_acquire) &&
            fresh(ssc_sample_us_.load(std::memory_order_acquire), now,
                  flight_config::kSscFreshUs);

        bool control_applied = false;
        if (!control_session_.permanentlyDisabled() && imu_fresh &&
            fin_fresh && lps_fresh && ssc_fresh && airspeed_valid &&
            airspeed > flight_config::kAirspeedPermanentStopMps) {
          if (!control_session_.referenceStarted()) {
            if (control_session_.startReference(now, roll_rate_rad_s))
              reference_capture_event_sequence_.fetch_add(
                  1U, std::memory_order_acq_rel);
          }

          if (control_session_.referenceStarted() &&
              control_session_.estimatorValid()) {
            const control::RollControlInput input{
                control_session_.rollDeviationRad(),
                fin_telemetry.angle_deg * kDegToRad, roll_rate_rad_s,
                fin_telemetry.rate_deg_s * kDegToRad, airspeed};
            const auto output = roll_controller_.compute(input);
            const auto current_state = state_.snapshot();
            const bool same_flight =
                current_state.phase == mission::Phase::flight &&
                current_state.generation == snapshot.generation &&
                !current_state.power_cutoff;
            if (output.valid && same_flight &&
                fin_.setRollControlTorque(output.torque_nm) == ESP_OK) {
              requested_control_torque_nm_.store(output.torque_nm,
                                                 std::memory_order_release);
              control_applied = true;
            } else if (!output.valid) {
              control_session_.disablePermanently();
            }
          }
        }

        if (!control_applied) {
          requested_control_torque_nm_.store(0.0, std::memory_order_release);
          if (fin_telemetry.zero_valid)
            (void)fin_.zeroHold();
        }
        control_active_.store(control_applied, std::memory_order_release);
        control_permanently_disabled_.store(
            control_session_.permanentlyDisabled(), std::memory_order_release);
        control_reference_valid_.store(
            control_session_.referenceStarted() &&
                control_session_.estimatorValid(),
            std::memory_order_release);
        control_roll_deviation_rad_.store(
            control_session_.rollDeviationRad(), std::memory_order_release);
      }
    } else {
      control_active_.store(false, std::memory_order_release);
      requested_control_torque_nm_.store(0.0, std::memory_order_release);
      if (snapshot.phase == mission::Phase::descent &&
          fin_telemetry.zero_valid)
        (void)fin_.zeroHold();
    }

    if (snapshot.power_cutoff) {
      fin_.forceSafe();
      if (!local_logger_finished) {
        logger_.finishFlight();
        logger_finished_.store(true, std::memory_order_release);
        local_logger_finished = true;
      }
    }

    if (logger_started_.load(std::memory_order_acquire) &&
        snapshot.liftoff_valid && !snapshot.power_cutoff) {
      const auto fin = fin_.telemetry();
      const auto para = para_.telemetry();
      storage::LogSample sample{};
      sample.monotonic_us = now;
      sample.flight_elapsed_us = elapsed(snapshot, now);
      sample.generation = snapshot.generation;
      sample.phase = static_cast<uint8_t>(snapshot.phase);
      sample.flags = static_cast<uint8_t>(
          (imu_sample_valid ? 1U : 0U) |
          (lps_valid_.load(std::memory_order_acquire) ? 2U : 0U) |
          (ssc_valid_.load(std::memory_order_acquire) ? 4U : 0U) |
          (airspeed_valid_.load(std::memory_order_acquire) ? 8U : 0U) |
          (control_active_.load(std::memory_order_acquire) ? 16U : 0U) |
          (control_permanently_disabled_.load(std::memory_order_acquire)
               ? 32U
               : 0U) |
          (control_reference_valid_.load(std::memory_order_acquire) ? 64U
                                                                    : 0U));
      sample.fin_mode = static_cast<uint8_t>(
          fin.state == actuators::FinState::zero_hold
              ? protocol::FinMode::zero_hold
          : fin.state == actuators::FinState::roll_control
              ? protocol::FinMode::roll_control
          : fin.state == actuators::FinState::free
              ? protocol::FinMode::free
              : protocol::FinMode::unknown);
      sample.para_mode = static_cast<uint8_t>(para.mode);
      if (imu_sample_valid) {
        for (std::size_t i = 0; i < 3; ++i) {
          sample.acceleration_mg[i] = static_cast<int16_t>(std::clamp<long>(
              std::lround(imu_data.acceleration_g[i] * 1000.0), -32768,
              32767));
          sample.gyro_decidps[i] = static_cast<int16_t>(std::clamp<long>(
              std::lround(imu_data.angular_velocity_dps[i] * 10.0), -32768,
              32767));
        }
      }
      sample.fin_angle_cdeg = static_cast<int16_t>(std::clamp<long>(
          std::lround(fin.angle_deg * 100.0), -32768, 32767));
      sample.fin_rate_cdeg_s = static_cast<int16_t>(std::clamp<long>(
          std::lround(fin.rate_deg_s * 100.0), -32768, 32767));
      sample.pressure_pa = static_cast<int32_t>(std::clamp<long long>(
          std::llround(lps_pressure_hpa_.load(std::memory_order_acquire) *
                       100.0),
          std::numeric_limits<int32_t>::min(),
          std::numeric_limits<int32_t>::max()));
      sample.para_angle_decideg = static_cast<int16_t>(std::clamp<long>(
          std::lround(para.position_deg * 10.0), -32768, 32767));
      (void)logger_.append(sample);
    }

    vTaskDelayUntil(&wake, pdMS_TO_TICKS(1));
  }
}

} // namespace runtime
