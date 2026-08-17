#include "runtime/runtime.hpp"

#include <cmath>

#include "config/flight.hpp"
#include "esp_timer.h"
#include "freertos/task.h"

namespace runtime {
namespace {

constexpr uint8_t kAirDataFailureThreshold = 3U;

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

void Runtime::airTask() {
  TickType_t wake = xTaskGetTickCount();
  mission::Phase previous_phase = mission::Phase::command_receive;
  uint64_t next_device_retry = 0;
  uint64_t next_lps_poll = 0;

  for (;;) {
    const uint64_t now = static_cast<uint64_t>(esp_timer_get_time());
    const auto snapshot = state_.snapshot();

    if (snapshot.phase != previous_phase) {
      if (snapshot.phase == mission::Phase::liftoff_detection) {
        lps_liftoff_.reset();
        lps_liftoff_detected_.store(false, std::memory_order_release);
        differential_pressure_filter_.reset();
      }
      if (snapshot.phase == mission::Phase::flight)
        apex_.reset();
      previous_phase = snapshot.phase;
    }

    const bool lps_retry = lps_health_.recoveryRequested();
    const bool ssc_retry = ssc_health_.recoveryRequested();
    if (lps_retry || ssc_retry) {
      if (next_device_retry == 0) {
        next_device_retry = now + 1'000'000ULL;
      } else if (now >= next_device_retry) {
        if (lps_retry) {
          lps_health_.markRecovering();
          if (lps_.initialized() && lps_.end() != ESP_OK)
            lps_health_.markFailed();
        }
        if (ssc_retry) {
          ssc_health_.markRecovering();
          if (ssc_.initialized() && ssc_.end() != ESP_OK)
            ssc_health_.markFailed();
        }

        // 両deviceが同時に死んだ場合だけ共有I2C busも作り直す。
        if (lps_retry && ssc_retry && air_i2c_.initialized())
          (void)air_i2c_.end();

        (void)initializeAirData();
        next_device_retry = now + 1'000'000ULL;
      }
    } else {
      next_device_retry = 0;
    }

    bool lps_attempted = false;
    bool lps_new_valid = false;
    LPS25HB::Data lps_data{};
    if (now >= next_lps_poll) {
      lps_attempted = true;
      next_lps_poll = now + 40'000ULL;

      if (lps_.initialized()) {
        bool ready = false;
        const esp_err_t available_result = lps_.available(ready);
        if (available_result != ESP_OK) {
          lps_health_.markFailure(kAirDataFailureThreshold);
        } else if (ready) {
          const esp_err_t read_result = lps_.read(lps_data);
          if (read_result == ESP_OK &&
              std::isfinite(lps_data.pressure_pa) &&
              std::isfinite(lps_data.temperature_celsius)) {
            lps_new_valid = true;
            lps_health_.markHealthy();
            lps_pressure_hpa_.store(lps_data.pressure_pa / 100.0,
                                    std::memory_order_release);
            lps_temperature_c_.store(lps_data.temperature_celsius,
                                     std::memory_order_release);
            lps_sample_us_.store(now, std::memory_order_release);
          } else {
            lps_health_.markFailure(kAirDataFailureThreshold);
          }
        } else if (!fresh(lps_sample_us_.load(std::memory_order_acquire), now,
                          flight_config::kLpsFreshUs)) {
          // ready=false自体は正常だが、freshnessも失った場合は復旧対象にする。
          lps_health_.markFailure(kAirDataFailureThreshold);
        }
      } else {
        lps_health_.markFailed();
      }
    }

    bool ssc_new_valid = false;
    bool differential_valid = false;
    SSCDRRN005PD2A5::Data ssc_data{};
    double filtered_differential_pa = 0.0;
    if (ssc_.initialized()) {
      const esp_err_t ssc_result = ssc_.read(ssc_data);
      if (ssc_result == ESP_OK &&
          std::isfinite(ssc_data.differential_pressure_pa)) {
        ssc_new_valid = true;
        ssc_health_.markHealthy();
        ssc_sample_us_.store(now, std::memory_order_release);
        differential_valid = differential_pressure_filter_.update(
            ssc_data.differential_pressure_pa, filtered_differential_pa);
        if (differential_valid)
          differential_pressure_pa_.store(filtered_differential_pa,
                                          std::memory_order_release);
        else
          airspeed_valid_.store(false, std::memory_order_release);
      } else {
        ssc_health_.markFailure(kAirDataFailureThreshold);
      }
    } else {
      ssc_health_.markFailed();
    }

    const bool lps_fresh =
        fresh(lps_sample_us_.load(std::memory_order_acquire), now,
              flight_config::kLpsFreshUs);
    const bool ssc_fresh =
        fresh(ssc_sample_us_.load(std::memory_order_acquire), now,
              flight_config::kSscFreshUs);
    lps_valid_.store(lps_fresh, std::memory_order_release);
    ssc_valid_.store(ssc_fresh, std::memory_order_release);

    if (ssc_new_valid && differential_valid && lps_fresh) {
      // AirDataの温度源はLPS25HBへ統一し、SSCのoptional temperatureは使用しない。
      const auto result = sensors::computeSaintVenantAirspeed(
          lps_pressure_hpa_.load(std::memory_order_acquire) * 100.0,
          filtered_differential_pa,
          lps_temperature_c_.load(std::memory_order_acquire),
          flight_config::kPitotPressureCorrectionCoefficient);
      if (result.valid) {
        airspeed_mps_.store(result.airspeed_mps, std::memory_order_release);
        airspeed_sample_us_.store(now, std::memory_order_release);
      } else {
        airspeed_valid_.store(false, std::memory_order_release);
      }
    }

    const bool airspeed_fresh =
        fresh(airspeed_sample_us_.load(std::memory_order_acquire), now,
              flight_config::kAirspeedFreshUs);
    airspeed_valid_.store(airspeed_fresh, std::memory_order_release);

    if (lps_attempted) {
      if (snapshot.phase == mission::Phase::liftoff_detection) {
        if (lps_new_valid) {
          if (lps_liftoff_.update(lps_data.pressure_pa / 100.0, true)) {
            lps_liftoff_detected_.store(true, std::memory_order_release);
            if (state_.reportLiftoff(now)) {
              const auto current = state_.snapshot();
              if (logger_.startFlight(current.generation) == ESP_OK) {
                logger_started_.store(true, std::memory_order_release);
                logger_finished_.store(false, std::memory_order_release);
              }
            }
          }
        } else {
          (void)lps_liftoff_.update(0.0, false);
        }
      } else if (snapshot.phase == mission::Phase::flight &&
                 snapshot.liftoff_valid) {
        if (lps_new_valid) {
          if (apex_.update(lps_data.pressure_pa / 100.0, true,
                           elapsed(snapshot, now)))
            (void)state_.requestDescent(snapshot.generation);
        } else {
          (void)apex_.update(0.0, false, elapsed(snapshot, now));
        }
      }
    }

    vTaskDelayUntil(&wake, pdMS_TO_TICKS(3));
  }
}

void Runtime::paraTask() {
  (void)para_.initialize();
  uint32_t deployment_started_generation = 0;
  uint64_t next_deployment_retry = 0;
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    const uint64_t now = static_cast<uint64_t>(esp_timer_get_time());
    const auto snapshot = state_.snapshot();
    if (snapshot.power_cutoff) {
      para_.forcePowerOff();
    } else {
      para_.allowPower();
      const auto completion = para_.update(now);
      if (completion.has_value()) {
        pushResult({completion->transaction_id, completion->command,
                    completion->reason == protocol::CommandReason::none
                        ? protocol::CommandPhase::completed
                        : protocol::CommandPhase::failed,
                    completion->reason, 0});
        para_command_pending_.store(false, std::memory_order_release);
      }

      ActuatorCommand command{};
      if (!para_.busy() &&
          xQueueReceive(para_command_queue_, &command, 0) == pdTRUE) {
        const esp_err_t result = para_.startCommand(
            command.transaction_id,
            static_cast<protocol::CommandCode>(command.command));
        if (result != ESP_OK) {
          pushResult({command.transaction_id, command.command,
                      protocol::CommandPhase::failed, reasonForEsp(result), 0});
          para_command_pending_.store(false, std::memory_order_release);
        }
      }

      if (snapshot.phase == mission::Phase::descent &&
          snapshot.generation != 0 &&
          deployment_started_generation != snapshot.generation &&
          !para_.busy() && now >= next_deployment_retry) {
        const esp_err_t result = para_.startDeployment(snapshot.generation);
        if (result == ESP_OK)
          deployment_started_generation = snapshot.generation;
        else
          next_deployment_retry = now + 250'000ULL;
      }
      if (snapshot.phase == mission::Phase::liftoff_detection)
        deployment_started_generation = 0;
    }
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(10));
  }
}

} // namespace runtime
