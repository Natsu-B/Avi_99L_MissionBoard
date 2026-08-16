#include "mission/state_machine.hpp"

#include "config/flight.hpp"

namespace mission {

bool StateMachine::startSequence() {
  Phase expected = Phase::command_receive;
  if (!phase_.compare_exchange_strong(expected, Phase::liftoff_detection,
                                      std::memory_order_acq_rel))
    return false;
  liftoff_claimed_.store(false, std::memory_order_release);
  liftoff_valid_.store(false, std::memory_order_release);
  liftoff_us_.store(0, std::memory_order_release);
  deployment_started_.store(false, std::memory_order_release);
  power_cutoff_.store(false, std::memory_order_release);
  return true;
}

bool StateMachine::reportLiftoff(uint64_t detected_us) {
  bool unclaimed = false;
  if (!liftoff_claimed_.compare_exchange_strong(
          unclaimed, true, std::memory_order_acq_rel,
          std::memory_order_acquire))
    return false;
  if (phase_.load(std::memory_order_acquire) != Phase::liftoff_detection) {
    liftoff_claimed_.store(false, std::memory_order_release);
    return false;
  }

  const uint64_t liftoff_us =
      detected_us >= flight_config::kOneSecondUs
          ? detected_us - flight_config::kOneSecondUs
          : 0;
  generation_.fetch_add(1, std::memory_order_acq_rel);
  liftoff_us_.store(liftoff_us, std::memory_order_release);
  liftoff_valid_.store(true, std::memory_order_release);
  deployment_started_.store(false, std::memory_order_release);
  power_cutoff_.store(false, std::memory_order_release);
  phase_.store(Phase::flight, std::memory_order_release);
  return true;
}

bool StateMachine::liftoffEmergencyRollback() {
  Phase expected = Phase::flight;
  if (!phase_.compare_exchange_strong(expected, Phase::liftoff_detection,
                                      std::memory_order_acq_rel))
    return false;
  generation_.fetch_add(1, std::memory_order_acq_rel);
  liftoff_claimed_.store(false, std::memory_order_release);
  liftoff_valid_.store(false, std::memory_order_release);
  liftoff_us_.store(0, std::memory_order_release);
  deployment_started_.store(false, std::memory_order_release);
  power_cutoff_.store(false, std::memory_order_release);
  return true;
}

bool StateMachine::requestDescent(uint32_t generation) {
  if (generation == 0 ||
      generation_.load(std::memory_order_acquire) != generation ||
      !liftoff_valid_.load(std::memory_order_acquire))
    return false;
  Phase expected = Phase::flight;
  if (!phase_.compare_exchange_strong(expected, Phase::descent,
                                      std::memory_order_acq_rel))
    return false;
  deployment_started_.store(true, std::memory_order_release);
  return true;
}

void StateMachine::latchPowerCutoff() {
  power_cutoff_.store(true, std::memory_order_release);
}

Snapshot StateMachine::snapshot() const {
  Snapshot result{};
  result.phase = phase_.load(std::memory_order_acquire);
  result.generation = generation_.load(std::memory_order_acquire);
  result.liftoff_valid = liftoff_valid_.load(std::memory_order_acquire);
  result.liftoff_us = liftoff_us_.load(std::memory_order_acquire);
  result.deployment_started = deployment_started_.load(std::memory_order_acquire);
  result.power_cutoff = power_cutoff_.load(std::memory_order_acquire);
  return result;
}

} // namespace mission
