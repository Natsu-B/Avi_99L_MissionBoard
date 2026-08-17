#include <cassert>

#include "mission/state_machine.hpp"

int main() {
  mission::StateMachine state;

  // CancelSequenceはLiftoffDetectionでのみ受理する。
  assert(!state.cancelSequence());
  assert(state.startSequence());
  assert(state.snapshot().phase == mission::Phase::liftoff_detection);

  const uint32_t generation_before_cancel = state.snapshot().generation;
  assert(state.cancelSequence());
  const auto cancelled = state.snapshot();
  assert(cancelled.phase == mission::Phase::command_receive);
  assert(cancelled.generation != generation_before_cancel);
  assert(!cancelled.liftoff_valid);
  assert(cancelled.liftoff_us == 0);
  assert(!cancelled.deployment_started);
  assert(!cancelled.power_cutoff);

  // Cancel後は新しいsequenceを開始できる。
  assert(state.startSequence());
  assert(state.reportLiftoff(5'000'000));
  assert(state.snapshot().phase == mission::Phase::flight);
  assert(!state.cancelSequence());

  const auto flight = state.snapshot();
  assert(state.requestDescent(flight.generation));
  assert(state.snapshot().phase == mission::Phase::descent);
  assert(!state.cancelSequence());

  return 0;
}
