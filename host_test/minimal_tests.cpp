#include <cassert>

#include "mission/flight_detectors.hpp"
#include "mission/state_machine.hpp"
#include "protocol/command_cache.hpp"
#include "protocol/wire.hpp"

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
    apex_detected = apex.update(1000.0 + 0.01 * i, true, 11'000'000) || apex_detected;
  assert(apex_detected);

  protocol::CommandCache cache;
  protocol::GenericCommandRequest request{};
  request.transaction_id = 1;
  request.command = static_cast<uint8_t>(protocol::CommandCode::para_open);
  assert(cache.lookup(request).kind == protocol::CommandCache::Lookup::miss);
  cache.rememberAccepted(request);
  assert(cache.lookup(request).kind == protocol::CommandCache::Lookup::replay);
  protocol::CommandResult final{1, request.command, protocol::CommandPhase::completed,
                                protocol::CommandReason::none, 0};
  cache.finish(final);
  assert(cache.lookup(request).result.phase == protocol::CommandPhase::completed);
  auto conflict = request;
  conflict.command = static_cast<uint8_t>(protocol::CommandCode::para_close);
  assert(cache.lookup(conflict).kind == protocol::CommandCache::Lookup::conflict);

  protocol::CanFrame frame{};
  frame.identifier = static_cast<uint16_t>(protocol::CanId::generic_command_request);
  frame.data_length = 8;
  frame.data[0] = 7;
  frame.data[1] = static_cast<uint8_t>(protocol::CommandCode::fin_hold_current);
  protocol::GenericCommandRequest decoded{};
  assert(protocol::decodeGenericCommand(frame, decoded));
  assert(decoded.transaction_id == 7);
  assert(decoded.command == frame.data[1]);
  return 0;
}
