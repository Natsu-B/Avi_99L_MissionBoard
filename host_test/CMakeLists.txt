cmake_minimum_required(VERSION 3.16)
project(avi_99l_minimal_host_tests LANGUAGES CXX)
enable_testing()
add_executable(minimal_tests
  minimal_tests.cpp
  ../src/mission/state_machine.cpp
  ../src/mission/flight_detectors.cpp
  ../src/protocol/wire.cpp
  ../src/protocol/command_cache.cpp
  ../src/control/roll_control.cpp
  ../src/control/zero_hold.cpp
  ../src/actuators/fin_torque_mapper.cpp
  ../src/sensors/airspeed.cpp
)
target_include_directories(minimal_tests PRIVATE ../src)
target_compile_features(minimal_tests PRIVATE cxx_std_17)
target_compile_options(minimal_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME minimal_tests COMMAND minimal_tests)

add_executable(cancel_sequence_tests
  cancel_sequence_tests.cpp
  ../src/mission/state_machine.cpp
)
target_include_directories(cancel_sequence_tests PRIVATE ../src)
target_compile_features(cancel_sequence_tests PRIVATE cxx_std_17)
target_compile_options(cancel_sequence_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME cancel_sequence_tests COMMAND cancel_sequence_tests)
