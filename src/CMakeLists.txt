FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.cpp)
idf_component_register(
  SRCS ${app_sources}
  INCLUDE_DIRS "."
  REQUIRES driver esp_timer fatfs sdmmc
)
target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++17 -Wall -Wextra -Wpedantic)
