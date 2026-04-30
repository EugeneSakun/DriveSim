add_executable(VehicleTests
  "VehicleTests.c"
  "${CMAKE_SOURCE_DIR}/src/Vehicle.c"
)

target_include_directories(VehicleTests PRIVATE
  "${CMAKE_SOURCE_DIR}/src"
)

target_link_libraries(VehicleTests PRIVATE
  Unity
)

add_test(NAME VehicleTests COMMAND $<TARGET_FILE:VehicleTests>)
