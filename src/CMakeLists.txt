# CMakeList.txt : CMake project for DriveSim, include source and define
# project specific logic here.
#

# Add source to this project's executable.
add_executable (DriveSim
  "DriveSim.c"
  "ModbusTcp.c"
  "Vehicle.c"
)

target_link_libraries(DriveSim PRIVATE
  nanoMODBUS
  tinycsocket
  tinycthread
)

# TODO: Add tests and install targets if needed.
