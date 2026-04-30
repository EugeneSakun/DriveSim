# CMakeList.txt : Top-level CMake project file, do global configuration
# and include sub-projects here.
#
cmake_minimum_required (VERSION 3.10)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project ("DriveSim")

enable_testing()

add_library(nanoMODBUS STATIC
  lib/nanoMODBUS/nanomodbus.c
)

target_include_directories(nanoMODBUS PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/nanoMODBUS
)

add_library(tinycsocket STATIC
  lib/tinycsocket/tinycsocket.c
)

target_include_directories(tinycsocket PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/tinycsocket
)

add_library(tinycthread STATIC
  lib/tinycthread/tinycthread.c
)

target_include_directories(tinycthread PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/tinycthread
)

add_library(Unity STATIC
  lib/Unity/unity.c
)

target_include_directories(Unity PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/Unity
)

target_compile_definitions(Unity PUBLIC
  UNITY_INCLUDE_DOUBLE
)

# Include sub-projects.
add_subdirectory ("src")
add_subdirectory ("tests")
