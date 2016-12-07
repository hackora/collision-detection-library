# Minimum CMake version required
cmake_minimum_required(VERSION 3.4)

# Nice name for the IDEs
project(collision_library_stub VERSION 0.1 LANGUAGES CXX)

# Specify compiler features
set(CMAKE_CXX_STANDARD 14)

# Find GMlib
find_package(
  GMlib 0.6.9 EXACT
  PATHS ${GMLIB_DIR}
  NO_DEFAULT_PATH
)
message( "GMlib_INCLUDE_DIRS: ${GMlib_INCLUDE_DIRS}" )
message( "GMlib_LINK_DIRS:    ${GMlib_LINK_DIRS}" )
message( "GMlib_DEFINITIONS:  ${GMlib_DEFINITIONS}" )
message( "GMlib_LIBRARIES:    ${GMlib_LIBRARIES}" )
include_directories( ${GMlib_INCLUDE_DIRS} ${GLEW_INCLUDE_DIRS} )
link_directories( ${GMlib_LINK_DIRS} )
add_definitions(${GMlib_DEFINITIONS})

message("COLLISION_INTERFACE_INCLUDE_DIR: ${COLLISION_INTERFACE_INCLUDE_DIR}")
if(NOT COLLISION_INTERFACE_INCLUDE_DIR)
  message( FATAL_ERROR "COLLISION_INTERFACE_INCLUDE_DIR not set")
endif()


include_directories(${COLLISION_INTERFACE_INCLUDE_DIR})


set(HDRS include/collision_library.h)

set(SRCS include/collision_library.cpp)

add_library( ${CMAKE_PROJECT_NAME} SHARED ${HDRS} ${SRCS} )


option( UNITTESTS "Enable building of unittests" OFF )
if(UNITTESTS)

  # Add the interface to the include directories
  include_directories(include)

  # GTest
  find_package(GTest)
  if(NOT GTEST_FOUND)
    message(FATAL_ERROR "GTest NOT found: Unittests disabled")
    message(FATAL_ERROR "GTest include dir: ${GTEST_INCLUDE_DIRS}")
  endif(NOT GTEST_FOUND)

  include_directories(${GTEST_INCLUDE_DIRS})
  message("GTest include dir: ${GTEST_INCLUDE_DIRS}")
  enable_testing()

  # Tests
  set(TESTS
    hello_world
    )

  # Create tetst
  foreach(TEST ${TESTS})
    add_executable( ${TEST} ${HDRS} unittests/${TEST}.cc  )
    target_link_libraries(${TEST}
      ${CMAKE_PROJECT_NAME}
      ${GMlib_LIBRARIES}
      ${GLEW_LIBRARIES}
      ${OPENGL_LIBRARIES}
      ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES} pthread)
    gtest_add_tests( ${TEST} "" AUTO )
  endforeach(TEST)

endif(UNITTESTS)
