# CMake configuration file for UnitTests subdirectory.
# Top level docs for 3.1.3 at: https://cmake.org/cmake/help/v3.1/
# Commands herein described at: https://cmake.org/cmake/help/v3.1/manual/cmake-commands.7.html

if (MSVC)
  if (BUILD_SHARED_LIBS)
    message(STATUS "UnitTests: Adding definition for linking with googletest as shared library.")
    add_definitions(-DGTEST_LINKED_AS_SHARED_LIBRARY)
  else()
    message(STATUS "UnitTests: Adding option setting for linking with googletest as static library.")
    set(gtest_force_shared_crt ON)
  endif()
endif()

# Add subdirectory to build.
# For cmd details, see: https://cmake.org/cmake/help/v3.1/command/add_subdirectory.html
# googletest build instructions at: https://github.com/google/googletest/blob/master/googletest/README.md
# Adds googletest here...
add_subdirectory(googletest)

# Add include directories to build...
# For cmd details, see: https://cmake.org/cmake/help/v3.1/command/include_directories.html
include_directories(${PlayRho_SOURCE_DIR})

file(GLOB UnitTest_SRCS *.cpp)

# Add an executable to the project using specified source files.
# See details at: https://cmake.org/cmake/help/v3.1/command/add_executable.html
add_executable(UnitTests ${UnitTest_SRCS})

# enable code coverage generation (only with GCC for now)
if(${PLAYRHO_ENABLE_COVERAGE} AND "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  message(STATUS "UnitTests: Adding definitions for coverage analysis.")
  add_definitions(-fprofile-arcs -ftest-coverage)
endif()

# Link a target to given libraries.
# See details at: https://cmake.org/cmake/help/v3.1/command/target_link_libraries.html
target_link_libraries(UnitTests PlayRho gtest)

# link with coverage library
if(${PLAYRHO_ENABLE_COVERAGE} AND "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  target_link_libraries(UnitTests -fprofile-arcs -ftest-coverage)
endif()
