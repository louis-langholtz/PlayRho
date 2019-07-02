# CMake configuration file for UnitTests subdirectory.
# Top level docs for 3.1.3 at: https://cmake.org/cmake/help/v3.1/
# Commands herein described at: https://cmake.org/cmake/help/v3.1/manual/cmake-commands.7.html

# Hides options.
mark_as_advanced(FORCE BUILD_GMOCK BUILD_GTEST BUILD_SHARED_LIBS)
mark_as_advanced(FORCE INSTALL_GMOCK INSTALL_GTEST)
mark_as_advanced(FORCE gmock_build_tests)
mark_as_advanced(FORCE gtest_build_samples gtest_build_tests gtest_disable_pthreads gtest_hide_internal_symbols)
mark_as_advanced(FORCE gtest_force_shared_crt)
mark_as_advanced(FORCE CMAKE_DEBUG_POSTFIX)

set(BUILD_GMOCK ON)
set(INSTALL_GTEST OFF CACHE BOOL "Enable installation of googletest.")
set(INSTALL_GMOCK OFF CACHE BOOL "Enable installation of googlemock.")

if (MSVC)
  if (PLAYRHO_BUILD_SHARED)
    message(STATUS "UnitTests: Adding definition for linking with googletest as shared library.")
    add_definitions(-DGTEST_LINKED_AS_SHARED_LIBRARY)
  else()
    message(STATUS "UnitTests: Setting option value for linking googletest with static library.")
    set(gtest_force_shared_crt ON CACHE BOOL "Required setting for linking googletest with static library." FORCE)
  endif()
endif()

# Add subdirectory to build.
# For cmd details, see: https://cmake.org/cmake/help/v3.1/command/add_subdirectory.html
# googletest build instructions at: https://github.com/google/googletest/blob/master/googletest/README.md
# Adds googletest here...
add_subdirectory(googletest EXCLUDE_FROM_ALL)

# Add include directories to build...
# For cmd details, see: https://cmake.org/cmake/help/v3.1/command/include_directories.html
include_directories(${PlayRho_SOURCE_DIR})

file(GLOB UnitTest_SRCS *.cpp)

# Add an executable to the project using specified source files.
# See details at: https://cmake.org/cmake/help/v3.1/command/add_executable.html
add_executable(UnitTests ${UnitTest_SRCS})

# enable code coverage generation (only with GCC for now)
if(${PLAYRHO_ENABLE_COVERAGE})
    if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
        # Use -ftest-coverage to generate .gcno notes files.
        # Use -fprofile-arcs to generate .gcda count data files when resulting objects are run.
        message(STATUS "UnitTests: Adding definitions for coverage analysis.")
        add_definitions(-fprofile-arcs -ftest-coverage)
    endif()
endif()

# Link a target to given libraries.
# See details at: https://cmake.org/cmake/help/v3.1/command/target_link_libraries.html
target_link_libraries(UnitTests PlayRho gtest)

# link with coverage library
if(${PLAYRHO_ENABLE_COVERAGE})
    if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
        # Use -ftest-coverage to generate .gcno notes files.
        # Use -fprofile-arcs to generate .gcda count data files when resulting objects are run.
        target_link_libraries(UnitTests -fprofile-arcs -ftest-coverage)
    endif()
endif()

# Adds UnitTests to be run when like "make test" is executed.
add_test(UnitTests UnitTests)
