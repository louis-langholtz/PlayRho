# This is a configuration file for CMake.
# Top level docs for 3.1.3 at: https://cmake.org/cmake/help/v3.1/
# Commands herein described at: https://cmake.org/cmake/help/v3.1/manual/cmake-commands.7.html

# Minimally require CMake version 3.1...
cmake_minimum_required(VERSION 3.1)

# Require C++14...
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Defaults the CMAKE_BUILD_TYPE to Release
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Semicolon separated list of supported configuration types, only
# supports Debug, Release, MinSizeRel, and RelWithDebInfo, anything
# else will be ignored. PlayRho only supports Debug and Release
set(CMAKE_CONFIGURATION_TYPES "Debug;Release;" CACHE STRING "Semicolon separated list of supported configuration types. PlayRho only supports Debug and Release configurations.")

# Set name for entire project.
# Details at: https://cmake.org/cmake/help/v3.1/command/project.html
project(PlayRho)

# Tell C++ compiler to optimize release builds for speed.
# In clang++, the optimize for speed flag is '-Ot'. This option isn't supported on g++
# however and it'd be nice to use an option that works for both compilers. So use '-O3'.
# For Visual Studio Compilers, the speed optimization flag is /O2, which is the default setting for release builds
if(MSVC)
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")
else()
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
endif()

# Provide options that user can optionally select.
# Details at: https://cmake.org/cmake/help/v3.1/command/option.html
option(PLAYRHO_BUILD_STATIC "Build PlayRho static libraries" ON)
option(PLAYRHO_BUILD_SHARED "Build PlayRho shared libraries" OFF)
option(PLAYRHO_INSTALL "Install PlayRho libs, includes, and CMake scripts" OFF)
option(PLAYRHO_INSTALL_DOC "Install PlayRho documentation" OFF)
option(PLAYRHO_BUILD_HELLOWORLD "Build PlayRho HelloWorld console application" OFF)
option(PLAYRHO_BUILD_UNIT_TESTS "Build PlayRho unit tests" OFF)
option(PLAYRHO_BUILD_TESTBED "Build PlayRho Testbed GUI application" OFF)
option(PLAYRHO_ENABLE_COVERAGE "Enable code coverage generation" OFF)

set(PLAYRHO_VERSION 0.9.0)
set(LIB_INSTALL_DIR lib${LIB_SUFFIX})

# The PlayRho library.
add_subdirectory(PlayRho)

# HelloWorld console example.
if(PLAYRHO_BUILD_HELLOWORLD)
  add_subdirectory(HelloWorld)
endif(PLAYRHO_BUILD_HELLOWORLD)

# Testbed GUI application.
if(PLAYRHO_BUILD_TESTBED)
  add_subdirectory(Testbed)
endif(PLAYRHO_BUILD_TESTBED)

# Unit tests console application.
if(PLAYRHO_BUILD_UNIT_TESTS)
  add_subdirectory(UnitTests)
endif(PLAYRHO_BUILD_UNIT_TESTS)

if(PLAYRHO_INSTALL_DOC)
  install(DIRECTORY Documentation DESTINATION share/doc/PlayRho PATTERN ".svn" EXCLUDE)
endif(PLAYRHO_INSTALL_DOC)
