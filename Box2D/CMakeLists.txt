# This is a configuration file for CMake.
# Top level docs for 3.1.3 at: https://cmake.org/cmake/help/v3.1/
# Commands herein described at: https://cmake.org/cmake/help/v3.1/manual/cmake-commands.7.html

# Minimally require CMake version 3.1...
cmake_minimum_required(VERSION 3.1)

# Require C++14...
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set name for entire project.
# Details at: https://cmake.org/cmake/help/v3.1/command/project.html
project(Box2D)

# Provide options that user can optionally select.
# Details at: https://cmake.org/cmake/help/v3.1/command/option.html
option(BOX2D_INSTALL "Install Box2D libs, includes, and CMake scripts" OFF)
option(BOX2D_INSTALL_DOC "Install Box2D documentation" OFF)
option(BOX2D_BUILD_SHARED "Build Box2D shared libraries" OFF)
option(BOX2D_BUILD_STATIC "Build Box2D static libraries" ON)
option(BOX2D_BUILD_EXAMPLES "Build Box2D examples" OFF)
option(BOX2D_BUILD_UNIT_TESTS "Build Box2D unit tests" OFF)

set(BOX2D_VERSION 3.0.0)
set(LIB_INSTALL_DIR lib${LIB_SUFFIX})

# The Box2D library.
add_subdirectory(Box2D)

if(BOX2D_BUILD_EXAMPLES)
  # HelloWorld console example.
  add_subdirectory(HelloWorld)

  # Testbed and dependencies.
  find_package(OpenGL REQUIRED)
  add_subdirectory(glew)
  add_subdirectory(glfw)
  add_subdirectory(Testbed)
endif(BOX2D_BUILD_EXAMPLES)

if(BOX2D_BUILD_UNIT_TESTS)
  add_subdirectory(UnitTests)
endif(BOX2D_BUILD_UNIT_TESTS)

if(BOX2D_INSTALL_DOC)
  install(DIRECTORY Documentation DESTINATION share/doc/Box2D PATTERN ".svn" EXCLUDE)
endif(BOX2D_INSTALL_DOC)
