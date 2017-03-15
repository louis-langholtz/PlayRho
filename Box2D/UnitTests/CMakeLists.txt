# CMake configuration file for UnitTests subdirectory.
# Top level docs for 3.1.3 at: https://cmake.org/cmake/help/v3.1/
# Commands herein described at: https://cmake.org/cmake/help/v3.1/manual/cmake-commands.7.html

add_subdirectory(googletest)

include_directories(${Box2D_SOURCE_DIR})

file(GLOB UnitTest_SRCS *.cpp)

# Add an executable to the project using specified source files.
# See details at: https://cmake.org/cmake/help/v3.1/command/add_executable.html
add_executable(UnitTests ${UnitTest_SRCS})

# Link a target to given libraries.
# See details at: https://cmake.org/cmake/help/v3.1/command/target_link_libraries.html
target_link_libraries(UnitTests Box2D gtest)
