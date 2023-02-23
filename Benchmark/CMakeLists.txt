# CMake configuration file for Benchmark subdirectory.
# Top level docs for 3.1.3 at: https://cmake.org/cmake/help/v3.1/
# Commands herein described at: https://cmake.org/cmake/help/v3.1/manual/cmake-commands.7.html

# Hides options.
mark_as_advanced(FORCE LIBRT)
mark_as_advanced(FORCE BENCHMARK_BUILD_32_BITS)
mark_as_advanced(FORCE BENCHMARK_ENABLE_EXCEPTIONS)
mark_as_advanced(FORCE BENCHMARK_ENABLE_INSTALL)
mark_as_advanced(FORCE BENCHMARK_ENABLE_TESTING)
mark_as_advanced(FORCE BENCHMARK_ENABLE_LTO)
mark_as_advanced(FORCE BENCHMARK_ENABLE_DOXYGEN)
mark_as_advanced(FORCE BENCHMARK_USE_LIBCXX)
mark_as_advanced(FORCE BENCHMARK_INSTALL_DOCS)

set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "Enable testing of the benchmark library.")
set(BENCHMARK_ENABLE_INSTALL OFF CACHE BOOL "Enable installation of the benchmark library.")
set(BENCHMARK_INSTALL_DOCS OFF CACHE BOOL "Enable installation of docs.")

# Add subdirectory to build.
# For cmd details, see: https://cmake.org/cmake/help/v3.1/command/add_subdirectory.html
# Adds benchmark here...
add_subdirectory(googlebench)

# Add include directories to build...
# For cmd details, see: https://cmake.org/cmake/help/v3.1/command/include_directories.html
include_directories(${PlayRho_SOURCE_DIR})

file(GLOB Benchmark_SRCS *.cpp)

# Add an executable to the project using specified source files.
# See details at: https://cmake.org/cmake/help/v3.1/command/add_executable.html
add_executable(Benchmark ${Benchmark_SRCS})

# Link a target to given libraries.
# See details at: https://cmake.org/cmake/help/v3.1/command/target_link_libraries.html
target_link_libraries(Benchmark PlayRho benchmark::benchmark)

# link with coverage library
if(${PLAYRHO_ENABLE_COVERAGE})
    if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
        # Use -ftest-coverage to generate .gcno notes files.
        # Use -fprofile-arcs to generate .gcda count data files when resulting objects are run.
        target_link_libraries(Benchmark -fprofile-arcs -ftest-coverage)
    endif()
endif()

if(PLAYRHO_INSTALL)
	include(GNUInstallDirs)
	install(TARGETS Benchmark
	        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	        COMPONENT Applications)
endif()
