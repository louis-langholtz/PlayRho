# Hello World examples
include_directories (${PlayRho_SOURCE_DIR})
add_executable(HelloWorld HelloWorld.cpp)
target_link_libraries (HelloWorld PlayRho)

# link with coverage library
if(${PLAYRHO_ENABLE_COVERAGE} AND "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    target_link_libraries(HelloWorld -fprofile-arcs -ftest-coverage)
endif()
