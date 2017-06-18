# Hello World examples
include_directories (${Box2D_SOURCE_DIR})
add_executable(HelloWorld HelloWorld.cpp)
target_link_libraries (HelloWorld Box2D)

# link with coverage library
if(${BOX2D_ENABLE_COVERAGE} AND "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    target_link_libraries(HelloWorld -fprofile-arcs -ftest-coverage)
endif()
