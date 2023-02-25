# Hello World examples
add_executable(HelloWorld HelloWorld.cpp)

target_link_libraries(HelloWorld PlayRho)

# link with coverage library
if(${PLAYRHO_ENABLE_COVERAGE})
    if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
        # Use -ftest-coverage to generate .gcno notes files.
        # Use -fprofile-arcs to generate .gcda count data files when resulting objects are run.
        target_link_libraries(HelloWorld -fprofile-arcs -ftest-coverage)
    endif()
endif()

if(PLAYRHO_INSTALL)
	include(GNUInstallDirs)
	install(TARGETS HelloWorld
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	COMPONENT Applications)
endif()
