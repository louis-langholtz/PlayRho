find_package(OpenGL REQUIRED)
if(NOT APPLE)
	find_package(GLEW REQUIRED)
	message(STATUS "GLEW_INCLUDE_DIR=${GLEW_INCLUDE_DIR}")
	message(STATUS "GLEW_LIBRARIES=${GLEW_LIBRARIES}")
endif()

find_package(GLFW3 REQUIRED)
message(STATUS "GLFW3_FOUND=${GLFW3_FOUND}")
message(STATUS "GLFW3_LIBRARY=${GLFW3_LIBRARY}")
message(STATUS "GLFW3_INCLUDE_DIR =${GLFW3_INCLUDE_DIR}")

# Some flags for Freeglut and GLUI.
if(APPLE)
	add_definitions( -DIMGUI_IMPL_OPENGL_LOADER_GL3W )
else()
	add_definitions( -DGLEW_STATIC -D_CRT_SECURE_NO_WARNINGS )
endif()

# Define the framework files.
file(GLOB Testbed_Framework_SRCS
	"Framework/*.cpp"
	"Framework/*.hpp"
	"Framework/imgui/imgui.cpp"
	"Framework/imgui/imgui_tables.cpp"
	"Framework/imgui/imgui_widgets.cpp"
	"Framework/imgui/imgui_draw.cpp"
	"Framework/imgui/backends/imgui_impl_opengl3.cpp"
	"Framework/imgui/backends/imgui_impl_glfw.cpp"
	"Framework/imgui/examples/libs/gl3w/GL/gl3w.c"
)

# define the test files.
file(GLOB Testbed_Tests_SRCS
	"Tests/*.cpp"
	"Tests/*.hpp"
)

# These are used to create visual studio folders.
source_group(Framework FILES ${Testbed_Framework_SRCS})
source_group(Tests FILES ${Testbed_Tests_SRCS})

if(APPLE)
	find_library(COCOA_LIBRARY Cocoa)
	find_library(COREFOUNDATION_LIBRARY CoreFoundation)
	find_library(COREVIDEO_LIBRARY CoreVideo)
	find_library(IOKIT_LIBRARY IOKit)
	find_library(OPENGL_LIBRARY OpenGL)
	mark_as_advanced(COREFOUNDATION_LIBRARY)
	set(ADDITIONAL_LIBRARIES ${COCOA_LIBRARY} ${COREFOUNDATION_LIBRARY} ${COREVIDEO_LIBRARY} ${IOKIT_LIBRARY} ${OPENGL_LIBRARY})
elseif(WIN32)
	set(ADDITIONAL_LIBRARIES winmm)
endif()

# Resolve Linker error LNK4098; make sure default libcmt doesn't clash with other libraries
if(MSVC)
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /NODEFAULTLIB:libcmt.lib")
endif()

message(STATUS "Including header directories")
include_directories(
	Framework/imgui
	Framework/imgui/examples/libs/gl3w
	${OPENGL_INCLUDE_DIR}
	${GLEW_INCLUDE_DIR}
	${GLFW3_INCLUDE_DIR}
	${PlayRho_SOURCE_DIR}
)

message(STATUS "Setting link directories")

add_executable(Testbed MACOSX_BUNDLE
	${Testbed_Framework_SRCS}
	${Testbed_Tests_SRCS}
)

target_link_libraries(
	Testbed
	PlayRho
	${GLFW3_LIBRARY}
	${GLEW_LIBRARIES}
	${ADDITIONAL_LIBRARIES}
	${OPENGL_LIBRARIES}
)

# link with coverage library
if(${PLAYRHO_ENABLE_COVERAGE})
    if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
        # Use -ftest-coverage to generate .gcno notes files.
        # Use -fprofile-arcs to generate .gcda count data files when resulting objects are run.
        target_link_libraries(Testbed -fprofile-arcs -ftest-coverage)
    endif()
endif()


file(COPY Data DESTINATION ..)
