find_package(OpenGL REQUIRED)
if(NOT APPLE)
find_package(GLEW REQUIRED)
endif()

if(GLFW_INCLUDE_DIRS AND GLFW_LIBRARY_DIRS AND GLFW_STATIC_LIBRARIES AND OPENGL_INCLUDE_DIR)
	set(GLFW_TARGET OFF)
	message(STATUS "Values found for header and library paths.")
else()

	find_package(GLFW3 QUIET) #Find package assuming config file can be located
	message(STATUS "GLFW3_FOUND=${GLFW_FOUND}")
	
	if(GLFW3_FOUND)
		set(GLFW_TARGET ON)
	else() #Find package using pkg-config
		set(GLFW_TARGET OFF)
		
		message(STATUS "GLFW3 not found yet... Searching using pkg-config")
		find_package(PkgConfig REQUIRED)
		pkg_search_module(GLFW REQUIRED glfw3)
		
		find_library(GLFW_LIBRARY NAMES glfw3)
		message(STATUS "GLFW_STATIC_LIBRARIES=${GLFW_STATIC_LIBRARIES}")
		message(STATUS "GLFW_INCLUDE_DIRS=${GLFW_INCLUDE_DIRS}")
		message(STATUS "GLFW_LIBRARY_DIRS=${GLFW_LIBRARY_DIRS}")
		message(STATUS "OPENGL_INCLUDE_DIR=${OPENGL_INCLUDE_DIR}")
	endif()
	
endif()

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
	find_library(COREFOUNDATION_LIBRARY CoreFoundation)
	mark_as_advanced(COREFOUNDATION_LIBRARY)
	set(ADDITIONAL_LIBRARIES ${COREFOUNDATION_LIBRARY})
elseif(WIN32)
	set (ADDITIONAL_LIBRARIES winmm)
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
	${GLEW_INCLUDE_DIRS}
	${GLFW_INCLUDE_DIRS}
	${PlayRho_SOURCE_DIR}
)

message(STATUS "Setting link directories")
link_directories (
	${GLFW_LIBRARY_DIRS}
)

add_executable(Testbed
	${Testbed_Framework_SRCS}
	${Testbed_Tests_SRCS}
)

if(GLFW_TARGET)
	target_link_libraries(
		Testbed
		PlayRho
		glfw
		${GLEW_LIBRARY}
		${ADDITIONAL_LIBRARIES}
		${OPENGL_LIBRARIES}
	)
else()
	message(STATUS "Setting link libraries")
	target_link_libraries(
		Testbed
		PlayRho
		${GLEW_LIBRARY}
		${GLFW_STATIC_LIBRARIES}
		${ADDITIONAL_LIBRARIES}
		${OPENGL_LIBRARIES}
	)
endif()

# link with coverage library
if(${PLAYRHO_ENABLE_COVERAGE})
    if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
        # Use -ftest-coverage to generate .gcno notes files.
        # Use -fprofile-arcs to generate .gcda count data files when resulting objects are run.
        target_link_libraries(Testbed -fprofile-arcs -ftest-coverage)
    endif()
endif()


file(COPY Data DESTINATION ..)
