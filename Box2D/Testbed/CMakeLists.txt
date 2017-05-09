find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)
message("GLFW_FOUND=${GLFW_FOUND}")

find_library(GLFW_LIBRARY NAMES glfw3)
message("GLFW_LIBRARY=${GLFW_LIBRARY}")
message("GLFW_LIBRARY_DIRS=${GLFW_LIBRARY_DIRS}")

# Some flags for Freeglut and GLUI.
add_definitions( -DGLEW_STATIC -D_CRT_SECURE_NO_WARNINGS )

# Define the framework files.

file(GLOB Testbed_Framework_SRCS
	"Framework/*.cpp"
	"Framework/*.hpp"
)

#define the test files.
file(GLOB Testbed_Tests_SRCS
	"Tests/*.cpp"
	"Tests/*.hpp"
)

# These are used to create visual studio folders.
source_group(Framework FILES ${Testbed_Framework_SRCS})
source_group(Tests FILES ${Testbed_Tests_SRCS})

include_directories (
	${OPENGL_INCLUDE_DIR}
	${GLEW_INCLUDE_DIRS}
	${GLFW_INCLUDE_DIRS}
	${Box2D_SOURCE_DIR}
)

link_directories (
  ${GLFW_LIBRARY_DIRS}
)

if(APPLE)
	# We are not using the Apple's framework version, but X11's
	include_directories( /usr/X11/include )
	link_directories( /usr/X11/lib )
	set (OPENGL_LIBRARIES GL GLU X11)
elseif(WIN32)
	set (ADDITIONAL_LIBRARIES winmm)
endif(APPLE)

add_executable(Testbed
	${Testbed_Framework_SRCS}
	${Testbed_Tests_SRCS}
)

target_link_libraries (
	Testbed
	Box2D
	${GLEW_LIBRARY}
	${GLFW_STATIC_LIBRARIES}
	${ADDITIONAL_LIBRARIES}
	${OPENGL_LIBRARIES}
)

file(COPY ../Build/Data DESTINATION ..)
