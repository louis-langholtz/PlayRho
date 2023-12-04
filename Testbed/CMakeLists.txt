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

set(IMGUI_SRCS
    "imgui.cpp"
    "imgui_tables.cpp"
    "imgui_widgets.cpp"
    "imgui_draw.cpp"
    "backends/imgui_impl_opengl3.cpp"
    "backends/imgui_impl_glfw.cpp"
    "examples/libs/gl3w/GL/gl3w.c"
    )

set(IMGUI_INCLUDE_DIRS)
set(IMGUI_PREFIXED_SRCS)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/Framework/imgui/imgui.h")
    foreach(IMGUI_SRC IN LISTS IMGUI_SRCS)
        list(APPEND IMGUI_PREFIXED_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/Framework/imgui/${IMGUI_SRC}")
    endforeach()
    list(APPEND IMGUI_INCLUDE_DIRS "Framework/imgui" "Framework/imgui/examples/libs/gl3w")
else()
    include(FetchContent)
    message(STATUS "Attempting to fetch ImGui source code.")
    FetchContent_Declare(
        imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui.git
        GIT_TAG 393941ceea61e1307ffad8e3a045ab384d1299bd # <HASH or TAG>
    )
    FetchContent_MakeAvailable(imgui)
    foreach(IMGUI_SRC IN LISTS IMGUI_SRCS)
        list(APPEND IMGUI_PREFIXED_SRCS "${imgui_SOURCE_DIR}/${IMGUI_SRC}")
    endforeach()
    list(APPEND IMGUI_INCLUDE_DIRS "${imgui_SOURCE_DIR}" "${imgui_SOURCE_DIR}/examples/libs/gl3w")
endif()

# Define the framework files.
# /bin/ls -1 Framework/*.[hc]pp
set(Testbed_Framework_SRCS
    Framework/DebugDraw.cpp
    Framework/DebugDraw.hpp
    Framework/Drawer.cpp
    Framework/Drawer.hpp
    Framework/ExtensionsForImgui.cpp
    Framework/ExtensionsForImgui.hpp
    Framework/Main.cpp
    Framework/Test.cpp
    Framework/Test.hpp
    Framework/TestEntry.cpp
    Framework/TestEntry.hpp
    Framework/UiState.hpp
)

# define the test files.
# /bin/ls -1 Tests/*.[hc]pp
set(Testbed_Tests_SRCS
    Tests/AddPair.cpp
    Tests/ApplyForce.cpp
    Tests/BagOfDisks.cpp
    Tests/BasicSliderCrank.cpp
    Tests/BodyTypes.cpp
    Tests/Breakable.cpp
    Tests/BreakableTwo.cpp
    Tests/Bridge.cpp
    Tests/BulletTest.cpp
    Tests/Cantilever.cpp
    Tests/Car.cpp
    Tests/Chain.cpp
    Tests/CharacterCollision.cpp
    Tests/CollisionFiltering.cpp
    Tests/CollisionProcessing.cpp
    Tests/CompoundShapes.cpp
    Tests/Confined.cpp
    Tests/ContinuousTest.cpp
    Tests/ConvexHull.cpp
    Tests/ConveyorBelt.cpp
    Tests/DistanceTest.cpp
    Tests/Dominos.cpp
    Tests/DumpShell.cpp
    Tests/EdgeShapes.cpp
    Tests/EdgeTest.cpp
    Tests/FifteenPuzzle.cpp
    Tests/Gears.cpp
    Tests/HalfPipe.cpp
    Tests/HeavyOnLight.cpp
    Tests/HeavyOnLightTwo.cpp
    Tests/JointsTest.cpp
    Tests/Mobile.cpp
    Tests/MobileBalanced.cpp
    Tests/MotorJoint.cpp
    Tests/MotorJoint2.cpp
    Tests/NewtonsCradle.cpp
    Tests/OneSidedPlatform.cpp
    Tests/Orbiter.cpp
    Tests/Pinball.cpp
    Tests/PolyCollision.cpp
    Tests/PolyShapes.cpp
    Tests/Prismatic.cpp
    Tests/Pulleys.cpp
    Tests/Pyramid.cpp
    Tests/RayCast.cpp
    Tests/Revolute.cpp
    Tests/RopeJoint.cpp
    Tests/SensorTest.cpp
    Tests/ShapeEditing.cpp
    Tests/SliderCrank.cpp
    Tests/SolarSystem.cpp
    Tests/SphereStack.cpp
    Tests/SpinningCircle.cpp
    Tests/TheoJansen.cpp
    Tests/Tiles.cpp
    Tests/TimeOfImpact.cpp
    Tests/Tumbler.cpp
    Tests/VaryingFriction.cpp
    Tests/VaryingRestitution.cpp
    Tests/VerticalStack.cpp
    Tests/Web.cpp
    Tests/iforce2d_TopdownCar.cpp
    Tests/iforce2d_Trajectories.cpp
)

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

message(STATUS "Setting link directories")

add_executable(Testbed
    ${Testbed_Framework_SRCS}
    ${IMGUI_PREFIXED_SRCS}
    ${Testbed_Tests_SRCS}
)

target_link_libraries(Testbed PlayRho::PlayRho
    ${GLFW3_LIBRARY}
    ${GLEW_LIBRARIES}
    ${ADDITIONAL_LIBRARIES}
    ${OPENGL_LIBRARIES}
)

message(STATUS "Including header directories")
# Note: include BINARY_DIR and SOURCE_DIR before others to ensure most recent PlayRho headers
target_include_directories(Testbed PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/../>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../>"
    ${IMGUI_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIR}
    ${GLEW_INCLUDE_DIR}
    ${GLFW3_INCLUDE_DIR}
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

if(PLAYRHO_INSTALL)
    include(GNUInstallDirs)
    install(TARGETS Testbed
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        COMPONENT Applications)
endif()
