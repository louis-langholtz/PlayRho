# CMake configuration file for UnitTests subdirectory.
# Top level docs for 3.1.3 at: https://cmake.org/cmake/help/v3.1/
# Commands herein described at: https://cmake.org/cmake/help/v3.1/manual/cmake-commands.7.html

message(STATUS "PROJECT_VERSION=${PROJECT_VERSION}")
if(NOT PlayRho_VERSION_MAJOR)
    set(PlayRho_VERSION_MAJOR 0)
endif()
if(NOT PlayRho_VERSION_MINOR)
    set(PlayRho_VERSION_MINOR 0)
endif()
if(NOT PlayRho_VERSION_PATCH)
    set(PlayRho_VERSION_PATCH 0)
endif()
message(STATUS "UnitTests PlayRho_VERSION_MAJOR=${PlayRho_VERSION_MAJOR}")
message(STATUS "UnitTests PlayRho_VERSION_MINOR=${PlayRho_VERSION_MINOR}")
message(STATUS "UnitTests PlayRho_VERSION_PATCH=${PlayRho_VERSION_PATCH}")
add_compile_definitions(PLAYRHO_VERSION_MAJOR=${PlayRho_VERSION_MAJOR})
add_compile_definitions(PLAYRHO_VERSION_MINOR=${PlayRho_VERSION_MINOR})
add_compile_definitions(PLAYRHO_VERSION_PATCH=${PlayRho_VERSION_PATCH})

# Hides options.
mark_as_advanced(FORCE BUILD_GMOCK BUILD_GTEST)
mark_as_advanced(FORCE INSTALL_GMOCK INSTALL_GTEST)
mark_as_advanced(FORCE gmock_build_tests)
mark_as_advanced(FORCE gtest_build_samples gtest_build_tests gtest_disable_pthreads gtest_hide_internal_symbols)
mark_as_advanced(FORCE gtest_force_shared_crt)
mark_as_advanced(FORCE CMAKE_DEBUG_POSTFIX)

set(BUILD_GMOCK ON)
set(INSTALL_GTEST OFF CACHE BOOL "Enable installation of googletest.")
set(INSTALL_GMOCK OFF CACHE BOOL "Enable installation of googlemock.")
set(gtest_hide_internal_symbols OFF CACHE BOOL "")

# Don't build gtest shared to ensure GTEST_FLAG_SET symbols can be resolved
set(BUILD_SHARED_LIBS OFF)

# Prevent overriding the parent project's compiler/linker settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/googletest/googletest")
    add_subdirectory(googletest EXCLUDE_FROM_ALL)
else()
    include(FetchContent)
    message(STATUS "Attempting to fetch googletest source code.")
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG 391ce627def20c1e8a54d10b12949b15086473dd
    )
    FetchContent_MakeAvailable(googletest)
endif()

set(UnitTest_HDRS
    UnitTests.hpp
)
set(UnitTest_SRCS
    AABB.cpp
    AabbTreeWorld.cpp
    Acceleration.cpp
    Angle.cpp
    ArrayList.cpp
    BaseShapeConf.cpp
    BlockAllocator.cpp
    Body.cpp
    BodyConf.cpp
    BodyConstraint.cpp
    BodyID.cpp
    BodyType.cpp
    ChainShape.cpp
    Checked.cpp
    CollideShapes.cpp
    Compositor.cpp
    ConstraintSolverConf.cpp
    Contact.cpp
    ContactFeature.cpp
    ContactImpulsesList.cpp
    ContactSolver.cpp
    DiskShape.cpp
    Distance.cpp
    DistanceJoint.cpp
    DistanceProxy.cpp
    Dump.cpp
    DynamicMemory.cpp
    DynamicTree.cpp
    EdgeShape.cpp
    Epsilon.cpp
    Filter.cpp
    FlagGuard.cpp
    FrictionJoint.cpp
    GearJoint.cpp
    GrowableStack.cpp
    IndexPair.cpp
    Interval.cpp
    Island.cpp
    IslandStats.cpp
    Joint.cpp
    Manifold.cpp
    MassData.cpp
    Mat22.cpp
    Mat33.cpp
    Math.cpp
    MemoryResource.cpp
    MotorJoint.cpp
    MultiShape.cpp
    ObjectPool.cpp
    PolygonShape.cpp
    PoolMemoryResource.cpp
    Position.cpp
    PositionConstraint.cpp
    PositionSolverManifold.cpp
    PrismaticJoint.cpp
    PulleyJoint.cpp
    RayCastOutput.cpp
    Real.cpp
    RevoluteJoint.cpp
    RopeJoint.cpp
    SeparationScenario.cpp
    Shape.cpp
    Simplex.cpp
    SimplexEdge.cpp
    StackAllocator.cpp
    StatsResource.cpp
    StepConf.cpp
    StepStats.cpp
    Sweep.cpp
    TargetJoint.cpp
    ThreadLocalAllocator.cpp
    TimeOfImpact.cpp
    Transformation.cpp
    TypeInfo.cpp
    UnitVec.cpp
    Units.cpp
    Vec2.cpp
    Vec3.cpp
    Vector.cpp
    Velocity.cpp
    VelocityConstraint.cpp
    Version.cpp
    VertexSet.cpp
    WeldJoint.cpp
    WheelJoint.cpp
    World.cpp
    WorldBody.cpp
    WorldConf.cpp
    WorldContact.cpp
    WorldFixture.cpp
    WorldJoint.cpp
    WorldManifold.cpp
    WorldShape.cpp
    double.cpp
    float.cpp
    forward_list.cpp
    functional.cpp
    main.cpp
    polymorphic_allocator.cpp
)

# Add an executable to the project using specified source files.
# See details at: https://cmake.org/cmake/help/v3.1/command/add_executable.html
add_executable(UnitTests ${UnitTest_SRCS} ${UnitTest_HDRS})

# enable code coverage generation (only with GCC for now)
if(${PLAYRHO_ENABLE_COVERAGE})
    if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
        # Use -ftest-coverage to generate .gcno notes files.
        # Use -fprofile-arcs to generate .gcda count data files when resulting objects are run.
        message(STATUS "UnitTests: Adding definitions for coverage analysis.")
        add_definitions(-fprofile-arcs -ftest-coverage)
    endif()
endif()

# Link a target to given libraries.
# See details at: https://cmake.org/cmake/help/v3.1/command/target_link_libraries.html
target_link_libraries(UnitTests PlayRho::PlayRho GTest::gtest)

# link with coverage library
if(${PLAYRHO_ENABLE_COVERAGE})
    if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
        # Use -ftest-coverage to generate .gcno notes files.
        # Use -fprofile-arcs to generate .gcda count data files when resulting objects are run.
        target_link_libraries(UnitTests -fprofile-arcs -ftest-coverage)
    endif()
endif()

include(GoogleTest)
gtest_discover_tests(UnitTests)

if(PLAYRHO_INSTALL)
    include(GNUInstallDirs)
    install(TARGETS UnitTests
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        COMPONENT Applications)
endif()
