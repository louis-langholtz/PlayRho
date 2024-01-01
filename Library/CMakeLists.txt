if(PLAYRHO_REAL_TYPE)
    if(PLAYRHO_REAL_PROJECT)
        if(PLAYRHO_REAL_GIT_REPOSITORY)
            include(FetchContent)
            message(STATUS "Attempting to fetch PLAYRHO_REAL_PROJECT \"${PLAYRHO_REAL_PROJECT}\", for PLAYRHO_REAL_TYPE support")
            if(PLAYRHO_REAL_GIT_TAG)
                FetchContent_Declare(${PLAYRHO_REAL_PROJECT}
                    GIT_REPOSITORY ${PLAYRHO_REAL_GIT_REPOSITORY}
                    GIT_TAG ${PLAYRHO_REAL_GIT_TAG}
                )
            else()
                # Without GIT_TAG specified, defaults to fetching "master" branch of repo
                FetchContent_Declare(${PLAYRHO_REAL_PROJECT}
                    GIT_REPOSITORY ${PLAYRHO_REAL_GIT_REPOSITORY}
                )
            endif()
            FetchContent_MakeAvailable(${PLAYRHO_REAL_PROJECT})
            set(PLAYRHO_REAL_SOURCE_DIR "${${PLAYRHO_REAL_PROJECT}_SOURCE_DIR}")
            message(STATUS "Fetched, PLAYRHO_REAL_SOURCE_DIR=${PLAYRHO_REAL_SOURCE_DIR}")
        else()
            message(STATUS "Attempting to find PLAYRHO_REAL_PROJECT \"${PLAYRHO_REAL_PROJECT}\", for PLAYRHO_REAL_TYPE support")
            find_package(${PLAYRHO_REAL_PROJECT} REQUIRED)
            message(STATUS "Found, CMake config for PLAYRHO_REAL_PROJECT \"${PLAYRHO_REAL_PROJECT}\" at \"${${PLAYRHO_REAL_PROJECT}_CONFIG}\"")
        endif()
    endif()
else()
    set(PLAYRHO_REAL_TYPE float)
endif()
message(STATUS "PLAYRHO_REAL_TYPE=${PLAYRHO_REAL_TYPE}")

if(NOT PLAYRHO_REAL_LINEARSLOP)
    set(PLAYRHO_REAL_LINEARSLOP "0.005_m")
endif()
message(STATUS "PLAYRHO_REAL_LINEARSLOP=${PLAYRHO_REAL_LINEARSLOP}")

if(NOT PLAYRHO_REAL_MINVERTEXRADIUS)
    set(PLAYRHO_REAL_MINVERTEXRADIUS "0.01_m")
endif()
message(STATUS "PLAYRHO_REAL_MINVERTEXRADIUS=${PLAYRHO_REAL_MINVERTEXRADIUS}")

if(NOT PLAYRHO_REAL_MAXVERTEXRADIUS)
    set(PLAYRHO_REAL_MAXVERTEXRADIUS "255_m")
endif()
message(STATUS "PLAYRHO_REAL_MAXVERTEXRADIUS=${PLAYRHO_REAL_MAXVERTEXRADIUS}")

if(PLAYRHO_REAL_INCLUDE)
    foreach(line IN LISTS PLAYRHO_REAL_INCLUDE)
        string(APPEND PLAYRHO_REAL_INCLUDES "${line}\n")
    endforeach()
    message(STATUS "PLAYRHO_REAL_INCLUDES=${PLAYRHO_REAL_INCLUDES}")
endif()

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
message(STATUS "PlayRho_VERSION=${PlayRho_VERSION}")
message(STATUS "PlayRho_VERSION_MAJOR=${PlayRho_VERSION_MAJOR}")
message(STATUS "PlayRho_VERSION_MINOR=${PlayRho_VERSION_MINOR}")
message(STATUS "PlayRho_VERSION_PATCH=${PlayRho_VERSION_PATCH}")
add_compile_definitions(PLAYRHO_VERSION_MAJOR=${PlayRho_VERSION_MAJOR})
add_compile_definitions(PLAYRHO_VERSION_MINOR=${PlayRho_VERSION_MINOR})
add_compile_definitions(PLAYRHO_VERSION_PATCH=${PlayRho_VERSION_PATCH})

# /bin/ls -1 include/playrho/*.hpp
set(PLAYRHO_General_HDRS
    include/playrho/ArrayList.hpp
    include/playrho/BlockAllocator.hpp
    include/playrho/BodyID.hpp
    include/playrho/BodyShapeFunction.hpp
    include/playrho/BodyType.hpp
    include/playrho/ConstraintSolverConf.hpp
    include/playrho/Contact.hpp
    include/playrho/ContactFeature.hpp
    include/playrho/ContactFunction.hpp
    include/playrho/ContactID.hpp
    include/playrho/ContactKey.hpp
    include/playrho/Contactable.hpp
    include/playrho/Defines.hpp
    include/playrho/Doxygen.hpp
    include/playrho/DynamicMemory.hpp
    include/playrho/Filter.hpp
    include/playrho/Finite.hpp
    include/playrho/FlagGuard.hpp
    include/playrho/GrowableStack.hpp
    include/playrho/Interval.hpp
    include/playrho/Intervals.hpp
    include/playrho/InvalidArgument.hpp
    include/playrho/Island.hpp
    include/playrho/IslandStats.hpp
    include/playrho/JointFunction.hpp
    include/playrho/JointID.hpp
    include/playrho/KeyedContactID.hpp
    include/playrho/LengthError.hpp
    include/playrho/LimitState.hpp
    include/playrho/Math.hpp
    include/playrho/Matrix.hpp
    include/playrho/MovementConf.hpp
    include/playrho/Negative.hpp
    include/playrho/NonNegative.hpp
    include/playrho/NonPositive.hpp
    include/playrho/NonZero.hpp
    include/playrho/ObjectPool.hpp
    include/playrho/OutOfRange.hpp
    include/playrho/PointState.hpp
    include/playrho/Positive.hpp
    include/playrho/RayCastOpcode.hpp
    include/playrho/Real.hpp
    include/playrho/RealConstants.hpp
    include/playrho/Settings.hpp
    include/playrho/ShapeFunction.hpp
    include/playrho/ShapeID.hpp
    include/playrho/Span.hpp
    include/playrho/StackAllocator.hpp
    include/playrho/StepConf.hpp
    include/playrho/StepStats.hpp
    include/playrho/Templates.hpp
    include/playrho/ToiConf.hpp
    include/playrho/ToiOutput.hpp
    include/playrho/TypeInfo.hpp
    include/playrho/UnitInterval.hpp
    include/playrho/Units.hpp
    include/playrho/Vector.hpp
    include/playrho/Vector2.hpp
    include/playrho/Vector3.hpp
    include/playrho/Version.hpp
    include/playrho/WiderType.hpp
    include/playrho/WrongState.hpp
    include/playrho/ZeroToUnderOne.hpp
    include/playrho/to_underlying.hpp
)

# /bin/ls -1 source/playrho/*.cpp
set(PLAYRHO_General_SRCS
    source/playrho/BlockAllocator.cpp
    source/playrho/ConstraintSolverConf.cpp
    source/playrho/Contact.cpp
    source/playrho/DynamicMemory.cpp
    source/playrho/Island.cpp
    source/playrho/LimitState.cpp
    source/playrho/Math.cpp
    source/playrho/MovementConf.cpp
    source/playrho/StackAllocator.cpp
    source/playrho/StepConf.cpp
    source/playrho/StepStats.cpp
    source/playrho/ToiConf.cpp
    source/playrho/ToiOutput.cpp
    source/playrho/Version.cpp
)

# /bin/ls -1 include/playrho/detail/*.hpp
set(PLAYRHO_DETAIL_HDRS
    include/playrho/detail/AABB.hpp
    include/playrho/detail/Checked.hpp
    include/playrho/detail/CheckedMath.hpp
    include/playrho/detail/FiniteChecker.hpp
    include/playrho/detail/IndexingNamedType.hpp
    include/playrho/detail/NegativeChecker.hpp
    include/playrho/detail/NonNegativeChecker.hpp
    include/playrho/detail/NonPositiveChecker.hpp
    include/playrho/detail/NonZeroChecker.hpp
    include/playrho/detail/PositiveChecker.hpp
    include/playrho/detail/RayCastInput.hpp
    include/playrho/detail/Templates.hpp
    include/playrho/detail/TypeInfo.hpp
    include/playrho/detail/UnitIntervalChecker.hpp
    include/playrho/detail/Wider.hpp
    include/playrho/detail/ZeroToUnderOneChecker.hpp
    include/playrho/detail/underlying_type.hpp
)

configure_file(include/playrho/Defines.hpp.in include/playrho/Defines.hpp @ONLY)
configure_file(include/playrho/Real.hpp.in include/playrho/Real.hpp @ONLY)
configure_file(include/playrho/Settings.hpp.in include/playrho/Settings.hpp @ONLY)
set(PLAYRHO_GeneralCfg_HDRS
    "${CMAKE_CURRENT_BINARY_DIR}/include/playrho/Defines.hpp"
    "${CMAKE_CURRENT_BINARY_DIR}/include/playrho/Real.hpp"
    "${CMAKE_CURRENT_BINARY_DIR}/include/playrho/Settings.hpp"
)

# /bin/ls -1 include/playrho/pmr/*.hpp
set(PLAYRHO_PMR_HDRS
    include/playrho/pmr/MemoryResource.hpp
    include/playrho/pmr/PoolMemoryResource.hpp
    include/playrho/pmr/StatsResource.hpp
    include/playrho/pmr/ThreadLocalAllocator.hpp
)
# /bin/ls -1 source/playrho/pmr/*.cpp
set(PLAYRHO_PMR_SRCS
    source/playrho/pmr/MemoryResource.cpp
    source/playrho/pmr/PoolMemoryResource.cpp
    source/playrho/pmr/StatsResource.cpp
)
# /bin/ls -1 include/playrho/d2/*.hpp
set(PLAYRHO_D2_HDRS
    include/playrho/d2/AABB.hpp
    include/playrho/d2/AabbTreeWorld.hpp
    include/playrho/d2/Acceleration.hpp
    include/playrho/d2/BasicAPI.hpp
    include/playrho/d2/Body.hpp
    include/playrho/d2/BodyConf.hpp
    include/playrho/d2/BodyConstraint.hpp
    include/playrho/d2/ChainShapeConf.hpp
    include/playrho/d2/ClipList.hpp
    include/playrho/d2/CodeDumper.hpp
    include/playrho/d2/ContactImpulsesFunction.hpp
    include/playrho/d2/ContactImpulsesList.hpp
    include/playrho/d2/ContactManifoldFunction.hpp
    include/playrho/d2/ContactSolver.hpp
    include/playrho/d2/ConvexHull.hpp
    include/playrho/d2/DiskShapeConf.hpp
    include/playrho/d2/Distance.hpp
    include/playrho/d2/DistanceConf.hpp
    include/playrho/d2/DistanceJointConf.hpp
    include/playrho/d2/DistanceProxy.hpp
    include/playrho/d2/DynamicTree.hpp
    include/playrho/d2/DynamicTreeData.hpp
    include/playrho/d2/EdgeShapeConf.hpp
    include/playrho/d2/FrictionJointConf.hpp
    include/playrho/d2/GearJointConf.hpp
    include/playrho/d2/IndexPair.hpp
    include/playrho/d2/Joint.hpp
    include/playrho/d2/JointConf.hpp
    include/playrho/d2/Manifold.hpp
    include/playrho/d2/MassData.hpp
    include/playrho/d2/Math.hpp
    include/playrho/d2/MotorJointConf.hpp
    include/playrho/d2/MultiShapeConf.hpp
    include/playrho/d2/NgonWithFwdNormals.hpp
    include/playrho/d2/PointStates.hpp
    include/playrho/d2/PolygonShapeConf.hpp
    include/playrho/d2/Position.hpp
    include/playrho/d2/PositionConstraint.hpp
    include/playrho/d2/PositionSolution.hpp
    include/playrho/d2/PositionSolverManifold.hpp
    include/playrho/d2/PrismaticJointConf.hpp
    include/playrho/d2/PulleyJointConf.hpp
    include/playrho/d2/RayCastInput.hpp
    include/playrho/d2/RayCastOutput.hpp
    include/playrho/d2/RevoluteJointConf.hpp
    include/playrho/d2/RopeJointConf.hpp
    include/playrho/d2/SeparationScenario.hpp
    include/playrho/d2/Shape.hpp
    include/playrho/d2/ShapeConf.hpp
    include/playrho/d2/ShapeSeparation.hpp
    include/playrho/d2/Simplex.hpp
    include/playrho/d2/SimplexEdge.hpp
    include/playrho/d2/Sweep.hpp
    include/playrho/d2/TargetJointConf.hpp
    include/playrho/d2/TimeOfImpact.hpp
    include/playrho/d2/Transformation.hpp
    include/playrho/d2/UnitVec.hpp
    include/playrho/d2/Velocity.hpp
    include/playrho/d2/VelocityConstraint.hpp
    include/playrho/d2/VertexSet.hpp
    include/playrho/d2/WeldJointConf.hpp
    include/playrho/d2/WheelJointConf.hpp
    include/playrho/d2/World.hpp
    include/playrho/d2/WorldBody.hpp
    include/playrho/d2/WorldConf.hpp
    include/playrho/d2/WorldContact.hpp
    include/playrho/d2/WorldJoint.hpp
    include/playrho/d2/WorldManifold.hpp
    include/playrho/d2/WorldMisc.hpp
    include/playrho/d2/WorldShape.hpp
)
# /bin/ls -1 source/playrho/d2/*.cpp
set(PLAYRHO_D2_SRCS
    source/playrho/d2/AABB.cpp
    source/playrho/d2/AabbTreeWorld.cpp
    source/playrho/d2/Body.cpp
    source/playrho/d2/BodyConf.cpp
    source/playrho/d2/ChainShapeConf.cpp
    source/playrho/d2/ClipList.cpp
    source/playrho/d2/CodeDumper.cpp
    source/playrho/d2/ContactImpulsesList.cpp
    source/playrho/d2/ContactSolver.cpp
    source/playrho/d2/ConvexHull.cpp
    source/playrho/d2/DiskShapeConf.cpp
    source/playrho/d2/Distance.cpp
    source/playrho/d2/DistanceConf.cpp
    source/playrho/d2/DistanceJointConf.cpp
    source/playrho/d2/DistanceProxy.cpp
    source/playrho/d2/DynamicTree.cpp
    source/playrho/d2/EdgeShapeConf.cpp
    source/playrho/d2/FrictionJointConf.cpp
    source/playrho/d2/GearJointConf.cpp
    source/playrho/d2/Joint.cpp
    source/playrho/d2/JointConf.cpp
    source/playrho/d2/Manifold.cpp
    source/playrho/d2/MassData.cpp
    source/playrho/d2/Math.cpp
    source/playrho/d2/MotorJointConf.cpp
    source/playrho/d2/MultiShapeConf.cpp
    source/playrho/d2/PointStates.cpp
    source/playrho/d2/PolygonShapeConf.cpp
    source/playrho/d2/PositionSolverManifold.cpp
    source/playrho/d2/PrismaticJointConf.cpp
    source/playrho/d2/PulleyJointConf.cpp
    source/playrho/d2/RayCastOutput.cpp
    source/playrho/d2/RevoluteJointConf.cpp
    source/playrho/d2/RopeJointConf.cpp
    source/playrho/d2/SeparationScenario.cpp
    source/playrho/d2/Shape.cpp
    source/playrho/d2/ShapeSeparation.cpp
    source/playrho/d2/Simplex.cpp
    source/playrho/d2/Sweep.cpp
    source/playrho/d2/TargetJointConf.cpp
    source/playrho/d2/TimeOfImpact.cpp
    source/playrho/d2/UnitVec.cpp
    source/playrho/d2/Velocity.cpp
    source/playrho/d2/VelocityConstraint.cpp
    source/playrho/d2/WeldJointConf.cpp
    source/playrho/d2/WheelJointConf.cpp
    source/playrho/d2/World.cpp
    source/playrho/d2/WorldBody.cpp
    source/playrho/d2/WorldContact.cpp
    source/playrho/d2/WorldJoint.cpp
    source/playrho/d2/WorldManifold.cpp
    source/playrho/d2/WorldMisc.cpp
    source/playrho/d2/WorldShape.cpp
)

set(PLAYRHO_D2_DETAIL_HDRS
    include/playrho/d2/detail/JointConcept.hpp
    include/playrho/d2/detail/JointModel.hpp
    include/playrho/d2/detail/ShapeConcept.hpp
    include/playrho/d2/detail/ShapeModel.hpp
    include/playrho/d2/detail/WorldConcept.hpp
    include/playrho/d2/detail/WorldModel.hpp
)

set(PLAYRHO_D2_PART_HDRS
    include/playrho/d2/part/Compositor.hpp
)
set(PLAYRHO_D2_PART_SRCS
    source/playrho/d2/part/Compositor.cpp
)

set(libhdr
    ${PLAYRHO_General_HDRS}
    ${PLAYRHO_DETAIL_HDRS}
    ${PLAYRHO_PMR_HDRS}
    ${PLAYRHO_D2_HDRS}
    ${PLAYRHO_D2_PART_HDRS}
    )
set(libsrc
    ${PLAYRHO_General_SRCS}
    ${PLAYRHO_PMR_SRCS}
    ${PLAYRHO_D2_SRCS}
    ${PLAYRHO_D2_PART_SRCS}
)

if(PLAYRHO_BUILD_SHARED)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
    add_library(PlayRho SHARED ${libsrc} ${libhdr})
else()
    add_library(PlayRho STATIC ${libsrc} ${libhdr})
    target_compile_definitions(PlayRho PUBLIC -DPLAYRHO_STATIC_DEFINE)
endif()
add_library(PlayRho::PlayRho ALIAS PlayRho)

if (${PLAYRHO_ENABLE_COVERAGE} AND "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    message(STATUS "lib: Adding definitions for coverage analysis.")
    add_definitions(--coverage)
endif()

include(GenerateExportHeader)
generate_export_header(PlayRho EXPORT_FILE_NAME "include/playrho/Export.hpp")

target_compile_features(PlayRho PUBLIC cxx_std_17)
set_target_properties(PlayRho PROPERTIES
    OUTPUT_NAME "PlayRho"
    VERSION ${PlayRho_VERSION}
    SOVERSION ${PlayRho_VERSION_MAJOR}
)
target_include_directories(PlayRho PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include/>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

if (PLAYRHO_ENABLE_BOOST_UNITS)
    find_package(Boost REQUIRED)
    target_include_directories(PlayRho SYSTEM PUBLIC ${Boost_INCLUDE_DIR})
    target_compile_definitions(PlayRho PUBLIC -DPLAYRHO_USE_BOOST_UNITS)
endif()

# Enable additional warnings to help ensure library code compiles clean
# For GNU, see https://gcc.gnu.org/onlinedocs/gcc/Warning-Options.html
target_compile_options(PlayRho PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic -Wundef -Wcast-align -Wconversion -Wshadow -Wdouble-promotion -Werror>
)

if(PLAYRHO_REAL_PROJECT)
    target_link_libraries(PlayRho ${PLAYRHO_REAL_PROJECT}::${PLAYRHO_REAL_PROJECT})
endif()

if(PLAYRHO_INSTALL)
    include(GNUInstallDirs)
    include(CMakePackageConfigHelpers)

    # install headers
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/include/playrho/Export.hpp
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/playrho
        COMPONENT Library)
    install(FILES ${PLAYRHO_General_HDRS}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/playrho
        COMPONENT Library)
    install(FILES ${PLAYRHO_GeneralCfg_HDRS}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/playrho
        COMPONENT Library)
    install(FILES ${PLAYRHO_DETAIL_HDRS}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/playrho/detail
        COMPONENT Library)
    install(FILES ${PLAYRHO_PMR_HDRS}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/playrho/pmr
        COMPONENT Library)
    install(FILES ${PLAYRHO_D2_HDRS}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/playrho/d2
        COMPONENT Library)
    install(FILES ${PLAYRHO_D2_DETAIL_HDRS}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/playrho/d2/detail
        COMPONENT Library)
    install(FILES ${PLAYRHO_D2_PART_HDRS}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/playrho/d2/part
        COMPONENT Library)

    # install libraries
    install(TARGETS PlayRho
        EXPORT PlayRho-targets
        COMPONENT Library
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )

    # install build system hooks for third-party apps
    install(EXPORT PlayRho-targets
        FILE PlayRho-targets.cmake
        NAMESPACE PlayRho::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/PlayRho)

    set(PLAYRHO_INCLUDE_DIR ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR})
    set(PLAYRHO_INCLUDE_DIRS ${PLAYRHO_INCLUDE_DIR} )
    set(PLAYRHO_LIBRARY_DIRS ${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR})
    set(PLAYRHO_LIBRARY PlayRho)
    set(PLAYRHO_LIBRARIES ${PLAYRHO_LIBRARY})
    configure_package_config_file(PlayRhoConfig.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/PlayRhoConfig.cmake
        INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/PlayRho/PlayRhoConfig.cmake)
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/PlayRhoConfig.cmake DESTINATION ${LIB_INSTALL_DIR}/cmake/PlayRho)
endif()
