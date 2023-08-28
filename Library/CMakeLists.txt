if(NOT PLAYRHO_REAL_TYPE)
	set(PLAYRHO_REAL_TYPE float)
endif()
message(STATUS "PLAYRHO_REAL_TYPE=${PLAYRHO_REAL_TYPE}")

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

set(PLAYRHO_INCLUDE_PREFIX "include/playrho/")
set(PLAYRHO_SOURCE_PREFIX "source/playrho/")

file(GLOB PLAYRHO_General_SRCS "${PLAYRHO_SOURCE_PREFIX}*.cpp")
file(GLOB PLAYRHO_General_HDRS "${PLAYRHO_INCLUDE_PREFIX}*.hpp")

configure_file("${PLAYRHO_INCLUDE_PREFIX}Defines.hpp.in" "${PLAYRHO_INCLUDE_PREFIX}Defines.hpp" @ONLY)
configure_file("${PLAYRHO_INCLUDE_PREFIX}Real.hpp.in" "${PLAYRHO_INCLUDE_PREFIX}Real.hpp" @ONLY)
set(PLAYRHO_GeneralCfg_HDRS
	"${CMAKE_CURRENT_BINARY_DIR}/${PLAYRHO_INCLUDE_PREFIX}Defines.hpp"
	"${CMAKE_CURRENT_BINARY_DIR}/${PLAYRHO_INCLUDE_PREFIX}Real.hpp"
	)

set(PLAYRHO_PMR_HDRS
	"${PLAYRHO_INCLUDE_PREFIX}pmr/MemoryResource.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}pmr/PoolMemoryResource.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}pmr/StatsResource.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}pmr/ThreadLocalAllocator.hpp"
	)
set(PLAYRHO_PMR_SRCS
	"${PLAYRHO_SOURCE_PREFIX}pmr/MemoryResource.cpp"
	"${PLAYRHO_SOURCE_PREFIX}pmr/PoolMemoryResource.cpp"
	"${PLAYRHO_SOURCE_PREFIX}pmr/StatsResource.cpp"
	)
set(PLAYRHO_D2_HDRS
	"${PLAYRHO_INCLUDE_PREFIX}d2/AABB.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/Acceleration.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/Body.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/BodyConf.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/BodyConstraint.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/ChainShapeConf.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/ClipList.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/CodeDumper.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/Compositor.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/ContactImpulsesList.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/ContactSolver.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/DiskShapeConf.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/Distance.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/DistanceConf.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/DistanceJointConf.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/DistanceProxy.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/DynamicTree.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/DynamicTreeData.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/EdgeShapeConf.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/FrictionJointConf.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/GearJointConf.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/IndexPair.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/Joint.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/JointConf.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/LimitState.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/Manifold.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/MassData.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/MotorJointConf.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/MultiShapeConf.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/PointStates.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/PolygonShapeConf.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/Position.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/PositionConstraint.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/PositionSolverManifold.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/PrismaticJointConf.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/PulleyJointConf.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/RayCastInput.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/RayCastOutput.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/RevoluteJointConf.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/RopeJointConf.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/SeparationScenario.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/Shape.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/ShapeConf.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/ShapeSeparation.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/Simplex.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/SimplexEdge.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/Sweep.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/TargetJointConf.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/TimeOfImpact.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/Transformation.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/UnitVec.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/Velocity.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/VelocityConstraint.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/VertexSet.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/WeldJointConf.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/WheelJointConf.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/World.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/WorldBody.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/WorldConf.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/WorldContact.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/WorldImpl.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/WorldImplBody.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/WorldImplContact.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/WorldImplJoint.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/WorldImplMisc.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/WorldImplShape.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/WorldJoint.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/WorldManifold.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/WorldMisc.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/WorldShape.hpp"
	)
set(PLAYRHO_D2_SRCS
	"${PLAYRHO_SOURCE_PREFIX}d2/AABB.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/Body.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/BodyConf.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/ChainShapeConf.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/ClipList.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/CodeDumper.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/Compositor.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/ContactImpulsesList.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/ContactSolver.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/DiskShapeConf.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/Distance.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/DistanceConf.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/DistanceJointConf.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/DistanceProxy.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/DynamicTree.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/EdgeShapeConf.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/FrictionJointConf.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/GearJointConf.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/Joint.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/JointConf.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/LimitState.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/Manifold.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/MassData.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/MotorJointConf.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/MultiShapeConf.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/PointStates.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/PolygonShapeConf.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/PositionSolverManifold.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/PrismaticJointConf.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/PulleyJointConf.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/RayCastOutput.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/RevoluteJointConf.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/RopeJointConf.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/SeparationScenario.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/Shape.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/ShapeSeparation.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/Simplex.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/Sweep.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/TargetJointConf.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/TimeOfImpact.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/UnitVec.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/Velocity.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/VelocityConstraint.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/WeldJointConf.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/WheelJointConf.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/World.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/WorldBody.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/WorldContact.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/WorldImpl.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/WorldImplBody.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/WorldImplContact.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/WorldImplJoint.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/WorldImplMisc.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/WorldImplShape.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/WorldJoint.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/WorldManifold.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/WorldMisc.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/WorldShape.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/WorldSpecial.cpp"
	)

set(libsrc
	${PLAYRHO_General_SRCS}
	${PLAYRHO_PMR_SRCS}
	${PLAYRHO_D2_SRCS}
)

if(PLAYRHO_BUILD_SHARED)
	set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
	add_library(PlayRho SHARED ${libsrc})
else()
	add_library(PlayRho STATIC ${libsrc})
	target_compile_definitions(PlayRho PUBLIC -DPLAYRHO_STATIC_DEFINE)
endif()
add_library(PlayRho::PlayRho ALIAS PlayRho)

if (${PLAYRHO_ENABLE_COVERAGE} AND "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
	message(STATUS "lib: Adding definitions for coverage analysis.")
	add_definitions(--coverage)
endif()

include(GenerateExportHeader)
generate_export_header(PlayRho EXPORT_FILE_NAME "${PLAYRHO_INCLUDE_PREFIX}Export.hpp")

target_compile_features(PlayRho PUBLIC cxx_std_17)
set_target_properties(PlayRho PROPERTIES
	OUTPUT_NAME "PlayRho"
	VERSION ${PlayRho_VERSION}
	SOVERSION ${PlayRho_VERSION_MAJOR})
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

# These are used to create visual studio folders.
source_group(Include FILES ${PLAYRHO_General_HDRS} ${PLAYRHO_GeneralCfg_HDRS})
source_group(General FILES ${PLAYRHO_General_SRCS} ${PLAYRHO_GeneralCfg_HDRS} ${PLAYRHO_General_HDRS})

if(PLAYRHO_INSTALL)
	include(GNUInstallDirs)
	include(CMakePackageConfigHelpers)

	# install headers
	install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PLAYRHO_INCLUDE_PREFIX}Export.hpp
		DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/playrho
		COMPONENT Library)
	install(FILES ${PLAYRHO_General_HDRS}
		DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/playrho
		COMPONENT Library)
	install(FILES ${PLAYRHO_GeneralCfg_HDRS}
		DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/playrho
		COMPONENT Library)
	install(FILES ${PLAYRHO_PMR_HDRS}
		DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/playrho/pmr
		COMPONENT Library)
	install(FILES ${PLAYRHO_D2_HDRS}
		DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/playrho/d2
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
