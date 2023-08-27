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

set(PLAYRHO_INCLUDE_PREFIX "include/PlayRho/")
set(PLAYRHO_SOURCE_PREFIX "source/PlayRho/")

file(GLOB PLAYRHO_Collision_SRCS "${PLAYRHO_SOURCE_PREFIX}Collision/*.cpp")
file(GLOB PLAYRHO_Collision_HDRS "${PLAYRHO_INCLUDE_PREFIX}Collision/*.hpp")
file(GLOB PLAYRHO_Shapes_SRCS "${PLAYRHO_SOURCE_PREFIX}Collision/Shapes/*.cpp")
file(GLOB PLAYRHO_Shapes_HDRS "${PLAYRHO_INCLUDE_PREFIX}Collision/Shapes/*.hpp")
file(GLOB PLAYRHO_Dynamics_SRCS "${PLAYRHO_SOURCE_PREFIX}Dynamics/*.cpp")
file(GLOB PLAYRHO_Dynamics_HDRS "${PLAYRHO_INCLUDE_PREFIX}Dynamics/*.hpp")
file(GLOB PLAYRHO_Joints_SRCS "${PLAYRHO_SOURCE_PREFIX}Dynamics/Joints/*.cpp")
file(GLOB PLAYRHO_Joints_HDRS "${PLAYRHO_INCLUDE_PREFIX}Dynamics/Joints/*.hpp")
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
	"${PLAYRHO_INCLUDE_PREFIX}d2/Acceleration.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/BodyConstraint.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/CodeDumper.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/ContactSolver.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/Position.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/PositionConstraint.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/Sweep.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/Transformation.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/UnitVec.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/Velocity.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/VelocityConstraint.hpp"
	"${PLAYRHO_INCLUDE_PREFIX}d2/VertexSet.hpp"
	)
set(PLAYRHO_D2_SRCS
	"${PLAYRHO_SOURCE_PREFIX}d2/CodeDumper.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/ContactSolver.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/PositionSolverManifold.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/Sweep.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/UnitVec.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/Velocity.cpp"
	"${PLAYRHO_SOURCE_PREFIX}d2/VelocityConstraint.cpp"
	)

set(libsrc
	${PLAYRHO_General_SRCS}
	${PLAYRHO_Joints_SRCS}
	${PLAYRHO_Dynamics_SRCS}
	${PLAYRHO_Shapes_SRCS}
	${PLAYRHO_Collision_SRCS}
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
source_group(Collision FILES ${PLAYRHO_Collision_SRCS} ${PLAYRHO_Collision_HDRS})
source_group(Collision\\Shapes FILES ${PLAYRHO_Shapes_SRCS} ${PLAYRHO_Shapes_HDRS})
source_group(Dynamics FILES ${PLAYRHO_Dynamics_SRCS} ${PLAYRHO_Dynamics_HDRS})
source_group(Dynamics\\Joints FILES ${PLAYRHO_Joints_SRCS} ${PLAYRHO_Joints_HDRS})

if(PLAYRHO_INSTALL)
	include(GNUInstallDirs)
	include(CMakePackageConfigHelpers)

	# install headers
	install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PLAYRHO_INCLUDE_PREFIX}Export.hpp
		DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/PlayRho
		COMPONENT Library)
	install(FILES ${PLAYRHO_General_HDRS}
		DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/PlayRho
		COMPONENT Library)
	install(FILES ${PLAYRHO_GeneralCfg_HDRS}
		DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/PlayRho
		COMPONENT Library)
	install(FILES ${PLAYRHO_Collision_HDRS}
		DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/PlayRho/Collision
		COMPONENT Library)
	install(FILES ${PLAYRHO_Shapes_HDRS}
		DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/PlayRho/Collision/Shapes
		COMPONENT Library)
	install(FILES ${PLAYRHO_Dynamics_HDRS}
		DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/PlayRho/Dynamics
		COMPONENT Library)
	install(FILES ${PLAYRHO_Joints_HDRS}
		DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/PlayRho/Dynamics/Joints
		COMPONENT Library)
	install(FILES ${PLAYRHO_PMR_HDRS}
		DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/PlayRho/pmr
		COMPONENT Library)
	install(FILES ${PLAYRHO_D2_HDRS}
		DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/PlayRho/d2
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