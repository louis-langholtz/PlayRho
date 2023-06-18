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

file(GLOB PLAYRHO_Collision_SRCS "Collision/*.cpp")
file(GLOB PLAYRHO_Collision_HDRS "Collision/*.hpp")
file(GLOB PLAYRHO_Shapes_SRCS "Collision/Shapes/*.cpp")
file(GLOB PLAYRHO_Shapes_HDRS "Collision/Shapes/*.hpp")
file(GLOB PLAYRHO_Dynamics_SRCS "Dynamics/*.cpp")
file(GLOB PLAYRHO_Dynamics_HDRS "Dynamics/*.hpp")
file(GLOB PLAYRHO_Contacts_SRCS "Dynamics/Contacts/*.cpp")
file(GLOB PLAYRHO_Contacts_HDRS "Dynamics/Contacts/*.hpp")
file(GLOB PLAYRHO_Joints_SRCS "Dynamics/Joints/*.cpp")
file(GLOB PLAYRHO_Joints_HDRS "Dynamics/Joints/*.hpp")

file(GLOB PLAYRHO_General_HDRS "*.hpp")
file(COPY ${PLAYRHO_General_HDRS} DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")
configure_file("Defines.hpp.in" "Defines.hpp" @ONLY) # overwrites existing Defines.hpp
file(GLOB PLAYRHO_GeneralCfg_HDRS "${CMAKE_CURRENT_BINARY_DIR}/*.hpp")

file(GLOB PLAYRHO_Common_SRCS "Common/*.cpp")
file(GLOB PLAYRHO_Common_HDRS "Common/*.hpp")
file(COPY ${PLAYRHO_Common_HDRS} DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/Common")
configure_file("Common/Real.hpp.in" "Common/Real.hpp" @ONLY) # overwrites existing Common/Real.hpp
file(GLOB PLAYRHO_CommonCfg_HDRS "${CMAKE_CURRENT_BINARY_DIR}/Common/*.hpp")

if (${PLAYRHO_ENABLE_COVERAGE} AND "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
	message(STATUS "lib: Adding definitions for coverage analysis.")
	add_definitions(--coverage)
endif()

set(libsrc
	${PLAYRHO_GeneralCfg_HDRS}
	${PLAYRHO_Joints_SRCS}
	${PLAYRHO_Joints_HDRS}
	${PLAYRHO_Contacts_SRCS}
	${PLAYRHO_Contacts_HDRS}
	${PLAYRHO_Dynamics_SRCS}
	${PLAYRHO_Dynamics_HDRS}
	${PLAYRHO_Common_SRCS}
	${PLAYRHO_CommonCfg_HDRS}
	${PLAYRHO_Shapes_SRCS}
	${PLAYRHO_Shapes_HDRS}
	${PLAYRHO_Collision_SRCS}
	${PLAYRHO_Collision_HDRS}
	${PLAYRHO_Rope_SRCS}
	${PLAYRHO_Rope_HDRS}
)

if(PLAYRHO_BUILD_SHARED)
	set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
	add_library(PlayRho SHARED ${libsrc})
else()
	add_library(PlayRho STATIC ${libsrc})
endif()
add_library(PlayRho::PlayRho ALIAS PlayRho)

include(GenerateExportHeader)
generate_export_header(PlayRho EXPORT_FILE_NAME Export.h)
if (NOT PLAYRHO_BUILD_SHARED)
	target_compile_definitions(PlayRho PUBLIC -DPLAYRHO_STATIC_DEFINE)
endif()

target_compile_features(PlayRho PUBLIC cxx_std_17)
set_target_properties(PlayRho PROPERTIES
	OUTPUT_NAME "PlayRho"
	VERSION ${PlayRho_VERSION}
	SOVERSION ${PlayRho_VERSION_MAJOR})
target_include_directories(PlayRho PUBLIC
	"$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/../>"
	"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../>"
	"$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

# These are used to create visual studio folders.
source_group(Collision FILES ${PLAYRHO_Collision_SRCS} ${PLAYRHO_Collision_HDRS})
source_group(Collision\\Shapes FILES ${PLAYRHO_Shapes_SRCS} ${PLAYRHO_Shapes_HDRS})
source_group(Common FILES ${PLAYRHO_Common_SRCS} ${PLAYRHO_CommonCfg_HDRS})
source_group(Dynamics FILES ${PLAYRHO_Dynamics_SRCS} ${PLAYRHO_Dynamics_HDRS})
source_group(Dynamics\\Contacts FILES ${PLAYRHO_Contacts_SRCS} ${PLAYRHO_Contacts_HDRS})
source_group(Dynamics\\Joints FILES ${PLAYRHO_Joints_SRCS} ${PLAYRHO_Joints_HDRS})
source_group(Include FILES ${PLAYRHO_GeneralCfg_HDRS})

if(PLAYRHO_INSTALL)
	include(GNUInstallDirs)
	include(CMakePackageConfigHelpers)

	# install headers
	install(FILES ${PLAYRHO_GeneralCfg_HDRS}
		DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/PlayRho
		COMPONENT Library)
	install(FILES ${PLAYRHO_Collision_HDRS}
		DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/PlayRho/Collision
		COMPONENT Library)
	install(FILES ${PLAYRHO_Shapes_HDRS}
		DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/PlayRho/Collision/Shapes
		COMPONENT Library)
	install(FILES ${PLAYRHO_CommonCfg_HDRS}
		DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/PlayRho/Common
		COMPONENT Library)
	install(FILES ${PLAYRHO_Dynamics_HDRS}
		DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/PlayRho/Dynamics
		COMPONENT Library)
	install(FILES ${PLAYRHO_Contacts_HDRS}
		DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/PlayRho/Dynamics/Contacts
		COMPONENT Library)
	install(FILES ${PLAYRHO_Joints_HDRS}
		DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/PlayRho/Dynamics/Joints
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
