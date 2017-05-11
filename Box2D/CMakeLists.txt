set(BOX2D_REALNUM_TYPE "float")
message("BOX2D_REALNUM_TYPE=${BOX2D_REALNUM_TYPE}")

file(REMOVE "Common/Settings.hpp")
configure_file("Common/Settings.hpp.in" "Common/Settings.hpp" @ONLY)

file(GLOB BOX2D_Collision_SRCS
	"Collision/*.cpp"
)
file(GLOB BOX2D_Collision_HDRS
	"Collision/*.hpp"
)
file(GLOB BOX2D_Shapes_SRCS
	"Collision/Shapes/*.cpp"
)
file(GLOB BOX2D_Shapes_HDRS
	"Collision/Shapes/*.hpp"
)
file(GLOB BOX2D_Common_SRCS
	"Common/*.cpp"
)
file(GLOB BOX2D_Common_HDRS
	"Common/*.hpp"
)
file(GLOB BOX2D_Dynamics_SRCS
	"Dynamics/*.cpp"
)
file(GLOB BOX2D_Dynamics_HDRS
	"Dynamics/*.hpp"
)
file(GLOB BOX2D_Contacts_SRCS
	"Dynamics/Contacts/*.cpp"
)
file(GLOB BOX2D_Contacts_HDRS
	"Dynamics/Contacts/*.hpp"
)
file(GLOB BOX2D_Joints_SRCS
	"Dynamics/Joints/*.cpp"
)
file(GLOB BOX2D_Joints_HDRS
	"Dynamics/Joints/*.hpp"
)
file(GLOB BOX2D_Rope_SRCS
	"Rope/*.cpp"
)
file(GLOB BOX2D_Rope_HDRS
	"Rope/*.hpp"
)
file(GLOB BOX2D_General_HDRS
	"*.hpp"
)
include_directories( ../ )

if(BOX2D_BUILD_SHARED)
	add_library(Box2D_shared SHARED
		${BOX2D_General_HDRS}
		${BOX2D_Joints_SRCS}
		${BOX2D_Joints_HDRS}
		${BOX2D_Contacts_SRCS}
		${BOX2D_Contacts_HDRS}
		${BOX2D_Dynamics_SRCS}
		${BOX2D_Dynamics_HDRS}
		${BOX2D_Common_SRCS}
		${BOX2D_Common_HDRS}
		${BOX2D_Shapes_SRCS}
		${BOX2D_Shapes_HDRS}
		${BOX2D_Collision_SRCS}
		${BOX2D_Collision_HDRS}
		${BOX2D_Rope_SRCS}
		${BOX2D_Rope_HDRS}
	)
	set_target_properties(Box2D_shared PROPERTIES
		OUTPUT_NAME "Box2D"
		CLEAN_DIRECT_OUTPUT 1
		VERSION ${BOX2D_VERSION}
	)
endif()

if(BOX2D_BUILD_STATIC)
	add_library(Box2D STATIC
		${BOX2D_General_HDRS}
		${BOX2D_Joints_SRCS}
		${BOX2D_Joints_HDRS}
		${BOX2D_Contacts_SRCS}
		${BOX2D_Contacts_HDRS}
		${BOX2D_Dynamics_SRCS}
		${BOX2D_Dynamics_HDRS}
		${BOX2D_Common_SRCS}
		${BOX2D_Common_HDRS}
		${BOX2D_Shapes_SRCS}
		${BOX2D_Shapes_HDRS}
		${BOX2D_Collision_SRCS}
		${BOX2D_Collision_HDRS}
		${BOX2D_Rope_SRCS}
		${BOX2D_Rope_HDRS}
	)
	set_target_properties(Box2D PROPERTIES
		CLEAN_DIRECT_OUTPUT 1
		VERSION ${BOX2D_VERSION}
	)
endif()

# These are used to create visual studio folders.
source_group(Collision FILES ${BOX2D_Collision_SRCS} ${BOX2D_Collision_HDRS})
source_group(Collision\\Shapes FILES ${BOX2D_Shapes_SRCS} ${BOX2D_Shapes_HDRS})
source_group(Common FILES ${BOX2D_Common_SRCS} ${BOX2D_Common_HDRS})
source_group(Dynamics FILES ${BOX2D_Dynamics_SRCS} ${BOX2D_Dynamics_HDRS})
source_group(Dynamics\\Contacts FILES ${BOX2D_Contacts_SRCS} ${BOX2D_Contacts_HDRS})
source_group(Dynamics\\Joints FILES ${BOX2D_Joints_SRCS} ${BOX2D_Joints_HDRS})
source_group(Include FILES ${BOX2D_General_HDRS})
source_group(Rope FILES ${BOX2D_Rope_SRCS} ${BOX2D_Rope_HDRS})

if(BOX2D_INSTALL)
	# install headers
	install(FILES ${BOX2D_General_HDRS} DESTINATION include/Box2D)
	install(FILES ${BOX2D_Collision_HDRS} DESTINATION include/Box2D/Collision)
	install(FILES ${BOX2D_Shapes_HDRS} DESTINATION include/Box2D/Collision/Shapes)
	install(FILES ${BOX2D_Common_HDRS} DESTINATION include/Box2D/Common)
	install(FILES ${BOX2D_Dynamics_HDRS} DESTINATION include/Box2D/Dynamics)
	install(FILES ${BOX2D_Contacts_HDRS} DESTINATION include/Box2D/Dynamics/Contacts)
	install(FILES ${BOX2D_Joints_HDRS} DESTINATION include/Box2D/Dynamics/Joints)
	install(FILES ${BOX2D_Rope_HDRS} DESTINATION include/Box2D/Rope)

	# install libraries
	if(BOX2D_BUILD_SHARED)
		install(TARGETS Box2D_shared EXPORT Box2D-targets
                  LIBRARY DESTINATION ${LIB_INSTALL_DIR}
                  ARCHIVE DESTINATION ${LIB_INSTALL_DIR}
                  RUNTIME DESTINATION bin)
	endif()
	if(BOX2D_BUILD_STATIC)
		install(TARGETS Box2D EXPORT Box2D-targets DESTINATION ${LIB_INSTALL_DIR})
	endif()

	# install build system hooks for third-party apps
	install(EXPORT Box2D-targets DESTINATION ${LIB_INSTALL_DIR}/Box2D)

        set (BOX2D_INCLUDE_DIR ${CMAKE_INSTALL_PREFIX}/include)
        set (BOX2D_INCLUDE_DIRS ${BOX2D_INCLUDE_DIR} )
        set (BOX2D_LIBRARY_DIRS ${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR})
        set (BOX2D_LIBRARY Box2D)
        set (BOX2D_LIBRARIES ${BOX2D_LIBRARY})
        set (BOX2D_USE_FILE ${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}/cmake/Box2D/UseBox2D.cmake)
        configure_file(Box2DConfig.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/Box2DConfig.cmake @ONLY ESCAPE_QUOTES)
        install(FILES ${CMAKE_CURRENT_BINARY_DIR}/Box2DConfig.cmake UseBox2D.cmake DESTINATION ${LIB_INSTALL_DIR}/cmake/Box2D)
endif(BOX2D_INSTALL)
