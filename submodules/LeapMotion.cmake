###
# @uathor Zachary Wartell
# @brief LeapMotion SDK V2
#
# This is Work in progress...
# Initial file copied and modified from hidapi.cmake (not sure if that was the best example to copy)
###

set(LOCAL_LEAPMOTION_SUBMODULE_RETRIEVED FALSE)

if(EXISTS "${VRPN_SOURCE_DIR}/submodules/LeapSDK/include/Leap.h")
	set(LOCAL_LEAPMOTION_SUBMODULE_RETRIEVED TRUE)
endif()

if(NOT LOCAL_LEAPMOTION_SUBMODULE_RETRIEVED AND NOT SUBPROJECT)
	message(STATUS
		"Local LeapSDK submodule not found. To download with Git, run git submodule update --init")
endif()

set(LEAPMOTION_INCLUDE_DIRS)

option_requires(VRPN_USE_LOCAL_LEAPMOTION
	"Build with LeapMotion code from within VRPN source directory"
	LOCAL_LEAPMOTION_SUBMODULE_RETRIEVED)

if(VRPN_USE_LOCAL_LEAPMOTION)
	if(WIN32)
		set(LEAPMOTION_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/LeapSDK")
		if(CMAKE_SIZEOF_VOID_P EQUAL 8)						
			set(LEAPMOTION_LIBRARIES "${LEAPMOTION_SOURCE_DIR}/lib/x64/Leap.lib")
		else()			
			set(LEAPMOTION_LIBRARIES "${LEAPMOTION_SOURCE_DIR}/lib/x86/Leap.lib")
		endif()
	else()
		#\todo
		message(STATUS "LeapSDK Unsupported Configuration")
	endif()

	list(APPEND LEAPMOTION_INCLUDE_DIRS "${LEAPMOTION_SOURCE_DIR}/include")
	set(LEAPMOTION_SOURCES "${LEAPMOTION_SOURCE_DIR}/include/Leap.h")
	set(LEAPMOTION_FOUND TRUE)

	source_group("LeapSDK Submodule" FILES ${LEAPMOTION_SOURCES})
endif()