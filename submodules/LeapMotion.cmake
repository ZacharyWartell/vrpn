###
# @uathor Zachary Wartell
# @brief LeapMotion SDK V2
#
# This is Work in progress...
# Initial file copied and modified from hidapi.cmake (not sure if that was the best example to copy)
###

set(LOCAL_LEAPMOTION_SUBMODULE_RETRIEVED FALSE)

set(LEAPMOTION_BACKEND_FOUND NO)


if(0)
    # \todo ZJW: unlike for the HIDAPI I think LeapMotion SDK does not have any of these (similar to jsoncpp.cmake)

    # Local LeapSDK requirements
	if(ANDROID)
		# \todo
		message(STATUS "LeapSDK Unsupported Configuration")
	elseif(WIN32)
		set(LEAPMOTION_BACKEND_FOUND YES)
	elseif(APPLE)
		# \todo
		message(STATUS "LeapSDK Unsupported Configuration")
	else()
		# \todo	
		message(STATUS "LeapSDK Unsupported Configuration")
	endif()
else()
	set(LEAPMOTION_BACKEND_FOUND YES)
endif()

if(EXISTS "${VRPN_SOURCE_DIR}/submodules/LeapSDK/include/Leap.h")
	set(LOCAL_LEAPMOTION_SUBMODULE_RETRIEVED TRUE)
endif()

if(NOT LOCAL_LEAPMOTION_SUBMODULE_RETRIEVED AND NOT SUBPROJECT)
	message(STATUS
		"Local LeapSDK submodule not found. To download with Git, run git submodule update --init")
endif()

###
# OpenCV for VRPN_USE_LEAPMOTION_GLASSES_TRACKING
###
find_package(OpenCV PATHS "${CMAKE_INSTALL_PREFIX}/")

# XXX Safe to enable by default if we find it?
option_requires(VRPN_USE_LEAPMOTION_GLASSES_TRACKING
	"Build Tracker_LeapMotion with OpenCV support"
	#OFF_BY_DEFAULT
	OpenCV_FOUND)


set(LEAPMOTION_INCLUDE_DIRS)

if(VRPN_USE_LEAPMOTION_GLASSES_TRACKING AND OpenCV_FOUND)
	# what else needs to be done here?
	message(STATUS
		"OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
	# add_definitions(${OpenCV_COMPILE_FLAGS})
	# include_directories(${OpenCV_INCLUDE_DIRS})
	list(APPEND LEAPMOTION_INCLUDE_DIRS "${OpenCV_INCLUDE_DIRS}")
	list(APPEND SERVER_EXTRA_LIBS ${OpenCV_LIBS})
endif()

option_requires(VRPN_USE_LOCAL_LEAPMOTION
	"Build with LeapMotion code from within VRPN source directory"
	LOCAL_LEAPMOTION_SUBMODULE_RETRIEVED
	LEAPMOTION_BACKEND_FOUND)

if(VRPN_USE_LOCAL_LEAPMOTION)
	if(WIN32)
		set(LEAPMOTION_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/LeapSDK")
		if(CMAKE_SIZEOF_VOID_P EQUAL 8)			
			list(APPEND SERVER_EXTRA_LIBS "${LEAPMOTION_SOURCE_DIR}/lib/x64/Leap.lib")
			set(LEAPMOTION_LIBRARIES "${LEAPMOTION_SOURCE_DIR}/lib/x64/Leap.lib")
		else()
			list(APPEND SERVER_EXTRA_LIBS "${LEAPMOTION_SOURCE_DIR}/lib/x86/Leap.lib")
			set(LEAPMOTION_LIBRARIES "${LEAPMOTION_SOURCE_DIR}/lib/x86/Leap.lib")
		endif()
	else()
		#\todo
		message(STATUS "LeapSDK Unsupported Configuration")
	endif()

	list(APPEND LEAPMOTION_INCLUDE_DIRS "${LEAPMOTION_SOURCE_DIR}/include")
	set(LEAPMOTION_SOURCES "${LEAPMOTION_SOURCE_DIR}/include/Leap.h")
	set(LEAPMOTION_FOUND TRUE)
	
	if(0)
		# \todo ZJW: unlike for the HIDAPI I think LeapMotion SDK does not have any of these (similar to jsoncpp.cmake)
		# Set up desired backends
		if(APPLE)
			#\todo
			message(STATUS "LeapSDK Unsupported Configuration")
		elseif(WIN32)
			#\todo test&debug this...		
			set(LEAPMOTION_LIBRARIES Leap)
		elseif(VRPN_LEAPMOTION_USE_LIBUSB)
			#\todo
			message(STATUS "LeapSDK Unsupported Configuration")		
		elseif(VRPN_LEAPMOTION_USE_LINUXUDEV)
			#\todo
			message(STATUS "LeapSDK Unsupported Configuration")				
		else()
			message(STATUS "LeapSDK Unsupported Configuration")						
			set(LEAPMOTION_FOUND FALSE)

		endif()
		if(VRPN_LEAPMOTION_USE_LIBUSB OR VRPN_LEAPMOTION_USE_LINUXUDEV)
			#\todo ... (see hidapi.cmake)		
			message(STATUS "LeapSDK Unsupported Configuration")						
		endif()
	endif()

	source_group("LeapSDK Submodule" FILES ${LEAPMOTION_SOURCES})
endif()
