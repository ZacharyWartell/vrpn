# - try to find LeapMotion Library
# from ???
#
# Cache Variables: (probably not for direct use in your scripts)
#  LEAPMOTION_INCLUDE_DIR
#  LEAPMOTION_LIBRARY
#
# Non-cache variables you might use in your CMakeLists.txt:
#  LEAPMOTION_FOUND
#  LEAPMOTION_INCLUDE_DIRS
#  LEAPMOTION_LIBRARIES
#
# Requires these CMake modules:
#  FindPackageHandleStandardArgs (known included with CMake >=2.6.2)
#
# Original Author:
# Zachary Wartell, zwartell@uncc.edu, https://webpages.uncc.edu/zwartell/
#
# Copyright UNC Charlotte 2020.
# Distributed under the Boost Software License, Version 1.0.
# (See accompanying file LICENSE_1_0.txt or copy at
# http://www.boost.org/LICENSE_1_0.txt)

find_library(LEAPMOTION_LIBRARY
	NAMES Leap )

find_path(LEAPMOTION_INCLUDE_DIR
	NAMES Leap.h
	PATH_SUFFIXES
	LeapSDK )

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(LEAPMOTION
	DEFAULT_MSG
	LEAPMOTION_LIBRARY
	LEAPMOTION_INCLUDE_DIR)

if(LEAPMOTION_FOUND)
	set(LEAPMOTION_LIBRARIES "${LEAPMOTION_LIBRARY}")

	set(LEAPMTION_INCLUDE_DIRS "${LEAPMOTION_INCLUDE_DIR}")
endif()

mark_as_advanced(LEAPMOTION_INCLUDE_DIR LEAPMOTION_LIBRARY)
