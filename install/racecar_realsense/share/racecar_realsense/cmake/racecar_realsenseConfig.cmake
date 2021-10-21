# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_racecar_realsense_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED racecar_realsense_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(racecar_realsense_FOUND FALSE)
  elseif(NOT racecar_realsense_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(racecar_realsense_FOUND FALSE)
  endif()
  return()
endif()
set(_racecar_realsense_CONFIG_INCLUDED TRUE)

# output package information
if(NOT racecar_realsense_FIND_QUIETLY)
  message(STATUS "Found racecar_realsense: 0.0.0 (${racecar_realsense_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'racecar_realsense' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${racecar_realsense_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(racecar_realsense_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${racecar_realsense_DIR}/${_extra}")
endforeach()
