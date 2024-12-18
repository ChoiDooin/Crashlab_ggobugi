# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_crash_motor_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED crash_motor_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(crash_motor_FOUND FALSE)
  elseif(NOT crash_motor_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(crash_motor_FOUND FALSE)
  endif()
  return()
endif()
set(_crash_motor_CONFIG_INCLUDED TRUE)

# output package information
if(NOT crash_motor_FIND_QUIETLY)
  message(STATUS "Found crash_motor: 0.0.1 (${crash_motor_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'crash_motor' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${crash_motor_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(crash_motor_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${crash_motor_DIR}/${_extra}")
endforeach()
