# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_fix2nmea_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED fix2nmea_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(fix2nmea_FOUND FALSE)
  elseif(NOT fix2nmea_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(fix2nmea_FOUND FALSE)
  endif()
  return()
endif()
set(_fix2nmea_CONFIG_INCLUDED TRUE)

# output package information
if(NOT fix2nmea_FIND_QUIETLY)
  message(STATUS "Found fix2nmea: 0.0.0 (${fix2nmea_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'fix2nmea' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${fix2nmea_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(fix2nmea_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${fix2nmea_DIR}/${_extra}")
endforeach()
