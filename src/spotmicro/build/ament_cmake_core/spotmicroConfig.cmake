# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_spotmicro_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED spotmicro_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(spotmicro_FOUND FALSE)
  elseif(NOT spotmicro_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(spotmicro_FOUND FALSE)
  endif()
  return()
endif()
set(_spotmicro_CONFIG_INCLUDED TRUE)

# output package information
if(NOT spotmicro_FIND_QUIETLY)
  message(STATUS "Found spotmicro: 0.0.0 (${spotmicro_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'spotmicro' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${spotmicro_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(spotmicro_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${spotmicro_DIR}/${_extra}")
endforeach()
