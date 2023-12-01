# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_walker_808x_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED walker_808x_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(walker_808x_FOUND FALSE)
  elseif(NOT walker_808x_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(walker_808x_FOUND FALSE)
  endif()
  return()
endif()
set(_walker_808x_CONFIG_INCLUDED TRUE)

# output package information
if(NOT walker_808x_FIND_QUIETLY)
  message(STATUS "Found walker_808x: 1.0.0 (${walker_808x_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'walker_808x' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${walker_808x_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(walker_808x_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${walker_808x_DIR}/${_extra}")
endforeach()
