# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_DOG_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED DOG_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(DOG_FOUND FALSE)
  elseif(NOT DOG_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(DOG_FOUND FALSE)
  endif()
  return()
endif()
set(_DOG_CONFIG_INCLUDED TRUE)

# output package information
if(NOT DOG_FIND_QUIETLY)
  message(STATUS "Found DOG: 0.0.0 (${DOG_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'DOG' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT DOG_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(DOG_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${DOG_DIR}/${_extra}")
endforeach()
