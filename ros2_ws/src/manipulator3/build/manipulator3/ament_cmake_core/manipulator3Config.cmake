# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_manipulator3_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED manipulator3_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(manipulator3_FOUND FALSE)
  elseif(NOT manipulator3_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(manipulator3_FOUND FALSE)
  endif()
  return()
endif()
set(_manipulator3_CONFIG_INCLUDED TRUE)

# output package information
if(NOT manipulator3_FIND_QUIETLY)
  message(STATUS "Found manipulator3: 0.0.0 (${manipulator3_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'manipulator3' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${manipulator3_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(manipulator3_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${manipulator3_DIR}/${_extra}")
endforeach()
