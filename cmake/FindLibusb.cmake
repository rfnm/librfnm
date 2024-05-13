
#
# Includes
#

include(FindPackageHandleStandardArgs)



#
# Check environment
#

if(WIN32)
  set(_LIB_TYPE "STATIC")
  if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(_LIB_FOLDER lib_win64)
  else()
    message(FATAL_ERROR "only 64-bit is supported atm")
  endif()

  find_path(LIBUSB_INCLUDE_DIRS
      libusb-1.0/libusb.h
      HINTS "${CMAKE_SOURCE_DIR}/3rdparty/libusb/"
      PATH_SUFFIXES include
  )

  find_library(LIBUSB_LIBRARIES
      NAMES libusb-1.0
      HINTS "${CMAKE_SOURCE_DIR}/3rdparty/libusb/"
      PATH_SUFFIXES ${_LIB_FOLDER}
  )
else()
  set(_LIB_TYPE "SHARED")

  find_path(LIBUSB_INCLUDE_DIRS
      libusb-1.0/libusb.h
      PATH_SUFFIXES include
  )

  find_library(LIBUSB_LIBRARIES
      NAMES usb-1.0
  )
endif()


#
# Target
#

FIND_PACKAGE_HANDLE_STANDARD_ARGS(Libusb
    REQUIRED_VARS LIBUSB_LIBRARIES LIBUSB_INCLUDE_DIRS)

if(LIBUSB_FOUND)
  if(NOT TARGET LIBUSB::LIBUSB)
      add_library(LIBUSB::LIBUSB ${_LIB_TYPE} IMPORTED)
      set_target_properties(LIBUSB::LIBUSB PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${LIBUSB_INCLUDE_DIRS}")
      set_target_properties(LIBUSB::LIBUSB PROPERTIES IMPORTED_LOCATION "${LIBUSB_LIBRARIES}")
    endif()
endif()
