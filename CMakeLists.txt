#
# CMake
#

cmake_minimum_required(VERSION 3.22)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

#
# Project
#

project(librfnm
  LANGUAGES CXX
  VERSION 0.0.1
)

#select the release build type by default to get optimization flags
if(NOT DEFINED CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo")
  message(STATUS "Build type not specified: defaulting to release.")
endif()

#enable PIC globally
set(CMAKE_POSITION_INDEPENDENT_CODE ON)


#
# Dependencies
#

find_package(Libusb REQUIRED)

include(cmake/cpm.cmake)
#include(cmake/cpm-fmtlib.cmake)
include(cmake/cpm-spdlog.cmake)


#
# Library
#

add_library(objlib OBJECT ${librfnm})
add_library(rfnm_shared SHARED $<TARGET_OBJECTS:objlib>)
add_library(rfnm_static STATIC $<TARGET_OBJECTS:objlib>)

#target_include_directories(objlib PUBLIC "${CMAKE_SOURCE_DIR}/include")
target_include_directories(objlib ${SPDLOG_INCLUDES_LEVEL} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>"
"$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")

# sources
target_sources(objlib PRIVATE
  "src/librfnm.cpp" )

# definitions
if(MSVC)
  target_compile_definitions(objlib PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

# features
target_compile_features(objlib PUBLIC cxx_std_23)

# libraries
target_link_libraries(rfnm_static PRIVATE LIBUSB::LIBUSB)
#target_link_libraries(rfnm_static PRIVATE fmt::fmt)
target_link_libraries(rfnm_static PRIVATE spdlog)
target_link_libraries(rfnm_shared PRIVATE LIBUSB::LIBUSB)
#target_link_libraries(rfnm_shared PRIVATE fmt::fmt)
target_link_libraries(rfnm_shared PRIVATE spdlog)

#
# Installation
#

#set(DEST_DIR ${CMAKE_INSTALL_LIBDIR}/SoapySDR/modules${SOAPY_SDR_ABI_VERSION}/)

#install(DIRECTORY include/ DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
install(DIRECTORY include/librfnm DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(TARGETS rfnm_shared
    EXPORT librfnmTarget
    LIBRARY DESTINATION lib COMPONENT Runtime
    ARCHIVE DESTINATION lib COMPONENT Development
    RUNTIME DESTINATION bin COMPONENT Runtime
    PUBLIC_HEADER DESTINATION include COMPONENT Development
    BUNDLE DESTINATION bin COMPONENT Runtime
)

set_target_properties(rfnm_shared PROPERTIES OUTPUT_NAME rfnm)
set_target_properties(rfnm_static PROPERTIES OUTPUT_NAME rfnm_static)

#install(EXPORT librfnmTarget DESTINATION librfnm)

#install(
#  FILES "$<TARGET_RUNTIME_DLLS:librfnm>"
#  DESTINATION ${DEST_DIR}
#)

#install(
#  TARGETS librfnm
#  DESTINATION ${DEST_DIR}
#)

#if(MSVC)
#  install(
#    FILES "$<$<OR:$<CONFIG:Debug>,$<CONFIG:RelWithDebInfo>>:$<TARGET_PDB_FILE:librfnm>>"
#    DESTINATION "bin"
#  )
#endif()
