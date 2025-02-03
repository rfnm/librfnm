cmake_minimum_required(VERSION 3.13)
project(librfnm VERSION 0.2.0)

# Options
option(BUILD_RFNM_UTILS "Build RFNM utilities such as rfnm_info" ON)
option(INSTALL_UDEV_RULES "Install UDEV rules (Linux only)" ON)

set(ENABLE_ADDRESS_SANITIZER OFF)

# List all source files
file(GLOB_RECURSE SRC "src/*.cpp")

# On windows, all symbols must be exported
if (MSVC)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif ()

# Build objects
add_library(rfnm_obj OBJECT ${SRC})

# Enable building position independent code since that's required for a shared library
set_property(TARGET rfnm_obj PROPERTY POSITION_INDEPENDENT_CODE ON)

# Link to static and dynamic libs
add_library(rfnm_static STATIC $<TARGET_OBJECTS:rfnm_obj>)
add_library(rfnm SHARED $<TARGET_OBJECTS:rfnm_obj>)

# On platforms other than MSVC, make sure the static lib doesn't have "static" in its name
if (NOT MSVC)
    set_target_properties(rfnm_static PROPERTIES OUTPUT_NAME "rfnm")
endif ()

add_compile_definitions(SPDLOG_FMT_EXTERNAL=0)

# Find spdlog
find_package(spdlog CONFIG REQUIRED)

# Find libusb
find_package(PkgConfig REQUIRED)
pkg_check_modules(libusb REQUIRED IMPORTED_TARGET libusb-1.0)

# Find ASIO
set(ASIO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/asio")
include_directories("${ASIO_ROOT}/include")
add_definitions(-DASIO_STANDALONE)

# Called to configure the object, static and dynamic targets, avoids code duplication
function(configure_rfnm_target target)
    # Set C++ standard to C++17
    target_compile_features(${target} PUBLIC cxx_std_17)

    if(ENABLE_ADDRESS_SANITIZER)
        message(STATUS "Address Sanitizer enabled for target ${target}")
        target_compile_options(${target} PRIVATE -fsanitize=address)
        target_link_options(${target} PRIVATE -fsanitize=address -static-libasan)
    endif()

    # Include the include folder
    target_include_directories(${target} PUBLIC "include/")

    # Link to spdlog
    target_link_libraries(${target} PRIVATE spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>)

    # Link to libusb
    target_link_libraries(${target} PRIVATE PkgConfig::libusb)
endfunction()

# Configure all targets
configure_rfnm_target(rfnm_obj)
configure_rfnm_target(rfnm_static)
configure_rfnm_target(rfnm)

# On Windows, the install prefix should default to "C:/Program Files/RFNM" (TODO: Have it default instead of override)
if (MSVC)
    set(CMAKE_INSTALL_PREFIX "C:/Program Files/RFNM/")
    set(CMAKE_INSTALL_BINDIR "bin")
    set(CMAKE_INSTALL_LIBDIR "lib")
    set(CMAKE_INSTALL_INCLUDEDIR "include")
else ()
    include(GNUInstallDirs)
endif ()

# Configure pkgconfig file
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/librfnm.pc.in ${CMAKE_CURRENT_BINARY_DIR}/librfnm.pc @ONLY)

# Install pkgconfig file
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/librfnm.pc DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)

# Install the library
install(TARGETS rfnm_static rfnm)

# Install the headers
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# On Windows, install dependencies
if (MSVC)
    install(FILES $<TARGET_FILE_DIR:rfnm>/spdlog.dll DESTINATION ${CMAKE_INSTALL_BINDIR})
    install(FILES $<TARGET_FILE_DIR:rfnm>/fmt.dll DESTINATION ${CMAKE_INSTALL_BINDIR})
    install(FILES $<TARGET_FILE_DIR:rfnm>/libusb-1.0.dll DESTINATION ${CMAKE_INSTALL_BINDIR})
endif ()

# Install UDEV rules
if (LINUX AND INSTALL_UDEV_RULES)
    install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/rfnm.rules DESTINATION ${CMAKE_INSTALL_LIBDIR}/udev/rules.d)
endif ()

# Build utils if enabled
if (BUILD_RFNM_UTILS)
    add_subdirectory("utils/rfnm_info")
    add_subdirectory("utils/rfnm_tx_sine")
    add_subdirectory("utils/rfnm_rx")
    add_subdirectory("utils/rfnm_loopback")
endif ()

# Create uninstall target
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake @ONLY)
add_custom_target(uninstall ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
