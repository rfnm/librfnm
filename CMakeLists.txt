cmake_minimum_required(VERSION 3.13)

# Version truth lives in include/librfnm/version.h (numeric triple; the pre-release
# suffix there is display-only — CMake, pkg-config and the SONAME use the triple).
foreach(comp MAJOR MINOR PATCH)
  file(STRINGS include/librfnm/version.h _ver_line REGEX "#define RFNM_VERSION_${comp} ")
  string(REGEX MATCH "[0-9]+" RFNM_VERSION_${comp} "${_ver_line}")
endforeach()
project(librfnm VERSION ${RFNM_VERSION_MAJOR}.${RFNM_VERSION_MINOR}.${RFNM_VERSION_PATCH})

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo
      CACHE STRING "Build type: Debug, Release, RelWithDebInfo, MinSizeRel" FORCE)
endif()

# Options
option(BUILD_RFNM_UTILS "Build RFNM utilities such as rfnm_info" ON)
option(BUILD_RFNM_LOCAL_TRANSPORT "Enable local transport (turn on if compiling on the rfnm device itself)" OFF)
option(INSTALL_UDEV_RULES "Install UDEV rules (Linux only)" ON)
option(BUILD_RFNM_TESTS "Build the unit test layer (tests/): pure-function pins for timebase/extension/pad/pack" OFF)


if(CMAKE_C_COMPILER_ID STREQUAL "GNU"
   AND CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  message(STATUS "GNU/AArch64 detected, defining BUILD_RFNM_LOCAL_TRANSPORT (unset if you are not working on the rfnm device itself)")
  add_compile_definitions(BUILD_RFNM_LOCAL_TRANSPORT)
  set(BUILD_RFNM_LOCAL_TRANSPORT ON CACHE BOOL "Enable local transport" FORCE)
endif()


# List all source files
file(GLOB_RECURSE SRC "src/*.cpp")

# On windows, all symbols must be exported
if (MSVC)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif ()

# Apply optimizations to the library if embedded

if(BUILD_RFNM_LOCAL_TRANSPORT)
  message(STATUS "Adding AArch64 optimization flags to rfnm_obj")
  set_source_files_properties(
    "${CMAKE_CURRENT_SOURCE_DIR}/src/core/pack.cpp"
    PROPERTIES
      COMPILE_FLAGS "-mcpu=cortex-a53 -O3 -fopenmp-simd"
  )
endif()



# Build objects
add_library(rfnm_obj OBJECT ${SRC})

# Enable building position independent code since that's required for a shared library
set_property(TARGET rfnm_obj PROPERTY POSITION_INDEPENDENT_CODE ON)

# The exported surface is exactly the MSDLL-decorated API: everything else compiles
# hidden on GCC/Clang (MSVC exports via CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS instead).
target_compile_options(rfnm_obj PRIVATE
    $<$<OR:$<CXX_COMPILER_ID:GNU>,$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>>:-fvisibility=hidden -fvisibility-inlines-hidden>)

# Link to static and dynamic libs
add_library(rfnm_static STATIC $<TARGET_OBJECTS:rfnm_obj>)
add_library(rfnm SHARED $<TARGET_OBJECTS:rfnm_obj>)

# Shared-object versioning: SONAME tracks MAJOR.MINOR while the ABI is pre-1.0
# (every 0.x bump may break ABI, so every 0.x bump gets its own SONAME).
set_target_properties(rfnm PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR})

# A statically-linked libusb must never re-export through librfnm.so: a consumer
# linking its own libusb would interpose two copies (found live: a /usr/local
# static-only libusb leaked all 136 libusb_* symbols into the .so).
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_options(rfnm PRIVATE LINKER:--exclude-libs,ALL)
endif()

# On platforms other than MSVC, make sure the static lib doesn't have "static" in its name
if (NOT MSVC)
    set_target_properties(rfnm_static PROPERTIES OUTPUT_NAME "rfnm")
endif ()

# spdlog::spdlog (system package) already carries SPDLOG_FMT_EXTERNAL in its
# interface; defining it again here double-defined it on every TU (wave-3 hygiene)

# Find spdlog
find_package(spdlog CONFIG REQUIRED)

# Find libusb
find_package(PkgConfig REQUIRED)
pkg_check_modules(libusb REQUIRED IMPORTED_TARGET libusb-1.0)

# Apple ld has no --exclude-libs: a static libusb there would re-export (see the
# Linux note above). Homebrew ships libusb as a dylib, so this is normally moot —
# but fail loudly instead of shipping a leaky dylib if someone links a .a.
if(APPLE AND libusb_LINK_LIBRARIES MATCHES "\\.a$")
    message(WARNING "libusb resolved to a static archive (${libusb_LINK_LIBRARIES}); "
                    "its symbols would re-export from librfnm.dylib. Use the dynamic "
                    "libusb (brew install libusb) for shippable builds.")
endif()

# Called to configure the object, static and dynamic targets, avoids code duplication
function(configure_rfnm_target target)
    # Set C++ standard to C++17
    target_compile_features(${target} PUBLIC cxx_std_17)

    # Include the include folder
    target_include_directories(${target} PUBLIC "include/")

    # Link to spdlog
    target_link_libraries(${target} PRIVATE spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32 iphlpapi>)

    # Link to libusb
    target_link_libraries(${target} PRIVATE PkgConfig::libusb)
endfunction()

# Configure all targets
configure_rfnm_target(rfnm_obj)
configure_rfnm_target(rfnm_static)
configure_rfnm_target(rfnm)




# On Windows, default the install prefix to "C:/Program Files/RFNM" - but only when
# the user did not pass their own -DCMAKE_INSTALL_PREFIX (the old code overrode it)
if (MSVC)
    if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
        set(CMAKE_INSTALL_PREFIX "C:/Program Files/RFNM/" CACHE PATH "Install prefix" FORCE)
    endif ()
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
if (MSVC AND NOT BUILD_SHARED_LIBS)
    install(FILES $<TARGET_RUNTIME_DLLS:rfnm> DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

# Install UDEV rules (CMAKE_SYSTEM_NAME, not LINUX: the LINUX variable only exists
# on cmake >= 3.25, so the old check silently skipped the rules install)
if (CMAKE_SYSTEM_NAME STREQUAL "Linux" AND INSTALL_UDEV_RULES)
    install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/rfnm.rules DESTINATION ${CMAKE_INSTALL_LIBDIR}/udev/rules.d)
endif ()

# Unit test layer: pure functions only (timebase math, tick extension, pad/congruence
# plans, wire pack/unpack golden vectors) - no libusb/spdlog, runs anywhere
if (BUILD_RFNM_TESTS)
    enable_testing()
    add_executable(rfnm_unit
        tests/unit_main.cpp
        tests/test_timebase.cpp
        tests/test_extender.cpp
        tests/test_pad.cpp
        tests/test_pack.cpp
        tests/test_errors.cpp
        src/core/pack.cpp)
    target_include_directories(rfnm_unit PRIVATE include src)
    target_compile_features(rfnm_unit PRIVATE cxx_std_17)
    add_test(NAME rfnm_unit COMMAND rfnm_unit)
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
