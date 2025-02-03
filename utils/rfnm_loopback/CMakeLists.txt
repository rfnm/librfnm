cmake_minimum_required(VERSION 3.13)
project(rfnm_loopback)

# Get source files
file(GLOB_RECURSE SRC "src/*.cpp")

# Create executable
add_executable(${PROJECT_NAME} ${SRC})

# Set C++ standard to C++17
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

# Link to librfnm
target_link_libraries(${PROJECT_NAME} PRIVATE rfnm_static)

if(ENABLE_ADDRESS_SANITIZER)
    message(STATUS "Address Sanitizer enabled for target ${PROJECT_NAME}")
    target_compile_options(${PROJECT_NAME} PRIVATE -fsanitize=address)
    target_link_options(${PROJECT_NAME} PRIVATE -fsanitize=address -static-libasan)
endif()

# Create install directive
install(TARGETS ${PROJECT_NAME})