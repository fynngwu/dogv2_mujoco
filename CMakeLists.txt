cmake_minimum_required(VERSION 3.10)
project(dogv2_mujoco VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

get_filename_component(PROJECT_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}" ABSOLUTE)
set(SYSTEM_TYPE ${CMAKE_SYSTEM_NAME})

# ========================
# Inference Runtime
# ========================
set(INFERENCE_RUNTIME_DIR "${PROJECT_ROOT_DIR}/lib")

# ONNX Runtime
set(USE_ONNX "OFF")
set(ONNX_RUNTIME_LIB "")
set(ONNX_RUNTIME_DIR "${INFERENCE_RUNTIME_DIR}/onnxruntime")
if(EXISTS "${ONNX_RUNTIME_DIR}/include" AND EXISTS "${ONNX_RUNTIME_DIR}/lib")
    set(USE_ONNX "ON")
    add_compile_definitions(USE_ONNX)
    include_directories("${ONNX_RUNTIME_DIR}/include")
    if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
        set(LIB_NAMES onnxruntime.lib)
    else()
        set(LIB_NAMES onnxruntime.so onnxruntime.dylib libonnxruntime.so.1.19.2 libonnxruntime.so libonnxruntime.dylib)
    endif()
    find_library(ONNX_RUNTIME_LIB_PATH NAMES ${LIB_NAMES} PATHS "${ONNX_RUNTIME_DIR}/lib" NO_DEFAULT_PATH)
    if(ONNX_RUNTIME_LIB_PATH)
        add_library(onnxruntime_prebuilt SHARED IMPORTED GLOBAL)
        set_target_properties(onnxruntime_prebuilt PROPERTIES
            IMPORTED_LOCATION "${ONNX_RUNTIME_LIB_PATH}"
            INTERFACE_INCLUDE_DIRECTORIES "${ONNX_RUNTIME_DIR}/include")
        set(ONNX_RUNTIME_LIB onnxruntime_prebuilt)
        message(STATUS "ONNX Runtime: ${ONNX_RUNTIME_LIB_PATH}")
    else()
        set(USE_ONNX "OFF")
        message(WARNING "ONNX Runtime library not found")
    endif()
else()
    message(STATUS "ONNX Runtime not found (optional)")
endif()

if(NOT USE_ONNX)
    message(FATAL_ERROR "No inference backend found. Need ONNX Runtime.")
endif()

# RPATH
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)
set(PREBUILT_LIB_PATHS "")
if(USE_ONNX)
    list(APPEND PREBUILT_LIB_PATHS "${ONNX_RUNTIME_DIR}/lib")
endif()

# ========================
# MuJoCo
# ========================
set(MUJOCO_DIR "${PROJECT_ROOT_DIR}/lib/mujoco")
if(NOT EXISTS "${MUJOCO_DIR}/include/mujoco/mujoco.h")
    message(FATAL_ERROR "MuJoCo not found at ${MUJOCO_DIR}")
endif()

set(MUJOCO_INCLUDE_DIR "${MUJOCO_DIR}/include")
set(MUJOCO_LIB_DIR "${MUJOCO_DIR}/lib")
list(APPEND PREBUILT_LIB_PATHS "${MUJOCO_LIB_DIR}")
set(CMAKE_INSTALL_RPATH "${PREBUILT_LIB_PATHS}")

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(MUJOCO_LIB_NAME "libmujoco.so.3.8.1")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--disable-new-dtags")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    set(MUJOCO_LIB_NAME "libmujoco.3.8.1.dylib")
endif()

add_library(mujoco_prebuilt SHARED IMPORTED)
set_target_properties(mujoco_prebuilt PROPERTIES
    IMPORTED_LOCATION "${MUJOCO_LIB_DIR}/${MUJOCO_LIB_NAME}"
    INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}"
    IMPORTED_NO_SONAME TRUE)

find_package(glfw3 REQUIRED)

# ========================
# System deps
# ========================
find_package(Threads REQUIRED)
find_package(TBB REQUIRED)
find_package(yaml-cpp REQUIRED)

# ========================
# Core libraries
# ========================
set(CORE_SRCS
    src/core/rl_sdk.cpp
    src/core/observation_buffer.cpp
    src/core/inference_runtime.cpp
)
add_library(rl_sdk_lib STATIC ${CORE_SRCS})
target_include_directories(rl_sdk_lib PUBLIC
    src/core
    ${MUJOCO_INCLUDE_DIR}
)
target_link_libraries(rl_sdk_lib PUBLIC
    Threads::Threads TBB::tbb ${YAML_CPP_LIBRARIES}
)
if(USE_ONNX)
    target_include_directories(rl_sdk_lib PUBLIC ${ONNX_RUNTIME_DIR}/include)
    target_link_libraries(rl_sdk_lib PUBLIC ${ONNX_RUNTIME_LIB})
endif()

# ========================
# MuJoCo simulate thirdparty
# ========================
file(GLOB MUJOCO_SIMULATE_SRC thirdparty/mujoco_simulate/*.cc)
set(JOYSTICK_SRC thirdparty/joystick/joystick.cc)

# lodepng: header+implementation in one file
add_library(lodepng_lib STATIC ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/mujoco_simulate/lodepng.cpp)
target_include_directories(lodepng_lib PUBLIC thirdparty/mujoco_simulate)

# ========================
# Main executable
# ========================
set(MAIN_SRCS
    src/main.cpp
    ${MUJOCO_SIMULATE_SRC}
    ${JOYSTICK_SRC}
)
if(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    list(APPEND MAIN_SRCS
        thirdparty/mujoco_simulate/macos_gui.mm
        thirdparty/mujoco_simulate/glfw_corevideo.mm
    )
endif()

add_executable(dogv2_mujoco ${MAIN_SRCS})
target_include_directories(dogv2_mujoco PRIVATE
    src
    src/core
    thirdparty/mujoco_simulate
    thirdparty/joystick
    ${MUJOCO_INCLUDE_DIR}
)
target_link_libraries(dogv2_mujoco
    rl_sdk_lib
    mujoco_prebuilt
    lodepng_lib
    glfw
)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    find_package(OpenGL REQUIRED)
    target_link_libraries(dogv2_mujoco OpenGL::GL)
endif()
if(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    target_link_libraries(dogv2_mujoco
        "-framework CoreVideo"
        "-framework AppKit"
        "-framework Foundation"
    )
endif()

if(USE_ONNX)
    target_link_libraries(dogv2_mujoco ${ONNX_RUNTIME_LIB})
endif()

message(STATUS "")
message(STATUS "=== dogv2_mujoco ===")
message(STATUS "  ONNX:  ${USE_ONNX}")
message(STATUS "  MuJoCo: ${MUJOCO_DIR}")
message(STATUS "====================")
message(STATUS "")
