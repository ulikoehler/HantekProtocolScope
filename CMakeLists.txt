cmake_minimum_required(VERSION 3.22)
project(UliCanAlsoUseTheScope CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ----------------------------------------------------------------------------
# VolcanoPlot (submodule) — screen backend only, no tests/examples/python.
# ----------------------------------------------------------------------------
set(VOLCANO_BUILD_PYTHON OFF CACHE BOOL "" FORCE)
set(VOLCANO_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(VOLCANO_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(VOLCANO_BUILD_CLI OFF CACHE BOOL "" FORCE)
set(VOLCANO_BUILD_HEADLESS_BACKEND OFF CACHE BOOL "" FORCE)
set(VOLCANO_ENABLE_VALIDATION OFF CACHE BOOL "" FORCE)
set(VOLCANO_BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(VOLCANO_BUILD_STATIC_LIBS ON CACHE BOOL "" FORCE)
set(VOLCANO_BUILD_PARALLELISM 4 CACHE STRING "" FORCE)

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/VolcanoPlot/CMakeLists.txt")
    message(FATAL_ERROR
        "VolcanoPlot submodule fehlt — 'git submodule update --init "
        "--recursive' im Repo-Root ausführen")
endif()
add_subdirectory(VolcanoPlot)

# ----------------------------------------------------------------------------
# ulican executable
# ----------------------------------------------------------------------------
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED IMPORTED_TARGET libusb-1.0)
find_package(Threads REQUIRED)

add_executable(ulican
    src/main.cpp
    src/util/IntelHex.cpp
    src/device/HantekDevice.cpp
    src/device/IsoStream.cpp
    src/file/FileStreamer.cpp
    src/log/Loggers.cpp
    src/can/CanDecoder.cpp
    src/can/LiveDecoder.cpp
    src/can/VirtualTrigger.cpp
    src/ui/MessageListWidget.cpp
    src/ui/ScopeApp.cpp
)

target_include_directories(ulican PRIVATE
    src
    third_party
    # VMA is a directory-scoped include inside VolcanoPlot and does not
    # propagate through target usage requirements.
    ${CMAKE_CURRENT_SOURCE_DIR}/VolcanoPlot/dependencies/VulkanMemoryAllocator/include
)

target_link_libraries(ulican PRIVATE
    volcano_backend
    volcano_render
    volcano_plot
    volcano_encode
    volcano_text
    PkgConfig::LIBUSB
    Threads::Threads
)

target_compile_definitions(ulican PRIVATE
    UCAN_FIRMWARE_DEFAULT="${CMAKE_CURRENT_SOURCE_DIR}/../deps/dso6022bl-firmware.hex"
)
