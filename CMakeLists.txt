cmake_minimum_required(VERSION 3.17)

# ============================================================================
#  rp2350_lan8720 — RP2354 (2 MB) + LAN8720 (RMII) EtherCAT master + SH1107 OLED
#
#  pico-sdk + FreeRTOS (SMP) + LVGL
#
#  The LAN8720 is an RMII PHY (no MAC); the RP2350 implements the MAC in PIO+DMA
#  (src/rmii_mac.c, src/pio/lan8720_*.pio — edge-synced to REF_CLK, after
#  messani/pico-lan8720). A small hand-rolled EtherCAT master (src/ethercat.c)
#  runs on top (EtherType 0x88A4, no TCP/IP). Deps fetched into ./extern.
#
#  Build:
#     mkdir build && cd build && cmake .. && make flash monitor
#  Re-flash a running board (picotool + libusb, no BOOTSEL button):
#     make flash
# ============================================================================

include(FetchContent)
set(FETCHCONTENT_UPDATES_DISCONNECTED TRUE)
set(FETCHCONTENT_QUIET FALSE)

# ── Generic vendoring helper ────────────────────────────────────────────────
#   fetch_dep(<name> <var> <dir> <url> <tag> <sentinel-rel-path> <submodules> <shallow>)
#   Populates <dir> from GitHub once (only if <dir>/<sentinel> is missing),
#   then exports <var>=<dir> into the cache.  <shallow>=OFF is required when
#   <tag> is a raw commit SHA (GitHub can't shallow-fetch those).
macro(fetch_dep name var dir url tag sentinel submodules shallow)
    if(NOT EXISTS "${dir}/${sentinel}")
        message(STATUS "[extern] ${name} missing — fetching ${tag} from ${url}")
        FetchContent_Declare(_${name}_dl
            GIT_REPOSITORY          "${url}"
            GIT_TAG                 "${tag}"
            GIT_SUBMODULES_RECURSE  ${submodules}
            GIT_SHALLOW             ${shallow}
            SOURCE_DIR              "${dir}"
            SUBBUILD_DIR            "${CMAKE_BINARY_DIR}/_deps/${name}-sub"
            BINARY_DIR              "${CMAKE_BINARY_DIR}/_deps/${name}-bin"
        )
        FetchContent_GetProperties(_${name}_dl)
        if(NOT _${name}_dl_POPULATED)
            FetchContent_Populate(_${name}_dl)
        endif()
    else()
        message(STATUS "[extern] ${name} present in ${dir}")
    endif()
    set(${var} "${dir}" CACHE PATH "${name} path" FORCE)
endmacro()

set(EXTERN "${CMAKE_SOURCE_DIR}/extern")

# ── pico-sdk ─────────────────────────────────────────────────────────────────
fetch_dep(pico-sdk PICO_SDK_PATH
    "${EXTERN}/pico-sdk"
    "https://github.com/raspberrypi/pico-sdk.git"
    "2.2.0" "pico_sdk_init.cmake" FALSE TRUE)
unset(ENV{PICO_SDK_PATH})

# ── picotool (for `make flash` over USB/libusb) ─────────────────────────────
fetch_dep(picotool PICOTOOL_SRC_DIR
    "${EXTERN}/picotool"
    "https://github.com/raspberrypi/picotool.git"
    "2.2.0-a4" "CMakeLists.txt" FALSE TRUE)

# ── Target board / platform ──────────────────────────────────────────────────
# RP2354A — RP2350A core with 2 MB internal stacked flash.
set(PICO_PLATFORM rp2350-arm-s          CACHE STRING "Pico platform")
set(PICO_BOARD    rp2350_lan8720_board  CACHE STRING "Pico board")
list(APPEND PICO_BOARD_HEADER_DIRS "${CMAKE_SOURCE_DIR}/boards")
set(PICO_FLASH_SIZE_BYTES 2097152 CACHE STRING "RP2354 internal flash size" FORCE)

set(PICOTOOL_FETCH_FROM_GIT_PATH "${CMAKE_BINARY_DIR}/_picotool"
    CACHE PATH "Where pico-sdk fetches its internal picotool")

# ── Pull in the Pico SDK (must come before project()) ───────────────────────
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(rp2350_lan8720 C CXX ASM)
set(CMAKE_C_STANDARD   11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_FLAGS_RELEASE "-O2")

pico_sdk_init()

# ── FreeRTOS-Kernel (RP2350 ARM SMP / NTZ port) ─────────────────────────────
fetch_dep(FreeRTOS-Kernel FREERTOS_KERNEL_PATH
    "${EXTERN}/FreeRTOS-Kernel"
    "https://github.com/FreeRTOS/FreeRTOS-Kernel.git"
    "d1f551e2539ef7324013aebf22cd430a690e0d42" "tasks.c" FALSE FALSE)
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/Community-Supported-Ports/GCC/RP2350_ARM_NTZ/FreeRTOS_Kernel_import.cmake)

# EtherCAT: a small hand-rolled master in src/ethercat.c runs directly on the
# LAN8720 PIO MAC (raw L2 send/receive) — no SOEM, no lwIP.

# ── pico-lan8720 (reference for the edge-synced PIO RMII MAC in src/) ────────
# Vendored into extern/ for provenance; the MAC + PIO live in src/ (no lwIP).
# messani's design re-syncs to REF_CLK every di-bit, which is what makes 100 Mbit
# TX work on an unmodified module without phase-locking the system clock.
fetch_dep(pico-lan8720 LAN8720_REF_DIR
    "${EXTERN}/pico-lan8720"
    "https://github.com/messani/pico-lan8720.git"
    "main" "src/lan8720.c" FALSE TRUE)

# ── LVGL ─────────────────────────────────────────────────────────────────────
fetch_dep(lvgl LVGL_DIR
    "${EXTERN}/lvgl"
    "https://github.com/lvgl/lvgl.git"
    "v9.2.2" "lvgl.h" FALSE TRUE)
set(LV_CONF_PATH "${CMAKE_SOURCE_DIR}/lv_conf.h" CACHE STRING "" FORCE)
set(LV_CONF_BUILD_DISABLE_EXAMPLES TRUE CACHE BOOL "" FORCE)
set(LV_CONF_BUILD_DISABLE_DEMOS    TRUE CACHE BOOL "" FORCE)
add_subdirectory(${LVGL_DIR} lvgl EXCLUDE_FROM_ALL)
get_target_property(_LVGL_SRCS lvgl SOURCES)
list(FILTER _LVGL_SRCS EXCLUDE REGEX "\\.S$")
set_target_properties(lvgl PROPERTIES SOURCES "${_LVGL_SRCS}")

# ── Application ───────────────────────────────────────────────────────────────
add_executable(${PROJECT_NAME}
    src/main.c
    src/log.c
    src/shared_state.c
    src/rmii_mac.c
    src/ethercat.c
    src/sh1107.c
    src/display_task.c
    src/periph.c
)

# Compile the RMII PIO programs (pin map lives inside the .pio).
pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/src/pio/lan8720_rx.pio)
pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/src/pio/lan8720_tx.pio)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    pico_stdlib
    pico_multicore
    pico_sync
    hardware_i2c
    hardware_gpio
    hardware_clocks
    hardware_timer
    hardware_pio
    hardware_dma
    hardware_pwm
    hardware_vreg
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
    lvgl
)

# stdio over USB CDC only — printf lands on /dev/ttyACM0 (`make monitor`).
pico_enable_stdio_usb(${PROJECT_NAME}  1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    PICO_STDIO_USB_ENABLE_RESET_VIA_VENDOR_INTERFACE=1
    PICO_STDIO_USB_RESET_INTERFACE_SUPPORT_RESET_TO_BOOTSEL=1
    PICO_STDIO_USB_RESET_INTERFACE_SUPPORT_RESET_TO_FLASH_BOOT=1
    PICO_STDIO_USB_RESET_BOOTSEL_ACTIVITY_LED=PICO_DEFAULT_LED_PIN
    PICO_STDIO_USB_RESET_RESET_TO_FLASH_DELAY_MS=100
    PICO_STDIO_USB_ENABLE_RESET_VIA_BAUD_RATE=1
)

pico_add_extra_outputs(${PROJECT_NAME})

# Keep firmware/<name>.uf2 (tracked in git) in sync with each build.
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
            $<TARGET_FILE_DIR:${PROJECT_NAME}>/${PROJECT_NAME}.uf2
            ${CMAKE_SOURCE_DIR}/firmware/${PROJECT_NAME}.uf2
    COMMENT "Copying UF2 to firmware/")

# ============================================================================
#                          Convenience targets
# ============================================================================
include(ExternalProject)
set(PICOTOOL_BUILD_DIR ${CMAKE_BINARY_DIR}/picotool-host)
set(PICOTOOL_EXE       ${PICOTOOL_BUILD_DIR}/picotool)

ExternalProject_Add(picotool_host
    SOURCE_DIR        ${PICOTOOL_SRC_DIR}
    BINARY_DIR        ${PICOTOOL_BUILD_DIR}
    CMAKE_ARGS
        -DPICO_SDK_PATH=${PICO_SDK_PATH}
        -DCMAKE_BUILD_TYPE=Release
    BUILD_BYPRODUCTS  ${PICOTOOL_EXE}
    INSTALL_COMMAND   ""
    EXCLUDE_FROM_ALL  TRUE
    BUILD_ALWAYS      0
)

add_custom_target(flash
    COMMAND ${PICOTOOL_EXE} load -f -x -v $<TARGET_FILE:${PROJECT_NAME}>
    DEPENDS ${PROJECT_NAME} picotool_host
    USES_TERMINAL
    COMMENT "Flashing ${PROJECT_NAME}.elf and executing")

add_custom_target(reboot
    COMMAND ${PICOTOOL_EXE} reboot
    DEPENDS picotool_host
    USES_TERMINAL
    COMMENT "Rebooting connected RP2350")

set(MONITOR_PORT /dev/ttyACM0 CACHE STRING "Serial port for `make monitor`")
set(MONITOR_BAUD 115200       CACHE STRING "Baud rate for `make monitor`")
add_custom_target(monitor
    COMMAND tio -b ${MONITOR_BAUD} -m INLCRNL ${MONITOR_PORT}
    USES_TERMINAL
    COMMENT "tio ${MONITOR_PORT} @ ${MONITOR_BAUD}")
