# Flash an ELF over SWD without requiring an NRST connection.
#
# STM32CubeProgrammer programs flash by downloading a loader into SRAM and
# running it, which needs a cleanly reset, halted core. Without an NRST wire
# mode=UR is unavailable, so reproduce it through the Cortex-M debug registers:
# arm vector-catch-on-reset, then request a system reset.
#
# Usage: cmake -DPROG=<cli> -DELF=<file> -P flash.cmake

cmake_minimum_required(VERSION 3.22)

set(DEMCR 0xE000EDFC)   # Debug Exception and Monitor Control
set(DHCSR 0xE000EDF0)   # Debug Halting Control and Status
set(AIRCR 0xE000ED0C)   # Application Interrupt and Reset Control

set(CONNECT -c port=SWD mode=HOTPLUG freq=480)

# Writes that trigger a reset always report failure, because the debug link
# drops mid-transaction. That is expected, so errors are ignored here.
function(swd_write)
    execute_process(COMMAND ${PROG} ${CONNECT} ${ARGN} OUTPUT_QUIET ERROR_QUIET)
endfunction()

message(STATUS "Resetting target and halting at reset vector")
swd_write(-w32 ${DEMCR} 0x00000001    # VC_CORERESET: halt on reset vector
          -w32 ${DHCSR} 0xA05F0001    # C_DEBUGEN
          -w32 ${AIRCR} 0x05FA0004)   # SYSRESETREQ

# The debug port needs a moment to re-attach after the reset before the
# programmer can connect and run its SRAM loader.
execute_process(COMMAND ${CMAKE_COMMAND} -E sleep 1)

message(STATUS "Programming ${ELF}")
execute_process(COMMAND ${PROG} ${CONNECT} -w ${ELF} -v RESULT_VARIABLE result)
if(NOT result EQUAL 0)
    message(FATAL_ERROR "Programming failed (${result})")
endif()

message(STATUS "Releasing core")
swd_write(-w32 ${DEMCR} 0x00000000    # disarm vector catch
          -w32 ${DHCSR} 0xA05F0000)   # release debug
swd_write(-w32 ${AIRCR} 0x05FA0004)   # boot the new firmware

message(STATUS "Target reset into new firmware")
