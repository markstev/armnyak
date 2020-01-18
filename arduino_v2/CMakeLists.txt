#=============================================================================#
# Author: MarkStevens
# Date:   2012/04/17
#                                                                             #
# Description: Playing around with serial communication protocol              #
#                                                                             #
#=============================================================================#

cmake_minimum_required(VERSION 2.8)
#====================================================================#
#  Settings                                                          #
#====================================================================#

set(CMAKE_SOURCE_DIR $(pwd)/..) # Source dir:
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/../arduinoio/cmake/ArduinoToolchain.cmake) # Arduino Toolchain
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(ArduinoExample C CXX)
set(ARDUINO_SDK /usr/share/arduino-1.0.6)
set(FIRMWARE_NAME motor_controller)

# Protocol Buffers
set(CMAKE_MODULE_PATH /home/pi/armnyak/nanopb/extra)
#set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/../nanopb/extra)
find_package(Nanopb REQUIRED)
include_directories(${NANOPB_INCLUDE_DIRS})
#nanopb_generate_cpp(PROTO_SRCS PROTO_HDRS RELPATH proto
nanopb_generate_cpp(PROTO_SRCS PROTO_HDRS 
  ${CMAKE_SOURCE_DIR}/proto/motor_command.proto)
message(${PROTO_SRCS})
message(${PROTO_HDRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
# Note, a simple but ugly solution would be to simply add the arduino files directly here.
# Another solution would be to define the arduino mock-out via dependency
# injection, which is probably nicer.
set_source_files_properties(${PROTO_SRCS} ${PROTO_HDRS}
    PROPERTIES GENERATED TRUE)

# Needed to force generation of the proto sources.
#add_executable(test motor_controller.cc ${PROTO_SRCS} ${PROTO_HDRS})
#add_library(serial_link IMPORTED 
add_subdirectory(includes/arduinoio3)

#set(${FIRMWARE_NAME}_BOARD mega2560)               # Arduino Target board
set(${FIRMWARE_NAME}_BOARD uno)               # Arduino Target board
#SET(CMCU "-mmcu=atmega2560")
set(${FIRMWARE_NAME}_SRCS  motor_controller.cc ${PROTO_SRCS})
message("SRCS ARE " ${motor_controller_SRCS})
set(${FIRMWARE_NAME}_HDRS   ${PROTO_HDRS})

set(${FIRMWARE_NAME}_PORT /dev/ttyACM1)            # Serial upload port
#set(${FIRMWARE_NAME}_PORT /dev/ttyACM0)            # Serial upload port

#====================================================================#
#  Target generation                                                 #
#====================================================================#
generate_arduino_firmware(${FIRMWARE_NAME})

target_link_libraries(${FIRMWARE_NAME} arduinoio)
target_link_libraries(${FIRMWARE_NAME} uc_module)
#target_link_libraries(${FIRMWARE_NAME} serial_link)
