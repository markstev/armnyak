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

project(ArduinoExample C CXX)
set(ARDUINO_SDK /usr/share/arduino-1.0.6)
set(FIRMWARE_NAME motor_controller)

# Protocol Buffers
set(CMAKE_MODULE_PATH /home/pi/armnyak/nanopb/extra)
#set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/../nanopb/extra)
find_package(Nanopb REQUIRED)
include_directories(${NANOPB_INCLUDE_DIRS})
nanopb_generate_cpp(PROTO_SRCS PROTO_HDRS RELPATH proto
	proto/motor_command.proto)
message(${PROTO_SRCS})
message(${PROTO_HDRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
set_source_files_properties(${PROTO_SRCS} ${PROTO_HDRS}
    PROPERTIES GENERATED TRUE)

# Needed to force generation of the proto sources.
#add_executable(test test.cc ${PROTO_SRCS} ${PROTO_HDRS})

#set(${FIRMWARE_NAME}_BOARD mega2560)               # Arduino Target board
set(${FIRMWARE_NAME}_BOARD uno)               # Arduino Target board
#SET(CMCU "-mmcu=atmega2560")
set(${FIRMWARE_NAME}_SRCS  motor_controller.cc ${PROTO_SRCS})
set(${FIRMWARE_NAME}_HDRS   ${PROTO_HDRS})

set(${FIRMWARE_NAME}_PORT /dev/ttyACM0)            # Serial upload port

#====================================================================#
#  Target generation                                                 #
#====================================================================#
generate_arduino_firmware(${FIRMWARE_NAME})
