if(${PLATFORM} MATCHES "Arduino")
  set(CMAKE_TOOLCHAIN_FILE ${CMAKE_ROOT}/Modules/ArduinoToolchain.cmake) # Arduino Toolchain
  set(ARDUINO_AVRDUDE_PROGRAM avrdude.py) # override arvdude for upload to micro and leonardo
endif(${PLATFORM} MATCHES "Arduino")

cmake_minimum_required (VERSION 2.8.5)
cmake_policy(VERSION 2.8.5)
project(signal)

# flags for debug version: "-DCMAKE_BUILD_TYPE=Debug/Release"
if(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  message("-- Configuring ${PROJECT_NAME} for Debug")
  add_definitions(-D__DEBUG__)
  set(CMAKE_CC_FLAGS "${CMAKE_CC_CFLAGS} -Wall -Werror -g -O0 -fPIC")
else(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  set(CMAKE_CC_FLAGS "${CMAKE_CC_CFLAGS} -Wall -Werror -O2 -fPIC")
endif(${CMAKE_BUILD_TYPE} MATCHES "Debug")

# define dependencies path
set(BLOB_LIBS_DIR ${PROJECT_SOURCE_DIR}/../../libs)
set(BLOB_TYPE_DIR ${BLOB_LIBS_DIR}/types)
set(BLOB_PINCHANGEINT_DIR ${PROJECT_SOURCE_DIR}/../../deps/PinChangeInterrupt)


# add include directories (-I)
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${BLOB_TYPE_DIR}/include)
include_directories(${BLOB_PINCHANGEINT_DIR}/src) 

# link include directories (-L)
link_directories(${BLOB_PINCHANGEINT_DIR}/lib)

# libs to link
set(PWM_EXTRA_LIBS pcint)

# sources
set(I2C_SRC src/i2c.cpp)
set(PWM_SRC src/pwm.cpp)

# output files path: libs at /lib and executables at bin/
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# build the shared and static libraries
if(${PLATFORM} MATCHES "Arduino")
  set(ARDUINO_DEFAULT_BOARD leonardo) # Default Board ID, when not specified
  set(ARDUINO_DEFAULT_PORT /dev/ttyACM0) # Default Port, when not specified
  generate_arduino_library(blob_i2c
    SRCS ${I2C_SRC}
    BOARD leonardo)
  generate_arduino_library(blob_pwm
    SRCS ${PWM_SRC}
    LIBS ${PWM_EXTRA_LIBS}
    BOARD leonardo)
else(${PLATFORM} MATCHES "Arduino")
  add_library(blob_i2c SHARED ${I2C_SRC})
  add_library(blob_i2c_static STATIC ${I2C_SRC})
  add_library(blob_pwm SHARED ${PWM_SRC})
  add_library(blob_pwm_static STATIC ${PWM_SRC})
endif(${PLATFORM} MATCHES "Arduino")

# compile external dependencies
add_subdirectory(${BLOB_PINCHANGEINT_DIR} "${CMAKE_CURRENT_BINARY_DIR}/pcint")

# compile tests and blob dependencies only if standalone compilation
string(FIND ${CMAKE_BINARY_DIR} ${PROJECT_NAME} IS_PROJECT) 
if("${IS_PROJECT}" GREATER -1)
  # compile tests
  add_subdirectory(test)
endif("${IS_PROJECT}" GREATER -1)

