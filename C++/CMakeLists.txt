cmake_minimum_required(VERSION 3.0.2)
project(navio)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
	INCLUDE_DIRS Navio
	LIBRARIES ${PROJECT_NAME}
)

include_directories(${catkin_INCLUDE_DIRS} Navio)

add_subdirectory(Navio/pigpio)

# Library
set(LIB_SRC_FILES
	Navio/Common/I2Cdev.cpp
	Navio/Common/MPU9250.cpp
	Navio/Common/MS5611.cpp
	Navio/Common/nav_payloads.cpp
	Navio/Common/Ublox.cpp
	Navio/Common/Util.cpp
	Navio/Common/gpio.cpp
	Navio/Navio+/ADC_Navio.cpp
	Navio/Navio+/ADS1115.cpp
	Navio/Navio+/Led_Navio.cpp
	Navio/Navio+/MB85RC256.cpp
	Navio/Navio+/PCA9685.cpp
	Navio/Navio+/RCInput_Navio.cpp
	Navio/Navio+/RCOutput_Navio.cpp
	Navio/Navio2/ADC_Navio2.cpp
	Navio/Navio2/LSM9DS1.cpp
	Navio/Navio2/Led_Navio2.cpp
	Navio/Navio2/PWM.cpp
	Navio/Navio2/DSHOT.cpp
	Navio/Navio2/RCInput_Navio2.cpp
	Navio/Navio2/RCOutput_Navio2.cpp
	Navio/Navio2/RGBled.cpp
)
add_library(${PROJECT_NAME} STATIC ${LIB_SRC_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Examples
file(GLOB_RECURSE EXAMPLE_SRC_FILES RELATIVE ${PROJECT_SOURCE_DIR} Examples/[^.]*.cpp)
foreach(FILE IN LISTS EXAMPLE_SRC_FILES)
	get_filename_component(NODE_NAME ${FILE} NAME_WE)
	add_executable(${NODE_NAME} ${FILE})
	target_link_libraries(${NODE_NAME} ${catkin_LIBRARIES} ${PROJECT_NAME} pigpio pthread)
endforeach()
