cmake_minimum_required(VERSION 3.5)
project(zivid_camera LANGUAGES CXX)

if(${CMAKE_VERSION} VERSION_LESS "3.8.0")
  add_compile_options(-std=c++17)
else()
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

option(USE_COMPILER_WARNING_OPTIONS off)

if(USE_COMPILER_WARNING_OPTIONS)
    include(CompilerWarningOptions NO_POLICY_SCOPE)
endif(USE_COMPILER_WARNING_OPTIONS)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  tf2_ros
  dynamic_reconfigure
  message_generation
  image_transport
  nodelet
)

find_package(Zivid COMPONENTS Core REQUIRED)
find_package(OpenMP)
if(OpenMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

set(LIBRARY_NAME ${PROJECT_NAME})

set(SETTINGS_GENERATOR_TARGET_NAME ${PROJECT_NAME}_settings_generator)
add_executable(${SETTINGS_GENERATOR_TARGET_NAME} src/settings_generator.cpp)
target_link_libraries(${SETTINGS_GENERATOR_TARGET_NAME} PRIVATE Zivid::Core)

################################################
## Declare ROS messages, services and actions ##
################################################

generate_dynamic_reconfigure_options(
  ${CMAKE_CURRENT_BINARY_DIR}/CaptureGeneral.cfg
  ${CMAKE_CURRENT_BINARY_DIR}/CaptureFrame.cfg
)

add_service_files(
  DIRECTORY
  srv
  FILES
  Capture.srv
  CameraInfoModelName.srv
  CameraInfoSerialNumber.srv
)

set(GENERATOR_TARGET_NAME ${PROJECT_NAME}_generator)
add_custom_target(
  ${GENERATOR_TARGET_NAME}
  COMMAND mkdir -p ${CMAKE_CURRENT_BINARY_DIR}/state_srv_files
  COMMAND mkdir -p ${CMAKE_CURRENT_BINARY_DIR}/generated_headers
  COMMAND ${SETTINGS_GENERATOR_TARGET_NAME}
  COMMAND chmod a+x ${CMAKE_CURRENT_BINARY_DIR}/CaptureFrame.cfg
  COMMAND chmod a+x ${CMAKE_CURRENT_BINARY_DIR}/CaptureGeneral.cfg
  BYPRODUCTS ${CMAKE_CURRENT_BINARY_DIR}/CaptureFrame.cfg
             ${CMAKE_CURRENT_BINARY_DIR}/CaptureGeneral.cfg
             ${CMAKE_CURRENT_BINARY_DIR}/generated_headers/CaptureFrameConfigUtils.h
             ${CMAKE_CURRENT_BINARY_DIR}/generated_headers/CaptureGeneralConfigUtils.h
  COMMENT "Running generator"
)

add_dependencies(${catkin_EXPORTED_TARGETS} ${GENERATOR_TARGET_NAME})
add_dependencies(${PROJECT_NAME}_gencfg ${GENERATOR_TARGET_NAME})

generate_messages(
  DEPENDENCIES
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  LIBRARIES ${LIBRARY_NAME}
  CATKIN_DEPENDS message_runtime sensor_msgs std_msgs nodelet
)

###########
## Build ##
###########

function(set_compile_options target)
  if($ENV{ROS_DISTRO} STREQUAL "kinetic")
    # Kinetic (16.04) unfortunately has some headers that fail to  compile
    # with extended warnings.
    target_compile_options(${target} PRIVATE -Werror)
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Werror -pedantic)
  endif()
endfunction()

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/generated_headers/
  ${catkin_INCLUDE_DIRS}
)

add_library(${LIBRARY_NAME} src/zivid_camera.cpp)
set_compile_options(${LIBRARY_NAME})
target_compile_definitions(${LIBRARY_NAME} PRIVATE "ZIVID_ROS_DRIVER_VERSION=\"${${PROJECT_NAME}_VERSION}\"")
target_link_libraries(${LIBRARY_NAME} PUBLIC ${catkin_LIBRARIES})
target_link_libraries(${LIBRARY_NAME} PRIVATE Zivid::Core)
add_dependencies(${LIBRARY_NAME} ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

set(NODE_NAME ${PROJECT_NAME}_node)
add_executable(${NODE_NAME} src/node.cpp)
set_compile_options(${NODE_NAME})
target_link_libraries(${NODE_NAME} ${LIBRARY_NAME} ${catkin_LIBRARIES})
add_dependencies(${NODE_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${LIBRARY_NAME} )

set(NODELET_NAME ${PROJECT_NAME}_nodelet)
add_library(${NODELET_NAME} src/nodelet.cpp)
target_link_libraries(${NODELET_NAME} ${LIBRARY_NAME} ${catkin_LIBRARIES})
set_compile_options(${NODELET_NAME})
add_dependencies(${NODELET_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${LIBRARY_NAME} )

#############
## Install ##
#############

install(
  TARGETS ${LIBRARY_NAME} ${NODE_NAME} ${NODELET_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(${PROJECT_NAME}_test test/test_zivid_camera.test test/test_zivid_camera.cpp)
  target_link_libraries(${PROJECT_NAME}_test ${LIBRARY_NAME} Zivid::Core ${catkin_LIBRARIES})
endif()
