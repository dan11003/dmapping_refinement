cmake_minimum_required(VERSION 3.0.2)
project(dmapping_refinement)

set(CMAKE_BUILD_TYPE "release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  lio_sam
  floam
  rosbag
  std_msgs
  tf
  eigen_conversions
  tf_conversions
  dynamic_reconfigure
)

#add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)
find_package(catkin REQUIRED )
find_package(GTSAM REQUIRED QUIET)

generate_dynamic_reconfigure_options(
        cfg/calibParams.cfg
)


find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)


find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()




include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(
  include
  ${PCL_LIBRARY_DIRS}
  ${CERES_LIBRARY_DIRS}
    ${GTSAM_LIBRARY_DIRS}
)


catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs lio_sam floam
  DEPENDS EIGEN3 PCL Ceres GTSAM
  INCLUDE_DIRS include
)


set (${PROJECT_NAME}_LIB_SRCS
  src/dmapping_refinement/dutils.cpp
  src/dmapping_refinement/fuser.cpp
  src/dmapping_refinement/registration.cpp
)
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_LIB_SRCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam)



add_executable(refinement_node src/refinement_node.cpp)
target_link_libraries(refinement_node ${PROJECT_NAME} ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} gtsam ) #${PROJECT_NAME}_ODOMEST_LIB ${PROJECT_NAME}
add_dependencies(refinement_node ${catkin_EXPORTED_TARGETS})

#add_executable(testopti src/test_opti.cpp)
#target_link_libraries(testopti ${PROJECT_NAME} ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ) #${PROJECT_NAME}_ODOMEST_LIB ${PROJECT_NAME}
#add_dependencies(testopti ${catkin_EXPORTED_TARGETS})

add_executable(calib src/calib.cpp)
target_link_libraries(calib ${PROJECT_NAME} ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} gtsam)
add_dependencies(calib ${catkin_EXPORTED_TARGETS})
