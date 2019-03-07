#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <https://www.gnu.org/licenses/>.


cmake_minimum_required(VERSION 2.8.12)
project(inertia_estimator)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

add_definitions(-std=c++11)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    nav_msgs
    sensor_msgs
    message_generation
)

include_directories(
    include/inertia_estimator
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

add_message_files(
    FILES
    ParameterEstimates.msg
    MotorRPM.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
)

catkin_package(
    CATKIN_DEPENDS roscpp geometry_msgs nav_msgs sensor_msgs message_runtime
    DEPENDS EIGEN3
)

set(ESTIMATOR_SOURCE_FILES
    src/inertia_estimator.cpp
    src/v_tools.cpp
    include/inertia_estimator/calc_EKF_F_optimized/calc_EKF_F_optimized.cpp
    include/inertia_estimator/calc_EKF_F_optimized/rt_nonfinite.cpp
    include/inertia_estimator/calc_EKF_F_optimized/rtGetInf.cpp
    include/inertia_estimator/calc_EKF_F_optimized/rtGetNaN.cpp
    include/inertia_estimator/calc_EKF_F_optimized/sign.cpp
    include/inertia_estimator/calc_EKF_F_optimized/sqrt.cpp
    include/inertia_estimator/calc_EKF_H_imu_optimized_simple/calc_EKF_H_imu_optimized_simple.cpp
    include/inertia_estimator/calc_EKF_H_odom_optimized_simple/calc_EKF_H_odom_optimized_simple.cpp
)


set(ESTIMATOR_HEADER_FILES
    include/inertia_estimator/inertia_estimator.h
    include/inertia_estimator/v_tools.h
    include/inertia_estimator/calc_EKF_F_optimized/calc_EKF_F_optimized.h
    include/inertia_estimator/calc_EKF_F_optimized/rt_nonfinite.h
    include/inertia_estimator/calc_EKF_F_optimized/rtGetInf.h
    include/inertia_estimator/calc_EKF_F_optimized/rtGetNaN.h
    include/inertia_estimator/calc_EKF_F_optimized/sign.h
    include/inertia_estimator/calc_EKF_F_optimized/sqrt.h
    include/inertia_estimator/calc_EKF_H_imu_optimized_simple/calc_EKF_H_imu_optimized_simple.h
    include/inertia_estimator/calc_EKF_H_odom_optimized_simple/calc_EKF_H_odom_optimized_simple.h
)


###########
## Build ##
###########
add_library(inertia_estimator_lib ${ESTIMATOR_SOURCE_FILES})
target_link_libraries(inertia_estimator_lib ${catkin_LIBRARIES} ${EIGEN_LIBS})
add_executable(inertia_estimator  src/inertia_estimator_node.cpp)
add_dependencies(inertia_estimator ${catkin_EXPORTED_TARGETS} )
target_link_libraries(inertia_estimator inertia_estimator_lib ${catkin_LIBRARIES} ${EIGEN_LIBS})
