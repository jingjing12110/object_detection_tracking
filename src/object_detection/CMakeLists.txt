cmake_minimum_required(VERSION 2.8.3)
project(object_detection)
add_compile_options(-std=c++11)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED
             COMPONENTS roscpp
                        pcl_ros
                        tf
                        sensor_msgs
                        lidar_msgs
                        tracking_msgs
                        pcl_conversions
                        message_generation)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS std_msgs sensor_msgs message_runtime)

#
# Build ##
#

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

add_library(lib_name_object
            # src/ground_filter/ground_filter.cpp
            src/ground_filter/ray_ground_filter.cpp
            # src/ground_filter/piecewise_plane_fitting.cpp
            # src/segment/bb_supervoxel_segmentor.cpp
            # src/segment/point_cloud_segmenter.cpp
            # src/cluster/DBSCAN.cpp
            src/tracking/tracking.cpp
            src/tracking/ukf.cpp
            src/tracking/imm_ukf_pda.cpp
            src/postprocess/convex_hull.cpp
            src/postprocess/min_box.cpp)

add_executable(velo32_perception src/main.cpp
               # src/main_cs.cpp
               # src/main_vlp32.cpp
               )

target_link_libraries(velo32_perception
                      lib_name_object
                      ${catkin_LIBRARIES}
                      ${OpenCV_LIBRARIES})

add_dependencies(velo32_perception ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
