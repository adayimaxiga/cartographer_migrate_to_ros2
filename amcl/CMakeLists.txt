cmake_minimum_required(VERSION 3.5)
project(amcl)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_ros REQUIRED)
set(required_dependencies
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "rcutils"
  "sensor_msgs"
  "std_srvs"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_msgs"
  "tf2_ros"
)
ament_auto_find_build_dependencies(REQUIRED ${required_dependencies})

# dynamic reconfigure
# generate_dynamic_reconfigure_options(
#     cfg/AMCL.cfg
# )

include_directories(include/amcl include/amcl/map include/amcl/sensors include/amcl/pf)

ament_auto_add_library(amcl_pf
                    src/amcl/pf/pf.c
                    src/amcl/pf/pf_kdtree.c
                    src/amcl/pf/pf_pdf.c
                    src/amcl/pf/pf_vector.c
                    src/amcl/pf/eig3.c
                    src/amcl/pf/pf_draw.c)

ament_auto_add_library(amcl_map
                    src/amcl/map/map.c
                    src/amcl/map/map_cspace.cpp
                    src/amcl/map/map_range.c
                    src/amcl/map/map_store.c
                    src/amcl/map/map_draw.c)

ament_target_dependencies(amcl_map)
ament_auto_add_library(amcl_sensors
                    src/amcl/sensors/amcl_sensor.cpp
                    src/amcl/sensors/amcl_odom.cpp
                    src/amcl/sensors/amcl_laser.cpp)


ament_auto_add_executable(amcl
                       src/amcl_node.cpp)
# add_dependencies(amcl amcl_gencfg)

ament_target_dependencies(amcl
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "sensor_msgs"
  "std_srvs"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_msgs"
  "tf2_ros")

target_link_libraries(amcl
    amcl_sensors amcl_map amcl_pf
)

install(DIRECTORY examples/
    DESTINATION share/${PROJECT_NAME}/examples
)

## Configure Tests
#if(CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#
#  # Bags
#  catkin_download_test_data(${PROJECT_NAME}_basic_localization_stage_indexed.bag
#    http://download.ros.org/data/amcl/basic_localization_stage_indexed.bag
#    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
#    MD5 41fe43af189ec71e5e48eb9ed661a655)
#  catkin_download_test_data(${PROJECT_NAME}_global_localization_stage_indexed.bag
#    http://download.ros.org/data/amcl/global_localization_stage_indexed.bag
#    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
#    MD5 752f711cf4f6e8d1d660675e2da096b0)
#  catkin_download_test_data(${PROJECT_NAME}_small_loop_prf_indexed.bag
#    http://download.ros.org/data/amcl/small_loop_prf_indexed.bag
#    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
#    MD5 e4ef0fc006872b43f12ed8a7ce7dcd81)
#  catkin_download_test_data(${PROJECT_NAME}_small_loop_crazy_driving_prg_indexed.bag
#    http://download.ros.org/data/amcl/small_loop_crazy_driving_prg_indexed.bag
#    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
#    MD5 4a58d1a7962914009d99000d06e5939c)
#  catkin_download_test_data(${PROJECT_NAME}_texas_greenroom_loop_indexed.bag
#    http://download.ros.org/data/amcl/texas_greenroom_loop_indexed.bag
#    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
#    MD5 6e3432115cccdca1247f6c807038e13d)
#  catkin_download_test_data(${PROJECT_NAME}_texas_willow_hallway_loop_indexed.bag
#    http://download.ros.org/data/amcl/texas_willow_hallway_loop_indexed.bag
#    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
#    MD5 27deb742fdcd3af44cf446f39f2688a8)
#  catkin_download_test_data(${PROJECT_NAME}_rosie_localization_stage.bag
#    http://download.ros.org/data/amcl/rosie_localization_stage.bag
#    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
#    MD5 3347bf3835724cfa45e958c5c1846066)
#
#  # Maps
#  catkin_download_test_data(${PROJECT_NAME}_willow-full.pgm
#    http://download.ros.org/data/amcl/willow-full.pgm
#    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
#    MD5 b84465cdbbfe3e2fb9eb4579e0bcaf0e)
#  catkin_download_test_data(${PROJECT_NAME}_willow-full-0.05.pgm
#    http://download.ros.org/data/amcl/willow-full-0.05.pgm
#    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
#    MD5 b61694296e08965096c5e78611fd9765)
#
#  # Tests
#  add_rostest(test/set_initial_pose.xml)
#  add_rostest(test/set_initial_pose_delayed.xml)
#  add_rostest(test/basic_localization_stage.xml)
#  add_rostest(test/small_loop_prf.xml)
#  add_rostest(test/small_loop_crazy_driving_prg.xml)
#  add_rostest(test/texas_greenroom_loop.xml)
#  add_rostest(test/rosie_multilaser.xml)
#  add_rostest(test/texas_willow_hallway_loop.xml)
#
## Not sure when or if this actually passed.
##
## The point of this is that you start with an even probability
## distribution over the whole map and the robot localizes itself after
## some number of iterations of sensing and motion.
##
##  add_rostest(test/global_localization_stage.xml)
#endif()
ament_auto_package()
