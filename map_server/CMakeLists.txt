cmake_minimum_required(VERSION 3.5)

project(map_server)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)

find_package(SDL REQUIRED)
find_package(SDL_image REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP yaml-cpp REQUIRED)
if(YAMLCPP_VERSION VERSION_GREATER "0.5.0")
  add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
endif()

include_directories(include)

add_library(map_server_image_loader SHARED src/image_loader.cpp)
ament_target_dependencies(map_server_image_loader
  "nav_msgs"
  "tf2"
)
target_include_directories(map_server_image_loader
  PUBLIC ${SDL_IMAGE_INCLUDE_DIRS} ${SDL_INCLUDE_DIR}
)
target_link_libraries(map_server_image_loader
  ${SDL_LIBRARY}
  ${SDL_IMAGE_LIBRARIES}
)

add_executable(map_server src/main.cpp)
ament_target_dependencies(map_server
  "nav_msgs"
  "rclcpp"
  "rcutils"
)
target_include_directories(map_server_image_loader
  PUBLIC ${YAMLCPP_INCLUDE_DIRS}
)
target_link_libraries(map_server
  map_server_image_loader
  ${YAMLCPP_LIBRARIES}
)

add_executable(map_server-map_saver src/map_saver.cpp)
set_target_properties(map_server-map_saver PROPERTIES OUTPUT_NAME map_saver)
ament_target_dependencies(map_server-map_saver
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "tf2"
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_utest test/utest.cpp test/test_constants.cpp)
  target_link_libraries(
    ${PROJECT_NAME}_utest
    map_server_image_loader
    ${SDL_LIBRARY}
    ${SDL_IMAGE_LIBRARIES}
  )

  ament_add_gtest_executable(rtest test/rtest.cpp test/test_constants.cpp)
  ament_target_dependencies(rtest
    "nav_msgs"
    "rclcpp"
  )

  set(MAP_SERVER_EXECUTABLE "$<TARGET_FILE:map_server>")
  set(MAP_SERVER_TESTMAP_YAML "${CMAKE_CURRENT_SOURCE_DIR}/test/testmap.yaml")
  set(RTEST_EXECUTABLE "$<TARGET_FILE:rtest>")
  configure_file(
    test/rtest.launch.py.in
    rtest.launch.py.configured
    @ONLY
  )
  file(GENERATE
    OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/rtest$<CONFIG>.py"
    INPUT "${CMAKE_CURRENT_BINARY_DIR}/rtest.launch.py.configured"
  )
  ament_add_pytest_test(rtest_launch
    "${CMAKE_CURRENT_BINARY_DIR}/rtest$<CONFIG>.py"
    TIMEOUT 15
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(map_server_image_loader)
ament_package()

# Install libraries
install(
  TARGETS map_server_image_loader
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(
  TARGETS map_server-map_saver map_server 
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  PROGRAMS scripts/crop_map
  DESTINATION bin
)
