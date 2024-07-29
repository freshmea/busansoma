cmake_minimum_required(VERSION 3.8)
project(my_pkg_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# build
add_executable(helloworld src/helloworld.cpp)

add_executable(helloOpenCV src/helloOpenCV.cpp)
target_link_libraries(helloOpenCV ${OpenCV_LIBS})

add_executable(helloRclcpp src/helloRclcpp.cpp)
ament_target_dependencies(helloRclcpp rclcpp std_msgs)

# install
install(TARGETS
  helloworld
  helloOpenCV
  helloRclcpp
  DESTINATION lib/${PROJECT_NAME}
)

# test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
