cmake_minimum_required(VERSION 3.5)
project(my_cpp_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(GTest REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall)
endif()

include_directories(include)

# ライブラリとしてノードを作成
add_library(simple_node_lib src/simple_node.cpp)
ament_target_dependencies(simple_node_lib rclcpp std_msgs)

# 実行バイナリ
add_executable(simple_node_exec src/main.cpp)
target_link_libraries(simple_node_exec simple_node_lib)
ament_target_dependencies(simple_node_exec rclcpp std_msgs)
install(TARGETS simple_node_exec DESTINATION lib/${PROJECT_NAME})

# テストバイナリ
add_executable(test_simple_node test/test_simple_node.cpp)
target_link_libraries(test_simple_node simple_node_lib GTest::gtest GTest::gtest_main)
ament_target_dependencies(test_simple_node rclcpp std_msgs)
add_test(NAME test_simple_node COMMAND test_simple_node)

ament_package()
