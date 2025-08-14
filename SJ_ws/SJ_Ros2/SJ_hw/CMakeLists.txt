cmake_minimum_required(VERSION 3.8)
project(SJ_hw)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)# HardInter0. 硬件接口包
find_package(pluginlib REQUIRED)# HardInter0. 插件库包
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(PkgConfig REQUIRED)# 查找包pkg-config工具，提供pkg_check_modules指令
# 检查并导入libserial库，用于串口通信
pkg_check_modules(LIBSERIAL REQUIRED libserial)

# 添加达妙驱动库
add_library(damiao_driver SHARED
  src/damiao.cpp
)

# 设置达妙驱动库的头文件搜索路径
target_include_directories(damiao_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${LIBSERIAL_INCLUDE_DIRS}
)
ament_target_dependencies(damiao_driver
  rclcpp 
)

# HardInter1. 添加机械臂硬件接口库
add_library(dm_hw SHARED
  src/dm_hw.cpp
)
# HardInter1. 设置机械臂硬件接口库的头文件搜索路径
target_include_directories(dm_hw PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# HardInter1. 设置机械臂硬件接口库的依赖项
ament_target_dependencies(dm_hw
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
# HardInter1. 链接达妙驱动库到机械臂硬件接口库
target_link_libraries(dm_hw damiao_driver)

# HardInter2.安装插件声明 XML 文件
install(FILES dm_hw_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# HardInter2. 导出插件描述文件
pluginlib_export_plugin_description_file(hardware_interface dm_hw_plugins.xml)

# HardInter2.安装库文件
install(TARGETS dm_hw damiao_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()