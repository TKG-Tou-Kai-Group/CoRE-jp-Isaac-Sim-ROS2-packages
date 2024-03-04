cmake_minimum_required(VERSION 3.8)
project(core_jp_camera_viewer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(OpenGL_GL_PREFERENCE LEGACY)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(Qt5 COMPONENTS Core Gui Widgets OpenGL REQUIRED)
include_directories(
  ${QT_USE_FILE}
)

#you need to add your header files in add_executable, otherwise the moc won't parse them
add_executable(viewer
  src/viewer.cpp src/mainwindow.cpp
  src/mygraphicsview.cpp
  include/core_jp_camera_viewer/mainwindow.h
  include/core_jp_camera_viewer/mygraphicsview.h
   src/mainwindow.ui resource/resource.qrc)
ament_target_dependencies(viewer rclcpp sensor_msgs cv_bridge OpenCV OpenGL GLUT
  Qt5Core Qt5Gui Qt5Widgets Qt5OpenGL)
target_include_directories(viewer
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${CMAKE_CURRENT_BINARY_DIR}
    ${OpenGL_INCLUDE_DIRS}
    ${GLUT_INCLUDE_DIRS}
)
target_link_libraries(
  viewer
    ${OPENGL_LIBRARIES}
    ${GLUT_LIBRARY}
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(TARGETS viewer
  DESTINATION lib/${PROJECT_NAME})

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
