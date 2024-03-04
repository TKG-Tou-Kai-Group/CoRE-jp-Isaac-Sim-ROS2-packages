// display_camera_image.cpp

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <QApplication>
#include <QLabel>

#include "core_jp_camera_viewer/recieve_image_node.hpp"
#include "core_jp_camera_viewer/mainwindow.h"

int main(int argc, char **argv)
{
  QApplication app(argc, argv);
  MainWindow window(nullptr);
  window.showFullScreen();

  // Initialize ROS
  rclcpp::init(argc, argv);

  // Create subscriber node
  auto node = std::make_shared<RecieveTopicNode>();

  // ROS and application loop
  rclcpp::WallRate loop_rate(20);
  while (rclcpp::ok())
  {
    app.processEvents();
    rclcpp::spin_some(node);
    window.setCameraImage(node->camera_image_);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
