// Copyright 2022 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using namespace std::chrono_literals;

std::shared_ptr<rclcpp::Node> node;
bool common_goal_accepted = false;
bool action_is_sent = false;
rclcpp_action::ResultCode common_resultcode = rclcpp_action::ResultCode::UNKNOWN;
int common_action_result_code = control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL;

void common_goal_response(
  rclcpp_action::ClientGoalHandle
  <control_msgs::action::FollowJointTrajectory>::SharedPtr future)
{
  RCLCPP_DEBUG(
    node->get_logger(), "common_goal_response time: %f",
    rclcpp::Clock().now().seconds());
  auto goal_handle = future.get();
  if (!goal_handle) {
    common_goal_accepted = false;
    printf("Goal rejected\n");
  } else {
    common_goal_accepted = true;
    printf("Goal accepted\n");
  }
}

void common_result_response(
  const rclcpp_action::ClientGoalHandle
  <control_msgs::action::FollowJointTrajectory>::WrappedResult & result)
{
  printf("common_result_response time: %f\n", rclcpp::Clock().now().seconds());
  common_resultcode = result.code;
  common_action_result_code = result.result->error_code;
  action_is_sent = false;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      printf("SUCCEEDED result code\n");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      printf("Goal was aborted\n");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      printf("Goal was canceled\n");
      return;
    default:
      printf("Unknown result code\n");
      return;
  }
}

void common_feedback(
  rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr,
  const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback)
{
  std::cout << "feedback->desired.positions :";
  for (auto & x : feedback->desired.positions) {
    std::cout << x << "\t";
  }
  std::cout << std::endl;
  std::cout << "feedback->desired.velocities :";
  for (auto & x : feedback->desired.velocities) {
    std::cout << x << "\t";
  }
  std::cout << std::endl;
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  node = std::make_shared<rclcpp::Node>("controller");

  auto publisher = node->create_publisher<geometry_msgs::msg::Twist>(
    "/diff_drive_base_controller/cmd_vel_unstamped", 10);
  auto velocity_publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/velocity_controller/commands", 10);

  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr action_client;
  action_client = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
    node->get_node_base_interface(),
    node->get_node_graph_interface(),
    node->get_node_logging_interface(),
    node->get_node_waitables_interface(),
    "/joint_trajectory_controller/follow_joint_trajectory");

  bool response =
    action_client->wait_for_action_server(std::chrono::seconds(1));
  if (!response) {
    throw std::runtime_error("could not get action server");
  }

  RCLCPP_INFO(node->get_logger(), "node created");

  geometry_msgs::msg::Twist command;

  command.linear.x = 0.1;
  command.linear.y = 0.0;
  command.linear.z = 0.0;

  command.angular.x = 0.0;
  command.angular.y = 0.0;
  command.angular.z = 0.0;

  std_msgs::msg::Float64MultiArray velocity_commands;

  velocity_commands.data.resize(3);
  for (int speed = 5; speed <= 250; speed += 5)
  {
    velocity_commands.data[0] = speed;
    velocity_commands.data[1] = speed;
    velocity_commands.data[2] = speed;
    velocity_publisher->publish(velocity_commands);
    std::this_thread::sleep_for(100ms);
  }

  std::vector<std::string> joint_names = {"shooter_shaft_link_joint", "shooter_mag_link_joint"};

  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points;

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.time_from_start = rclcpp::Duration::from_seconds(0.0);  // start asap
  point.positions.resize(joint_names.size());
  point.positions[0] = 0.0;
  point.positions[1] = 0.0;

  trajectory_msgs::msg::JointTrajectoryPoint point2;
  point2.time_from_start = rclcpp::Duration::from_seconds(0.5);  // start asap
  point2.positions.resize(joint_names.size());
  point2.positions[0] = 0.0;
  point2.positions[1] = 0.0;

  points.push_back(point);
  points.push_back(point2);

  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions opt;
  opt.goal_response_callback = std::bind(common_goal_response, std::placeholders::_1);
  opt.result_callback = std::bind(common_result_response, std::placeholders::_1);
  opt.feedback_callback = std::bind(common_feedback, std::placeholders::_1, std::placeholders::_2);

  control_msgs::action::FollowJointTrajectory_Goal goal_msg;
  goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(0.5);
  goal_msg.trajectory.joint_names = joint_names;
  goal_msg.trajectory.points = points;
  
  float mag_position = 0.0;
  while (1) {
    publisher->publish(command);
    velocity_publisher->publish(velocity_commands);
    if (action_is_sent == false)
    {
      action_is_sent = true;
      goal_msg.trajectory.points[0].positions[1] = mag_position;
      goal_msg.trajectory.points[1].positions[1] = mag_position;
      mag_position += M_PI / 10;
      auto goal_handle_future = action_client->async_send_goal(goal_msg, opt);
    }
    std::this_thread::sleep_for(100ms);
    rclcpp::spin_some(node);
  }
  rclcpp::shutdown();

  return 0;
}
