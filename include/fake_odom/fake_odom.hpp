// Copyright 2022 M2Labo CO., LTD.

#ifndef FAKE_NODE__FAKENODE_HPP_
#define FAKE_NODE__FAKENODE_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <fake_odom/fake_odom.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace m2labo
{
namespace mobilemover
{
class FakeOdom : public rclcpp::Node
{
public:
  typedef struct
  {
    float x;
    float y;
  } MmJoy;

  explicit FakeOdom();
  virtual ~FakeOdom() {}


private:
  void cmd_vel_callback();
  void odom_timer_callback(std::chrono::milliseconds(timeout));  

  rclcpp::TimerBase::SharedPtr odom_timer_;
  
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Time time_past = rclcpp::Clock().now();

  MmJoy MmJoy_;
  float Vf;
  float Wz;
  float theta_od;
  float x_od;
  float y_od;
};
}  // namespace mobilemover
}  // namespace m2labo
#endif  // FAKE_NODE__FAKENODE_HPP_