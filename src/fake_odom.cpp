#include "fake_odom/fake_odom.hpp"
#include <memory>
#include <string>
#include <cmath>
#include <iostream>

using m2labo::mobilemover::FakeOdom;
using namespace std::chrono_literals;

FakeOdom::FakeOdom()
: Node("mm_node", rclcpp::NodeOptions().use_intra_process_comms(true))
{
  
  RCLCPP_INFO(get_logger(), "Init FAKE ODOM Node Main");
  MmJoy_.x=0.0;
  MmJoy_.y=0.0;
  RCLCPP_INFO(this->get_logger(), "Run!");
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("wheel/odometry", rclcpp::QoS(10));
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  odom_timer_callback(std::chrono::milliseconds(1));
  cmd_vel_callback();

}

void FakeOdom::cmd_vel_callback()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel",
    qos,
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) -> void
    {
      MmJoy_.x=msg->linear.x;
      MmJoy_.y=msg->angular.z;
      RCLCPP_DEBUG(
        this->get_logger(),
        "lin_vel: %f ang_vel: %f", msg->linear.x, msg->angular.z);
    }
  );
}


void FakeOdom::odom_timer_callback(const std::chrono::milliseconds timeout)
{
  odom_timer_ = this->create_wall_timer(
    timeout,
    [this]() -> void
    {
      rclcpp::Time time_now = rclcpp::Clock().now();
      double dt = (time_now - time_past).seconds();
      
      Vf=MmJoy_.x;
      Wz=MmJoy_.y;
      theta_od +=Wz*dt;
            
      x_od += Vf*dt*std::cos(theta_od);
      y_od += Vf*dt*std::sin(theta_od);
      auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();

      odom_msg->header.frame_id = "odom";
      odom_msg->child_frame_id = "base_footprint";
      odom_msg->header.stamp = time_now;
      odom_msg->pose.pose.position.x = x_od;
      odom_msg->pose.pose.position.y = y_od;
      odom_msg->pose.pose.position.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, theta_od);
      odom_msg->pose.pose.orientation.x = q.x();
      odom_msg->pose.pose.orientation.y = q.y();
      odom_msg->pose.pose.orientation.z = q.z();
      odom_msg->pose.pose.orientation.w = q.w();
      odom_msg->twist.twist.linear.x = Vf;
      odom_msg->twist.twist.angular.z = Wz;
      odom_pub_->publish(std::move(odom_msg));
      geometry_msgs::msg::TransformStamped odom_tf;
      odom_tf.transform.translation.x = x_od;
      odom_tf.transform.translation.y = y_od;
      odom_tf.transform.translation.z = 0.0;
      odom_tf.transform.rotation.x = q.x();
      odom_tf.transform.rotation.y = q.y();
      odom_tf.transform.rotation.z = q.z();
      odom_tf.transform.rotation.w = q.w();
      odom_tf.header.frame_id = "odom";
      odom_tf.child_frame_id = "base_footprint";
      odom_tf.header.stamp = time_now;
      tf_broadcaster_->sendTransform(odom_tf);
      time_past = time_now;
    }
  );
}