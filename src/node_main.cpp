#include <rclcpp/rclcpp.hpp>
#include "fake_odom/fake_odom.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto fake_odom = std::make_shared<m2labo::mobilemover::FakeOdom>();
  executor.add_node(fake_odom);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}