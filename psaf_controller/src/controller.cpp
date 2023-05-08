/**
 * @file controller.cpp
 * @brief the main method for the controller. This function gets called by the launch file
 * @author PSAF
 * @date 2022-06-01
 */

// test
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "psaf_controller/controller_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<ControllerNode> node = std::make_shared<ControllerNode>();
  rclcpp::WallRate rate(node->declare_parameter("update_frequency", rclcpp::PARAMETER_DOUBLE)
    .get<rclcpp::PARAMETER_DOUBLE>());

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    node->update();
    rate.sleep();
  }

  rclcpp::shutdown();
}
