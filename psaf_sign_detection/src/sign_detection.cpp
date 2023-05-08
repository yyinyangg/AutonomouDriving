/**
 * @file sign_detection.cpp
 * @brief the main method for the sign detection. This function gets called by the launch file
 * @author PSAF
 * @date 2022-06-01
 */
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "psaf_sign_detection/sign_detection_node.hpp"

/**
* Main: Start the Sign Detection Node
*/

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<SignDetectionNode> node = std::make_shared<SignDetectionNode>();
  rclcpp::WallRate rate(node->declare_parameter("update_frequency", rclcpp::PARAMETER_DOUBLE)
    .get<rclcpp::ParameterType::PARAMETER_DOUBLE>());
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    node->update();
    rate.sleep();
  }

  rclcpp::shutdown();
}
