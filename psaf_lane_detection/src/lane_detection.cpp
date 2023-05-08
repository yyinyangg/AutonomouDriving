/**
 * @file lane_detection.cpp
 * @brief the main method for the lane_detection. This function gets called by the launch file
 * @author PSAF
 * @date 2022-06-01
 */
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "psaf_lane_detection/lane_detection_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<LaneDetectionNode> node = std::make_shared<LaneDetectionNode>();
  rclcpp::WallRate rate(node->declare_parameter("update_frequency", rclcpp::PARAMETER_DOUBLE)
    .get<rclcpp::PARAMETER_DOUBLE>());
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    node->update();
    rate.sleep();
  }
}
