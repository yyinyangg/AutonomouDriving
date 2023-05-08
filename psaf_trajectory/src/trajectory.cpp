/**
 * @file trajectory.cpp
 * @brief the main method for the trajectory node. This function gets called by the launch file
 * @author PSAF
 * @date 2022-06-01
 */
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "psaf_trajectory/trajectory_node.hpp"

/**
* Main: Start the Startbox Node
*/

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<TrajectoryNode> node = std::make_shared<TrajectoryNode>();
  rclcpp::WallRate rate(node->declare_parameter("update_frequency", rclcpp::PARAMETER_DOUBLE)
    .get<rclcpp::PARAMETER_DOUBLE>());
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    node->update();
    rate.sleep();
  }

  rclcpp::shutdown();
}
