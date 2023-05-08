/**
 * @file manual_mode_node.cpp
 * @brief This file contains the implementation of the ManualModeNode class.
 * @author PSAF
 * @date 2022-06-01
 */
#include "psaf_manual_mode/manual_mode_node.hpp"
#include <string>

ManualModeNode::ManualModeNode()
: ManualModeInterface(
    MANUAL_MODE_NODE_NAME,
    STATE_TOPIC,
    MANUAL_SIGNALS_TOPIC,
    GET_STEERING_TOPIC,
    GET_SPEED_TOPIC,
    STATUS_INFO_TOPIC,
    rclcpp::QoS(rclcpp::KeepLast {10})
) {}

void ManualModeNode::updateState(std_msgs::msg::Int64::SharedPtr state)
{
  (void) state;
}

void ManualModeNode::processManualSignals(std_msgs::msg::UInt8::SharedPtr msignal)
{
  (void) msignal;
}

void ManualModeNode::updateSteering(std_msgs::msg::Int16::SharedPtr p)
{
  (void) p;
}

void ManualModeNode::updateSpeed(std_msgs::msg::Int16::SharedPtr p)
{
  (void) p;
}

void ManualModeNode::update()
{
}
