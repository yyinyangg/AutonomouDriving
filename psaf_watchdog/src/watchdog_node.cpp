/**
 * @file watchdog_node.cpp
 * @brief The implementation of the watchdog node
 * @author PSAF
 * @date 2022-06-01
 */
#include "psaf_watchdog/watchdog_node.hpp"
#include <string>
#include <vector>

WatchdogNode::WatchdogNode()
: WatchdogInterface(
    WATCHDOG_NODE_NAME,
    NBR_OF_CAMS,
    NBR_OF_US_SENSORS,
    CAM_TOPICS,
    US_TOPICS,
    GET_SPEED_TOPIC,
    GET_STEERING_TOPIC,
    STATE_TOPIC,
    WATCHDOG_ERROR_TOPIC,
    STATUS_INFO_TOPIC,
    rclcpp::QoS(rclcpp::KeepLast {10})
) {}

void WatchdogNode::processImage(cv::Mat & image, int sensor)
{
  (void) image;
  (void) sensor;
}

void WatchdogNode::updateSpeed(std_msgs::msg::Int16::SharedPtr speed)
{
  (void) speed;
}

void WatchdogNode::updateSteering(std_msgs::msg::Int16::SharedPtr steering)
{
  (void) steering;
}

void WatchdogNode::updateSensorValue(sensor_msgs::msg::Range::SharedPtr p, int sensor)
{
  (void) p;
  (void) sensor;
}

void WatchdogNode::updateState(std_msgs::msg::Int64::SharedPtr state)
{
  (void) state;
}

void WatchdogNode::update() {}
