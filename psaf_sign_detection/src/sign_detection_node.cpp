/**
 * @file sign_detection_node.cpp
 * @brief the implementation of the SignDetectionNode class
 * @author PSAF
 * @date 2022-06-01
 */
#include "psaf_sign_detection/sign_detection_node.hpp"
#include <vector>
#include <string>

SignDetectionNode::SignDetectionNode()
: SignDetectionInterface(
    SIGN_DETECTION_NODE_NAME,
    NBR_OF_CAMS_RGB,
    CAM_TOPIC_RGB,
    STATE_TOPIC,
    SIGN_TOPIC,
    STATUS_INFO_TOPIC,
    rclcpp::QoS(rclcpp::KeepLast {10})
) {}

void SignDetectionNode::processImage(cv::Mat & img, int sensor)
{
  (void) img;
  (void) sensor;
}

void SignDetectionNode::updateState(std_msgs::msg::Int64::SharedPtr state)
{
  (void) state;
}

void SignDetectionNode::update()
{
}
