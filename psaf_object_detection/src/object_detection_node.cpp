/**
 * @file object_detection_node.cpp
 * @brief The ObjectDetectionNode class implementation
 * @author PSAF
 * @date 2022-06-01
 */
#include "psaf_object_detection/object_detection_node.hpp"
#include <vector>
#include <string>

ObjectDetectionNode::ObjectDetectionNode()
: ObjectDetectionInterface(
    OBJECT_DETECTION_NODE_NAME,
    NBR_OF_CAMS,
    NBR_OF_US_SENSORS,
    CAM_TOPICS,
    US_TOPICS,
    STATE_TOPIC,
    OBJECT_TOPIC,
    STATUS_INFO_TOPIC,
    rclcpp::QoS(rclcpp::KeepLast {10})
) {}

void ObjectDetectionNode::update()
{
}

void ObjectDetectionNode::processImage(cv::Mat & img, int sensor)
{
  (void) img;
  (void) sensor;
}

void ObjectDetectionNode::updateState(std_msgs::msg::Int64::SharedPtr state)
{
  (void) state;
}

void ObjectDetectionNode::updateSensorValue(sensor_msgs::msg::Range::SharedPtr p, int sensor)
{
  (void) p;
  (void) sensor;
}
