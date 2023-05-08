/**
 * @file parking_node.cpp
 * @brief The implementation of the parking node
 * @author PSAF
 * @date 2022-06-01
 */
#include "psaf_parking/parking_node.hpp"
#include <vector>
#include <string>

ParkingNode::ParkingNode()
: ParkingInterface(
    PARKING_NODE_NAME,
    PARKING_SPOT_TOPIC,
    NBR_OF_US_SENSORS,
    US_TOPICS,
    STATE_TOPIC,
    SET_STEERING_TOPIC,
    SET_SPEED_TOPIC,
    STATUS_INFO_TOPIC,
    SET_LIGHT_TOPIC,
    rclcpp::QoS(rclcpp::KeepLast {10})
) {}

void ParkingNode::updateSensorValue(sensor_msgs::msg::Range::SharedPtr p, int sensor)
{
  (void) p;
  (void) sensor;
}

void ParkingNode::updateState(std_msgs::msg::Int64::SharedPtr state)
{
  (void) state;
}

void ParkingNode::processParkingSpot(libpsaf_msgs::msg::ParkingSpot::SharedPtr p)
{
  (void) p;
}

void ParkingNode::update()
{
}
