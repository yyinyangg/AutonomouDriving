/**
 * @file parking_node.hpp
 * @brief The ParkingNode class definition
 * @author PSAF
 * @date 2022-06-01
 */
#ifndef PSAF_PARKING__PARKING_NODE_HPP_
#define PSAF_PARKING__PARKING_NODE_HPP_

#include <string>
#include <vector>

#include "opencv4/opencv2/opencv.hpp"
#include "libpsaf/interface/parking_interface.hpp"
#include "psaf_configuration/configuration.hpp"


/**
 * @class ParkingNode
 * @implements ParkingInterface
 * @brief Control the car during a parking maneuver
 * @details This class is the main class of the parking node. It handles the parking process
 * after a parking spot has been detected. This class DOES NOT detect the parking spots itself.
 */
class ParkingNode : public libpsaf::ParkingInterface
{
public:
  ParkingNode();

  /**
  * @brief Method in which the results get published
  * @details This method is called periodically by the main method of the node.
  */
  void update();

protected:
  /**
   * @brief Callback Method for the Parking spot
   * @param[in] p the detected parking spot
   */
  void processParkingSpot(libpsaf_msgs::msg::ParkingSpot::SharedPtr p) override;

  /**
   * @brief Callback Method for the ultrasonic sensors
   * @param[in] p the measured range of the ultrasonic sensor
   * @param[in] sensor the position of the sensor
   */
  void updateSensorValue(sensor_msgs::msg::Range::SharedPtr p, int sensor);

  /**
   * @brief Callback Method for the state of the state machine
   * @param[in] state the current state of the state machine
   */
  void updateState(std_msgs::msg::Int64::SharedPtr state) override;

private:
};

#endif  // PSAF_PARKING__PARKING_NODE_HPP_
