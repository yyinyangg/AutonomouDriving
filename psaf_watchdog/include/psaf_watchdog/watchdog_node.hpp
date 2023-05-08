/**
 * @file watchdog_node.hpp
 * @brief The definition of the watchdog node
 * @author PSAF
 * @date 2022-06-01
 */
#ifndef PSAF_WATCHDOG__WATCHDOG_NODE_HPP_
#define PSAF_WATCHDOG__WATCHDOG_NODE_HPP_

#include <string>
#include "libpsaf/interface/watchdog_interface.hpp"
#include "psaf_configuration/configuration.hpp"

/**
 * @class WatchdogNode
 * @implements WatchdogInterface
 * @brief Monitor certain hardware components for faults
 * @details This class is the watchdog node. It is responsible for
 * monitoring various parameters of the system and issue error and warning
 * messages if the system is not behaving as expected.
 * This includes failures of the camera, US sensors, and motors
 * INPUT: camera image, us sensor data, speed and steering data. current state
 * OUTPUT: Error Message
 */
class WatchdogNode : public libpsaf::WatchdogInterface
{
public:
  WatchdogNode();

  /**
   * @brief Method in which the results get published
   * @details This method is called periodically by the main method of the node.
   */
  void update();

protected:
  /**
   * @brief Callback Method for the camera images
   * @param[in] image the image
   * @param[in] sensor the position of the topic in the topic vector
   */
  void processImage(cv::Mat & image, int sensor) final;

  /**
   * @brief Callback Method for the speed of the car
   * @param[in] speed the speed of the car in cm/s
   */
  void updateSpeed(std_msgs::msg::Int16::SharedPtr speed) override;

  /**
   * The callback Method for the steering angle
   * @param[in] steering the steering angle
   */
  void updateSteering(std_msgs::msg::Int16::SharedPtr steering) override;

  /**
   * @brief Callback Method for the ultrasonic sensors
   * @param[in] p the distance of the sensor in cm
   * @param[in] sensor the position of the topic in the topic vector
   */
  void updateSensorValue(sensor_msgs::msg::Range::SharedPtr p, int sensor) override;

  /**
   * Callback Method for the current state
   * @param[in] state the current state of the State Machine
   */
  void updateState(std_msgs::msg::Int64::SharedPtr state);
};

#endif  // PSAF_WATCHDOG__WATCHDOG_NODE_HPP_
