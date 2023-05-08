/**
 * @file object_detection_node.hpp
 * @brief Header file for the ObjectDetectionNode class.
 * @author PSAF
 * @date 2022-06-01
 */
#ifndef PSAF_OBJECT_DETECTION__OBJECT_DETECTION_NODE_HPP_
#define PSAF_OBJECT_DETECTION__OBJECT_DETECTION_NODE_HPP_

#include <string>
#include <vector>
#include "opencv4/opencv2/opencv.hpp"
#include "libpsaf/interface/object_detection_interface.hpp"
#include "psaf_configuration/configuration.hpp"

/**
 * @class ObjectDetectionNode
 * @implements ObjectDetectionInterface
 * @brief Detect objects on or next to the road.
 * @details This class is the node for the object detection. It is responsible for the detection
 * of obstacles in the path of the car.
 */
class ObjectDetectionNode : public libpsaf::ObjectDetectionInterface
{
public:
  ObjectDetectionNode();

  /**
  * @brief Method in which the results get published
  * @details This method is called periodically by the main method of the node.
  */
  void update();

protected:
  /**
   * @brief Callback Method for the image
   * @param[in] img the image
   * @param[in] sensor the position of the topic name in the image vector
   */
  void processImage(cv::Mat & img, int sensor) final;

  /**
   * @brief Callback Method for the current state of the state machine
   * @param[in] state the state of the state machine
   */
  void updateState(std_msgs::msg::Int64::SharedPtr state) override;

  /**
   * @brief Callback Method for the ultrasonic sensors
   * @param[in] p the ultrasonic sensor value
   * @param[in] sensor the position of the topic name in the topic vector
   */
  void updateSensorValue(sensor_msgs::msg::Range::SharedPtr p, int sensor);
};

#endif  // PSAF_OBJECT_DETECTION__OBJECT_DETECTION_NODE_HPP_
