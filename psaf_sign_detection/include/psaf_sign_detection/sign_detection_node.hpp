/**
 * @file sign_detection_node.hpp
 * @brief The definition of the SignDetectionNode class
 * @author PSAF
 * @date 2022-06-01
 */
#ifndef PSAF_SIGN_DETECTION__SIGN_DETECTION_NODE_HPP_
#define PSAF_SIGN_DETECTION__SIGN_DETECTION_NODE_HPP_

#include <string>
#include <vector>

#include "opencv4/opencv2/opencv.hpp"
#include "libpsaf/interface/sign_detection_interface.hpp"
#include "psaf_configuration/configuration.hpp"

/**
 * @class SignDetectionNode
 * @implements SignDetectionInterface
 * @brief Detect the signs located next to the road.
 * @details This class is the node of the sign detection. It is responsible for
 *          the detection of signs and the publishing of the detected signs.
 */
class SignDetectionNode : public libpsaf::SignDetectionInterface
{
public:
  SignDetectionNode();

  /**
   * @brief Method in which the results get published
   * @details This method is called periodically by the main method of the node.
   */
  void update();

protected:
  /**
   * @brief Callback Method for the image topic
   * @param[in] img the image
   * @param[in] sensor the position of the topic in the topic vector
   */
  void processImage(cv::Mat & img, int sensor) final;

  /**
   * @brief Callback Method for the state
   * @param[in] state the current state of the state machine
   */
  void updateState(std_msgs::msg::Int64::SharedPtr state) override;

private:
};

#endif  // PSAF_SIGN_DETECTION__SIGN_DETECTION_NODE_HPP_
