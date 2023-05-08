/**
 * @file manual_mode_node.hpp
 * @brief This file contains the definition of the ManualModeNode class.
 * @author PSAF
 * @date 2022-06-01
 */
#ifndef PSAF_MANUAL_MODE__MANUAL_MODE_NODE_HPP_
#define PSAF_MANUAL_MODE__MANUAL_MODE_NODE_HPP_

#include <string>
#include <vector>

#include "libpsaf/interface/manual_mode_interface.hpp"
#include "psaf_configuration/configuration.hpp"
#include "std_msgs/msg/u_int8.hpp"

/**
 * @class ManualModeNode
 * @implements ManualModeInterface
 * @brief Detects the entry in the manual mode and informs the state machine
 * @details This class is the node that handles the manual mode.
 * The class detects the entry and exit of the manual mode and informs the state machine.
 * It receives the speed and steering angle during manual operation.
 */
class ManualModeNode : public libpsaf::ManualModeInterface
{
public:
  ManualModeNode();

  /**
   * The method is called periodically by the main loop.
   * Use it to publish the results by calling the publisher(s).
   */
  void update();

protected:
  /**
   * @brief Callback Method for the state of the state machine
   * @param[in] state the state of the state machine
   */
  void updateState(std_msgs::msg::Int64::SharedPtr state) override;

  /**
   * @brief Callback Method for the manual mode
   * @param[in] msignal: 0 if not in manual mode, 1 if in manual mode
   */
  void processManualSignals(std_msgs::msg::UInt8::SharedPtr msignal) override;

  /**
   * @brief Callback Method for the steering angle
   * @param[in] p the steering angle
   */
  void updateSteering(std_msgs::msg::Int16::SharedPtr p) override;

  /**
   * @brief Callback Method for the speed
   * @param[in] p the speed in cm/s
   */
  void updateSpeed(std_msgs::msg::Int16::SharedPtr p) override;
};

#endif  // PSAF_MANUAL_MODE__MANUAL_MODE_NODE_HPP_
