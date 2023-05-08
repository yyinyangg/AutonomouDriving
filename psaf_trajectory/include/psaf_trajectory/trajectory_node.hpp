/**
 * @file trajectory_node.hpp
 * @brief Definition of the trajectory node
 * @author PSAF
 * @date 2022-06-01
 */
#ifndef PSAF_TRAJECTORY__TRAJECTORY_NODE_HPP_
#define PSAF_TRAJECTORY__TRAJECTORY_NODE_HPP_

#include <vector>
#include "libpsaf/interface/trajectory_interface.hpp"
#include "psaf_configuration/configuration.hpp"
#include "opencv4/opencv2/opencv.hpp"
#include "opencv4/opencv2/core/cvdef.h"

/**
 * @class TrajectoryNode
 * @implements TrajectoryInterface
 * @brief Calculate the trajectory based on the detected lane markings and obstacles
 * @details This class is the main class of the trajectory node. It is responsible for
 * the calculation of the trajectory and the publishing of the trajectory.
 * The trajectory is being calculated from the detected lanemarkings under consideration of obstacles.
 */
class TrajectoryNode : public libpsaf::TrajectoryInterface
{
public:
  TrajectoryNode();

  /**
   * @brief Method in which the results get published
   * @details This method is called periodically by the main method of the node.
   */
  void update();

protected:
  /**
   * The callback Method for the state
   * @param[in] state the current state of the StateMachine
   */
  void updateState(std_msgs::msg::Int64::SharedPtr state) override;

  /**
   * The callback Method for the detected object
   * @param[in] p the detected object
   */
  void processObstacle(libpsaf_msgs::msg::Obstacle::SharedPtr p) override;

  /**
   * @brief The callback Method for the detected lane markings
   * @param[in] p the detected lane markings
   */
  void processLaneMarkings(libpsaf_msgs::msg::LaneMarkings::SharedPtr p) override;

private:
  bool noLeftLine = false;
  bool noDottedLine = false;
  bool noRightLine = false;
  bool driveInTheOutsideLane = true;
  double lookForward = 40 + 55;
  double width = 22;
  int counter = 0;
  geometry_msgs::msg::Point lastTargetPoint;  // = geometry_msgs::msg::Point(0, 100);

  void transformCoordinate(
    std::vector<geometry_msgs::msg::Point> & pointsFromLeftLine,
    std::vector<geometry_msgs::msg::Point> & pointsFromDottedLine,
    std::vector<geometry_msgs::msg::Point> & pointsFromRightLine,
    libpsaf_msgs::msg::LaneMarkings::SharedPtr p);

  void checkPoints(
    std::vector<geometry_msgs::msg::Point> & pointsFromLeftLine,
    std::vector<geometry_msgs::msg::Point> & pointsFromDottedLine,
    std::vector<geometry_msgs::msg::Point> & pointsFromRightLine);

  void interpolate(
    std::vector<geometry_msgs::msg::Point> & points, cv::Vec4f & coefficientOfLeftLine,
    float toleranceOfInterpolation);

  cv::Vec4f executeRANSAC(
    std::vector<geometry_msgs::msg::Point> totalPoints,
    float distanceTolerance);
  geometry_msgs::msg::Point calculateReferencePointFromRight(cv::Vec4f coefficient);
  geometry_msgs::msg::Point calculateReferencePointFromLeft(cv::Vec4f coefficient);
  void drawInterpolatedLine(
    cv::Vec4f coefficient1, cv::Vec4f coefficient2,
    geometry_msgs::msg::Point targetPoint);
  void drawInterpolatedLine(cv::Vec4f coefficient1, geometry_msgs::msg::Point targetPoint);
  void draw(
    cv::Vec4f coefficient, cv::Mat & img, geometry_msgs::msg::Point targetPoint,
    cv::Scalar scalar = cv::Scalar(255, 255, 255));
};

#endif  // PSAF_TRAJECTORY__TRAJECTORY_NODE_HPP_
