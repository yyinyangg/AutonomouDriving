/**
 * @file trajectory_node.cpp
 * @brief The implementation of the trajectory node
 * @author PSAF
 * @date 2022-06-01
 */
#include "psaf_trajectory/trajectory_node.hpp"

TrajectoryNode::TrajectoryNode()
: TrajectoryInterface(
    TRAJECTORY_NODE_NAME,
    OBJECT_TOPIC,
    STATE_TOPIC,
    LANE_MARKINGS_TOPIC,
    TRAJECTORY_TOPIC,
    STATUS_INFO_TOPIC,
    rclcpp::QoS(rclcpp::KeepLast {10})
)
{
  lastTargetPoint.x = 0.5;
  lastTargetPoint.y = 100;
}

void TrajectoryNode::updateState(std_msgs::msg::Int64::SharedPtr state)
{
  (void) state;
}

void TrajectoryNode::processObstacle(libpsaf_msgs::msg::Obstacle::SharedPtr p)
{
  (void) p;
}

void TrajectoryNode::processLaneMarkings(libpsaf_msgs::msg::LaneMarkings::SharedPtr p)
{
  cv::Vec4f coefficientOfLeftLine;
  cv::Vec4f coefficientOfDottedLine;
  cv::Vec4f coefficientOfRightLine;
  std::vector<geometry_msgs::msg::Point> pointsFromLeftLine;
  std::vector<geometry_msgs::msg::Point> pointsFromDottedLine;
  std::vector<geometry_msgs::msg::Point> pointsFromRightLine;

  transformCoordinate(pointsFromLeftLine, pointsFromDottedLine, pointsFromRightLine, p);

  checkPoints(pointsFromLeftLine, pointsFromDottedLine, pointsFromRightLine);
  float toleranceOfInterpolation = 5;
  std::vector<geometry_msgs::msg::Point> points;

  if (driveInTheOutsideLane) {
    if (!noDottedLine) {
      interpolate(pointsFromDottedLine, coefficientOfDottedLine, toleranceOfInterpolation);
    }
    if (!noRightLine) {
      interpolate(pointsFromRightLine, coefficientOfRightLine, toleranceOfInterpolation);
    }

    if (noDottedLine && noRightLine) {
      points.push_back(lastTargetPoint);
    } else if (noDottedLine && !noRightLine) {
      geometry_msgs::msg::Point targetPoint = calculateReferencePointFromRight(
        coefficientOfRightLine);
      drawInterpolatedLine(coefficientOfRightLine, targetPoint);
      points.push_back(targetPoint);
      lastTargetPoint = targetPoint;
    } else if (!noDottedLine && noRightLine) {
      geometry_msgs::msg::Point targetPoint = calculateReferencePointFromLeft(
        coefficientOfDottedLine);
      drawInterpolatedLine(coefficientOfDottedLine, targetPoint);
      points.push_back(targetPoint);
      lastTargetPoint = targetPoint;
    } else {
      geometry_msgs::msg::Point ReferencePointDotted = calculateReferencePointFromLeft(
        coefficientOfDottedLine);
      geometry_msgs::msg::Point ReferencePointRight = calculateReferencePointFromRight(
        coefficientOfRightLine);
      geometry_msgs::msg::Point targetPoint;

      targetPoint.x = (ReferencePointRight.x + ReferencePointDotted.x) / 2;
      targetPoint.y = (ReferencePointRight.y + ReferencePointDotted.y) / 2;
      drawInterpolatedLine(coefficientOfDottedLine, coefficientOfRightLine, targetPoint);
      points.push_back(targetPoint);
      lastTargetPoint = targetPoint;
    }
  }
  publishTrajectory(points);
}


void TrajectoryNode::update()
{
}

geometry_msgs::msg::Point TrajectoryNode::calculateReferencePointFromRight(cv::Vec4f coefficient)
{
  float theta = atan(
    3 * coefficient[0] * lookForward * lookForward +
    2 * coefficient[1] * lookForward + coefficient[2]);
  if (theta < 0) {
    theta += CV_PI;
  }
  theta = theta - CV_PI / 2;
  if (theta < 0) {
    theta += CV_PI;
  }
  float deltaX = width * sin(theta);
  float deltaY = width * cos(theta);

  geometry_msgs::msg::Point ReferencePoint;
  ReferencePoint.x = coefficient[0] * lookForward * lookForward * lookForward +
    coefficient[1] * lookForward * lookForward +
    coefficient[2] * lookForward + coefficient[3] -
    deltaX;

  ReferencePoint.y = lookForward - deltaY;

  return ReferencePoint;
}
geometry_msgs::msg::Point TrajectoryNode::calculateReferencePointFromLeft(cv::Vec4f coefficient)
{
  float theta = atan(
    3 * coefficient[0] * lookForward * lookForward +
    2 * coefficient[1] * lookForward + coefficient[2]);
  if (theta < 0) {
    theta += CV_PI;
  }
  theta = theta - CV_PI / 2;
  if (theta < 0) {
    theta += CV_PI;
  }
  float deltaX = width * sin(theta);
  float deltaY = width * cos(theta);
  geometry_msgs::msg::Point ReferencePoint;
  ReferencePoint.x = coefficient[0] * lookForward * lookForward * lookForward +
    coefficient[1] * lookForward * lookForward +
    coefficient[2] * lookForward + coefficient[3] +
    deltaX;

  ReferencePoint.y = lookForward + deltaY;

  return ReferencePoint;
}
cv::Vec4f TrajectoryNode::executeRANSAC(
  std::vector<geometry_msgs::msg::Point> totalPoints,
  float distanceTolerance)
{
  std::unordered_set<int> bestInliers;
  int maxIteration = 20;
  float a_best = 0.0;
  float b_best = 0.0;
  float c_best = 0.0;
  float d_best = 0.0;
  for (int i = 0; i < maxIteration; i++) {
    std::unordered_set<int> inliers;

    // for line we need 2 points, select points randomly
    while (inliers.size() < 4) {
      inliers.insert(rand() % (totalPoints.size()));
    }

    float x1, y1, x2, y2, x3, y3, x4, y4;

    auto iterator = inliers.begin();

    x1 = totalPoints[*iterator].x;
    y1 = totalPoints[*iterator].y;

    iterator++;

    x2 = totalPoints[*iterator].x;
    y2 = totalPoints[*iterator].y;

    iterator++;

    x3 = totalPoints[*iterator].x;
    y3 = totalPoints[*iterator].y;

    iterator++;

    x4 = totalPoints[*iterator].x;
    y4 = totalPoints[*iterator].y;

    if (y1 == y2 || y2 == y3 || y1 == y3 || y3 == y4 || y2 == y4 || y1 == y4) {
      continue;
    }

    float alpha1 = ((x1 - x2) / (y1 - y2) - (x2 - x3) / (y2 - y3)) / (y1 - y3);
    float alpha2 = ((x2 - x3) / (y2 - y3) - (x3 - x4) / (y3 - y4)) / (y2 - y4);
    float betha1 = y1 + y2 + y3;
    float betha2 = y2 + y3 + y4;
    float a = (alpha1 - alpha2) / (betha1 - betha2);
    float b = alpha1 - a * betha1;
    float c = (x1 - x2) / (y1 - y2) - a * (y1 * y1 + y1 * y2 + y2 * y2) - b * (y1 + y2);
    float d = x1 - a * y1 * y1 * y1 - b * y1 * y1 - c * y1;
    for (std::size_t index = 0; index < totalPoints.size(); index++) {
      if (inliers.count(index) > 0) {
        continue;
      }
      geometry_msgs::msg::Point point = totalPoints[index];
      float x_test = point.x;
      float y_test = point.y;
      float distance = fabs(
        x_test - a * y_test * y_test * y_test - b * y_test * y_test - c * y_test - d);

      if (distance <= distanceTolerance) {
        inliers.insert(index);
      }
    }

    if (inliers.size() > bestInliers.size()) {
      bestInliers = inliers;
      a_best = a;
      b_best = b;
      c_best = c;
      d_best = d;
    }
  }
  cv::Vec4f coefficientOfInterpolation(a_best, b_best, c_best, d_best);

  return coefficientOfInterpolation;
}

void TrajectoryNode::transformCoordinate(
  std::vector<geometry_msgs::msg::Point> & pointsFromLeftLine,
  std::vector<geometry_msgs::msg::Point> & pointsFromDottedLine,
  std::vector<geometry_msgs::msg::Point> & pointsFromRightLine,
  libpsaf_msgs::msg::LaneMarkings::SharedPtr p)
{
  (void) pointsFromLeftLine;
  counter = p.get()->left_lane.at(0).x;
  for (size_t i = 0; i < p.get()->center_lane.size(); i++) {
    p.get()->center_lane.at(i).set__x((p.get()->center_lane.at(i).x - 260) * 0.15);
    p.get()->center_lane.at(i).set__y(((1280 - p.get()->center_lane.at(i).y) * 0.15) + 40);
    if (p.get()->center_lane.at(i).x > (270 - 260) * 0.15 && p.get()->center_lane.at(i).y >
      (550 * 0.15 + 40))
    {
      continue;
    }
    pointsFromDottedLine.push_back(p.get()->center_lane.at(i));
  }

  for (size_t i = 0; i < p.get()->right_lane.size(); i++) {
    p.get()->right_lane.at(i).set__x((p.get()->right_lane.at(i).x - 260) * 0.15);
    p.get()->right_lane.at(i).set__y(((1280 - p.get()->right_lane.at(i).y) * 0.15) + 40);
    if (p.get()->right_lane.at(i).x < (250 - 260) * 0.15 && p.get()->right_lane.at(i).y >
      (620 * 0.15 + 40))
    {
      continue;
    }
    pointsFromRightLine.push_back(p.get()->right_lane.at(i));
  }
}

void TrajectoryNode::interpolate(
  std::vector<geometry_msgs::msg::Point> & points, cv::Vec4f & coefficientOfLine,
  float toleranceOfInterpolation)
{
  coefficientOfLine = (executeRANSAC(points, toleranceOfInterpolation));
}

void TrajectoryNode::checkPoints(
  std::vector<geometry_msgs::msg::Point> & pointsFromLeftLine,
  std::vector<geometry_msgs::msg::Point> & pointsFromDottedLine,
  std::vector<geometry_msgs::msg::Point> & pointsFromRightLine)
{
  std::size_t thresholdOfDottedLine = 10;  // minimal nr of points
  std::size_t thresholdOfLine = 15;

  if (!pointsFromLeftLine.empty() && pointsFromLeftLine.size() > thresholdOfLine) {
    noLeftLine = false;
  } else {
    noLeftLine = true;
    RCLCPP_INFO(this->get_logger(), "Left Line not found!");
  }
  if (!pointsFromDottedLine.empty() && pointsFromDottedLine.size() > thresholdOfDottedLine) {
    noDottedLine = false;
  } else {
    noDottedLine = true;
    RCLCPP_INFO(this->get_logger(), "Dotted Line not found!");
  }

  if (!pointsFromRightLine.empty() && pointsFromRightLine.size() > thresholdOfLine) {
    noRightLine = false;
  } else {
    noRightLine = true;
    RCLCPP_INFO(this->get_logger(), "Right Line not found!");
  }
}

void TrajectoryNode::drawInterpolatedLine(
  cv::Vec4f coefficient1, cv::Vec4f coefficient2,
  geometry_msgs::msg::Point targetPoint)
{
  std::ostringstream src;
  src << "/home/psaf/Documents/Pictures/box/transformedImage_" << counter << ".jpg";
  cv::Mat img = cv::imread(src.str());
  draw(coefficient1, img, targetPoint, cv::Scalar(255, 0, 0));
  draw(coefficient2, img, targetPoint, cv::Scalar(0, 255, 0));

  std::ostringstream oss1;
  oss1 << "/home/psaf/Documents/Pictures/box/InterpolatedLine" << counter << ".jpg";
  cv::imwrite(oss1.str(), img);
}

void TrajectoryNode::drawInterpolatedLine(
  cv::Vec4f coefficient1,
  geometry_msgs::msg::Point targetPoint)
{
  std::ostringstream src;
  src << "/home/psaf/Documents/Pictures/box/transformedImage_" << counter << ".jpg";
  cv::Mat img = cv::imread(src.str());
  draw(coefficient1, img, targetPoint, cv::Scalar(255, 0, 0));

  std::ostringstream oss1;
  oss1 << "/home/psaf/Documents/Pictures/box/InterpolatedLine" << counter << ".jpg";
  cv::imwrite(oss1.str(), img);
}

void TrajectoryNode::draw(
  cv::Vec4f coefficient, cv::Mat & img, geometry_msgs::msg::Point targetPoint,
  cv::Scalar scalar)
{
  double a = coefficient[0], b = coefficient[1], c = coefficient[2], d = coefficient[3];

  cv::Point2d prev_pt(d / 0.15 + 260, 1280);
  for (double t = 0; t <= 1; t += 0.01) {
    double y = t * 250;
    double x = a * pow(y, 3) + b * pow(y, 2) + c * y + d;
    x = x / 0.15 + 260;
    y = 1280 - (y - 40) / 0.15;
    if (x <= 0) {x = 0;}
    if (x >= 480) {x = 480;}
    if (y <= 0) {y = 0;}
    if (y >= 1280) {y = 1280;}

    cv::Point2d pt(x, y);
    cv::line(img, prev_pt, pt, scalar, 2);
    prev_pt = pt;
  }
  double y = targetPoint.y;
  double x = targetPoint.x;
  x = x / 0.15 + 260;
  y = 1280 - (y - 40) / 0.15;
  if (x < 0) {x = 0;}
  if (x > 480) {x = 480;}
  if (y < 0) {y = 0;}
  if (y > 1280) {y = 1280;}
  cv::Point2d pt(x, y);
  cv::circle(img, pt, 4, cv::Scalar(0, 0, 255), -1);
}
