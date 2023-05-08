/**
 * @file lane_detection_node.cpp
 * @brief implementation of the lane_detection
 * @author PSAF
 * @date 2022-06-01
 */
#include "psaf_lane_detection/lane_detection_node.hpp"
#include <vector>
#include <string>
#include <iostream>
#include <stdexcept>
using namespace std;
using namespace cv;

LaneDetectionNode::LaneDetectionNode()
: LaneDetectionInterface(
    LANE_DETECTION_NODE_NAME,
    NBR_OF_CAMS_RGB,
    CAM_TOPIC_RGB,
    STATE_TOPIC,
    LANE_MARKINGS_TOPIC,
    STOP_LINE_TOPIC,
    STATUS_INFO_TOPIC,
    rclcpp::QoS(rclcpp::KeepLast {10})
)
{
  // Dynamic reconfigure of the used algorithm
  this->declare_parameter("use_secondary_algorithm", false);
}

int counter = 0;

void LaneDetectionNode::processImage(Mat & img, int sensor)
{
  // Sensor can be set to void, since the lane detection only receives the color image
  // If you want to use it, the sensor value should always be 0
  (void) sensor;

  // Get image dimensions
  int width = img.cols;
  int height = img.rows;

  //Resize image if it´s not 640x480
  if (width != 640 || height != 480) {
    resize(img, img, Size(640, 480));
  }

  // Get image dimensions


  Mat gray, transformed;
  grayscaleImage(img, gray);

  Mat homography = Mat::zeros(3, 3, CV_64F);
  // Set homography according to the used car
  if (PSAF1) {
    // homography for our camera calibration
    homography = (Mat(3, 3, CV_64F, homography_data_psaf1)).clone();

  } else {
    homography = (Mat(3, 3, CV_64F, homography_data_psaf2)).clone();
  }


  ostringstream oss2;


  GaussianBlur(gray, gray, Size(3, 3), 0);
  Canny(gray, gray, 100, 200, 3, false);
  Mat cropped_image;
  transformImage(gray, homography, cropped_image);

  //RCLCPP_ERROR(this->get_logger(), "Image saved");


  Mat cropped_image2 = cropped_image.clone();
  cvtColor(cropped_image2, cropped_image2, COLOR_GRAY2BGR);

  vector<Point> pointsOnMiddleLine;
  vector<Point> pointsOnRightLine;
  vector<Point> pointsOnLeftLine;
  vector<vector<tuple<Vec2i, double>>> lanes(2);

  findLane(cropped_image, cropped_image2, lanes);

  oss2 << "/home/psaf/Documents/Pictures/box/transformedImage_" << counter << ".jpg";
  imwrite(oss2.str(), cropped_image2);

  vector<tuple<Vec2i, double>> rightLane = lanes[0];
  for (tuple<Vec2i, double> lineSegment:rightLane) {
    //RCLCPP_ERROR(this->get_logger(), "forEach right");
    Point point = get<0>(lineSegment);

    pointsOnRightLine.push_back(point);
    // circle(cropped_image2, pointOnMiddle, 4, Scalar(0, 0, 255), -1);
  }

  vector<tuple<Vec2i, double>> leftLane = lanes[1];
  for (tuple<Vec2i, double> lineSegment:leftLane) {
    Point point = get<0>(lineSegment);
    pointsOnMiddleLine.push_back(point);
    // circle(cropped_image2, pointOnMiddle, 4, Scalar(0, 0, 255), -1);
  }
  Point point(counter, counter);
  pointsOnLeftLine.push_back(point);
  last_lane_markings_positions_.push_back(pointsOnLeftLine);
  last_lane_markings_positions_.push_back(pointsOnMiddleLine);
  last_lane_markings_positions_.push_back(pointsOnRightLine);
  // RCLCPP_INFO(this->get_logger(), "pointsOnMiddleLine.size() %ld",pointsOnMiddleLine.size());
  // RCLCPP_INFO(this->get_logger(), "pointsOnRightLine.size() %ld",pointsOnRightLine.size());

  if (!last_lane_markings_positions_.empty()) {
    publishLaneMarkings(
      last_lane_markings_positions_.at(0), last_lane_markings_positions_.at(1),
      last_lane_markings_positions_.at(2), has_blocked_area_, no_overtaking_, side_);
    last_lane_markings_positions_.clear();
  }
  counter++;
}

void LaneDetectionNode::update()
{
  // Publish Lane Markings information
  /*if (!last_lane_markings_positions_.empty()) {
    publishLaneMarkings(
      last_lane_markings_positions_.at(0), last_lane_markings_positions_.at(1),
      last_lane_markings_positions_.at(2), has_blocked_area_, no_overtaking_, side_);
    last_lane_markings_positions_.clear();
  }*/

  // Build the status info message and publish it, but not if state = -1
  // @TODO: Add additional states in which the lane_detection should not publish status info
  if (current_state_ != -1) {
    libpsaf_msgs::msg::StatusInfo status_msg;
    status_msg.header.stamp = rclcpp::Time(0);
    status_msg.header.frame_id = "";
    status_msg.type = status_info_;
    publishStatus(status_msg);
  }

  // Publish Stop Line information if the state is appropriate
  if (current_state_ == 10 || current_state_ == 13 || current_state_ == 14) {
    int x = last_stop_line_position_.x;
    int y = last_stop_line_position_.y;
    publishStopLine(stop_line_found_, stop_line_type_, x, y);
  }
}

void LaneDetectionNode::updateState(std_msgs::msg::Int64::SharedPtr state)
{
  current_state_ = state->data;
}

void LaneDetectionNode::grayscaleImage(Mat & img, Mat & result)
{
  if (img.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Image is empty");
    return;
  }
  // Check if image has only one channel. If true, do not convert to grayscale
  if (img.channels() == 1) {
    result = img.clone();
  } else {
    cvtColor(img, result, COLOR_BGR2GRAY);
  }
}

void LaneDetectionNode::binarizeImage(
  Mat & img, Mat & result, int threshold_low,
  int threshold_high)
{
  if (img.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Image is empty");
    return;
  }

  if (threshold_low >= threshold_high) {
    RCLCPP_ERROR(this->get_logger(), "Threshold low is higher than or equal threshold high");
    return;
  }
  threshold(img, result, threshold_low, threshold_high, THRESH_BINARY);
}

void LaneDetectionNode::transformImage(Mat & img, Mat & homography, Mat & result)
{
  if (DEBUG) {
    cout << "PSAF1 homography" << endl;
    cout << homography << endl;
  }

  if (img.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Image is empty");
    return;
  }
  // check if homography is valid (not empty and 3x3)
  if (homography.empty() || homography.rows != 3 || homography.cols != 3) {
    RCLCPP_ERROR(this->get_logger(), "Homography is empty or not 3x3");
    return;
  }
  Size size(480, 1280);
  warpPerspective(img, result, homography, size);
}

void LaneDetectionNode::resizeImage(Mat & img, Mat & result)
{
  if (img.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Input Image is empty");
    return;
  }
  if (img.cols == 640 && img.rows == 480) {
    result = img.clone();
  } else {
    resize(img, result, Size(640, 480), INTER_LINEAR);
  }
}

void LaneDetectionNode::extractLaneMarkings(Mat & img)
{
  vector<vector<Point>> lane_markings;
  if (img.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Input Image is empty");
    return;
  }
  // The following code is Dummy code and will not work with real images.
  // It is only used to make the current testcases pass.
  // Please remove the code and replace it with your own lane marking detection code.
  // @TODO replace with your own lane marking detection code

  // Get image width and height
  int width = img.cols;
  vector<int> y_values{0, 26, 53, 80, 106, 133, 160, 186, 213, 240, 253, 266, 280, 293, 306,
    320, 333, 346, 360, 373, 386, 400, 413, 426, 440, 453, 466};
  // Create a vector for the left, center and right lane points
  vector<Point> left_lane_points;
  vector<Point> center_lane_points;
  vector<Point> right_lane_points;
  for (size_t y = 0; y < y_values.size(); y++) {
    // Get the row from the image depending on the y value
    Mat row = img.row(y_values[y]);
    // Find the non zero pixels in the row
    Mat non_zero_pixels;
    findNonZero(row, non_zero_pixels);
    int sum = 0;
    int count = 0;
    // Iterate over the non endzero pixels and average, if the next pixel is a direct neighbor
    for (int x = 0; x < non_zero_pixels.rows - 1; x++) {
      if (non_zero_pixels.at<Point>(x + 1, 0).x - non_zero_pixels.at<Point>(x, 0).x == 1) {
        sum += non_zero_pixels.at<Point>(x, 0).x;
        count++;
      } else {
        if (count == 0) {
          sum = non_zero_pixels.at<Point>(x, 0).x;
          count = 1;
        }
        int average = sum / count;
        Mat slidingWindowHoughTransform(Mat & img,
          vector<Vec4i> & hlines, int heightProp, int widthProp);
        if (average < static_cast<int>(width * .18)) {
          left_lane_points.push_back(Point(average, y_values[y]));
        } else if (average > static_cast<int>(width * .18) && average < width / 2) {
          center_lane_points.push_back(Point(average, y_values[y]));
        } else {
          if (count < 80) {
            right_lane_points.push_back(Point(average, y_values[y]));
          }
        }
        sum = 0;
        count = 0;
      }
      if (x == non_zero_pixels.rows - 2) {
        if (count == 0) {
          sum = non_zero_pixels.at<Point>(x, 0).x;
          count = 1;
        }
        int average = sum / count;
        if (average < static_cast<int>(width * .18)) {
          left_lane_points.push_back(Point(average, y_values[y]));
        } else if (average > static_cast<int>(width * .18) && average < width / 2) {
          center_lane_points.push_back(Point(average, y_values[y]));
        } else {
          if (count < 80) {
            right_lane_points.push_back(Point(average, y_values[y]));
          }
        }
        sum = 0;
        count = 0;
      }
    }
  }
  if (DEBUG) {
    imshow("Lane Markings", img);
    waitKey(32);
    cout << "Found " << left_lane_points.size() << " left lane points" << endl;
    cout << "Found " << center_lane_points.size() << " center lane points" << endl;
    cout << "Found " << right_lane_points.size() << " right lane points" << endl;
  }
  lane_markings.push_back(left_lane_points);
  lane_markings.push_back(center_lane_points);
  lane_markings.push_back(right_lane_points);
  side_ = 0;
  no_overtaking_ = true;
  /** @todo end replace with your own lane marking detection code */

  last_lane_markings_positions_ = lane_markings;
}

void LaneDetectionNode::extractLaneMarkingsSecondary(Mat & img)
{
  // @todo implement a secondary algorithm if you like
  // Log that we are in the secondary lane marking extraction
  RCLCPP_INFO(this->get_logger(), "Using secondary lane markings algorithm");
  (void) img;
}

void LaneDetectionNode::extractStopLine(Mat & img)
{
  if (img.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Input Image is empty");
    stop_line_found_ = false;
    last_stop_line_position_ = Point(-1, -1);
    stop_line_type_ = 0;
    return;
  }
  // @TODO replace with your own stop line detection code
  stop_line_found_ = true;
  last_stop_line_position_ = Point(309, 76);
  stop_line_type_ = 0;
}

int LaneDetectionNode::getCurrentState()
{
  /** INFO ONLY USED FOR DEBUGGING AND TESTS AS OF NOW */
  return current_state_;
}

Mat LaneDetectionNode::getLastImage()
{
  /** INFO ONLY USED FOR DEBUGGING AND TESTS AS OF NOW */
  return last_image_;
}

bool LaneDetectionNode::findLeftLane(
  Mat & img, Mat & img2,
  vector<tuple<Vec2i, double>> & leftLane,
  int width, int height, int search_step)
{
  int notfound = 5;
  Vec2i startPoint = Vec2i(70, 1279 - search_step * height);


  bool startingPointFound = findLineInWindow(img, img2, leftLane, width * 0.85, height, startPoint);

  if (startingPointFound) {
    //find rest of left lane
    findRestOfLane(img, img2, leftLane, width, height, notfound);
    return true;
  }

  return false;
}

bool LaneDetectionNode::findRightLane(
  Mat & img, Mat & img2,
  vector<tuple<Vec2i, double>> & rightLane,
  int width, int height, int search_step)
{
  int notfound = 4;
  Vec2i startPoint = Vec2i(279, 1279 - search_step * height);


  bool startingPointFound = findLineInWindow(img, img2, rightLane, width, height, startPoint);

  if (startingPointFound) {
    //find rest of left lane
    findRestOfLane(img, img2, rightLane, width, height, notfound);
    return true;
  }

  return false;
}

void LaneDetectionNode::slidingWindowHoughTransform(
  Mat & img, vector<Vec4i> & hlines,
  int heightProp, int widthProp)
{
  int orig_width = img.cols;
  int orig_height = img.rows;
  int width = orig_width / widthProp;
  int height = orig_height / heightProp;

  vector<Vec4i> hlinesSW;
  int lineCounter = 1;

  for (int x = 0; x < widthProp; x++) {

    int y_start = x * width;
    int y_end = y_start + width;
    (void)y_end; // warning: unused variable ‘y_end’ [-Wunused-variable]
    for (int y = 0; y < heightProp; y++) {
      // could be even more efficient with just one point width and height
      // rectangulars can be shifted easier (look documentation) - could be even more efficient

      int x_start = orig_height - (y + 1) * height;
      int x_end = orig_height - y * height;
      (void)x_end; //warning: unused variable ‘x_end’ [-Wunused-variable]

      // calculate region of interest
      Mat windowContent(img(Rect(y_start, x_start, width, height)));

      vector<Vec4i> hlinesSW;
      HoughLinesP(windowContent, hlinesSW, 2, CV_PI / 180, 40, 30, 50);

      for (size_t i = 0; i < hlinesSW.size(); i++) {

        hlinesSW[i][0] += y_start;
        hlinesSW[i][1] += x_start;
        hlinesSW[i][2] += y_start;
        hlinesSW[i][3] += x_start;
      }
      hlines.insert(hlines.end(), hlinesSW.begin(), hlinesSW.end());
      lineCounter++;
    }
  }
  int numLines = static_cast<int>(hlines.size());
}

bool LaneDetectionNode::findLineInWindow(
  Mat & img, Mat & img2,
  vector<tuple<Vec2i, double>> & lane,
  int width, int height, Vec2i startPoint,
  double min_theta, double max_theta)
{

  Vec4i averageLine;
  averageLine[0] = 0;
  averageLine[1] = 0;
  averageLine[2] = 0;
  averageLine[3] = 0;
  tuple<Vec2i, double> laneSegment;

  int x_start = startPoint[0];
  int y_start = startPoint[1];

  Rect window = Rect(x_start, y_start, width, height);
  //Mat windowContent(houghInput(window));
  Mat windowContent(img(window));

  vector<Vec4i> hlinesSW;

  HoughLinesP(windowContent, hlinesSW, 2, CV_PI / 180, 20, 30, 2);
  size_t numberOfLines = hlinesSW.size();
  int numLinesInt = static_cast<int>(numberOfLines);
  if (numLinesInt <= 0) {
    rectangle(img2, window, Scalar(0, 255, 0));
    return false;
  } else {
    rectangle(img2, window, Scalar(255, 0, 0));

  }
  Point pt1, pt2;
  int effectiveNumberOfLines = 0;
  for (size_t i = 0; i < numberOfLines; i++) {
    pt1.x = hlinesSW[i][0];
    pt1.y = hlinesSW[i][1];
    pt2.x = hlinesSW[i][2];
    pt2.y = hlinesSW[i][3];
    double theta = calculateAngles(pt1, pt2);
    if (theta >= min_theta and theta <= max_theta) {
      averageLine[0] += hlinesSW[i][0];
      averageLine[1] += hlinesSW[i][1];
      averageLine[2] += hlinesSW[i][2];
      averageLine[3] += hlinesSW[i][3];
      effectiveNumberOfLines++;
    }

  }
  if (effectiveNumberOfLines == 0) {
    return false;
  }

  pt1.x = averageLine[0] / effectiveNumberOfLines + x_start;
  pt1.y = averageLine[1] / effectiveNumberOfLines + y_start;
  pt2.x = averageLine[2] / effectiveNumberOfLines + x_start;
  pt2.y = averageLine[3] / effectiveNumberOfLines + y_start;
  line(img2, pt1, pt2, Scalar(255, 0, 255), 3, 8);
  Point pta;
  pta.x = (pt1.x + pt2.x) / 2;
  pta.y = (pt1.y + pt2.y) / 2;
  get<0>(laneSegment) = Vec2i(static_cast<int>(pt1.x), static_cast<int>(pt1.y));
  double finalAngle = calculateAngles(pt1, pt2);
  get<1>(laneSegment) = finalAngle;
  lane.insert(lane.end(), laneSegment);
  get<0>(laneSegment) = Vec2i(static_cast<int>(pt2.x), static_cast<int>(pt2.y));
  get<1>(laneSegment) = finalAngle;
  lane.insert(lane.end(), laneSegment);
  get<0>(laneSegment) = Vec2i(static_cast<int>(pta.x), static_cast<int>(pta.y));
  get<1>(laneSegment) = finalAngle;
  lane.insert(lane.end(), laneSegment);
  return true;
}

void LaneDetectionNode::findLane(
  Mat & img, Mat & img2, vector<vector<tuple<Vec2i, double>>> & lanes)
{
  int width = 200;
  int height = 65;

  vector<tuple<Vec2i, double>> laneRight;
  vector<tuple<Vec2i, double>> laneLeft;
  int search_step = 1;
  int iteration = 1;
  bool right_detection = false;
  bool left_detection = false;
  while (iteration <= 3) {
    if (findRightLane(img, img2, laneRight, width, height, search_step) and !right_detection) {
      if (laneRight.size() >= 6) {
        lanes[0] = laneRight;
        right_detection = true;
        iteration += 10;
        //search_step = 10;
      }
    }
    if (findLeftLane(img, img2, laneLeft, width, height, search_step) and !left_detection) {
      if (laneLeft.size() >= 4) {
        lanes[1] = laneLeft;
        left_detection = true;
        iteration += 10;
//search_step = 10;
      }

    }
    iteration++;
    search_step++;
  }
}

void LaneDetectionNode::findRestOfLane(
  Mat & img, Mat & img2,
  vector<tuple<Vec2i, double>> & startOfLane,
  int width, int height, int num)
{
  bool searchForLane = true;
  int noLaneFoundCounter = 0;
  double min_theta;
  double max_theta;
  double splitAngle = acos(width / sqrt((pow((width), 2) + pow((height), 2))));
  int iteration = 1;
  int max_iteration = 20;
  while (searchForLane and iteration < max_iteration) {
    //RCLCPP_ERROR(this->get_logger(), "noLaneFoundCounter at beginning %d",noLaneFoundCounter);
    tuple<Vec2i, double> lastSegment = startOfLane.back();

    min_theta = get<1>(lastSegment) - CV_PI / 2 *
      (0.1 + 0.1 * noLaneFoundCounter);

    if (min_theta < 0) {
      min_theta = 0;
    }
    max_theta = get<1>(lastSegment) + CV_PI / 2 *
      (0.1 + 0.1 * noLaneFoundCounter);

    if (max_theta > CV_PI) {
      max_theta = CV_PI;
    }
    Vec2i nextStartPoint = getNextStartPoint(
      lastSegment, noLaneFoundCounter, splitAngle, width,
      height);
    if (pointOutOfImage(nextStartPoint, width, height)) {
      searchForLane = false;
      break;
    }
    bool nextSegmentFound = findLineInWindow(
      img, img2, startOfLane, width, height, nextStartPoint,
      min_theta, max_theta);
    if (!nextSegmentFound) {
      noLaneFoundCounter++;
    } else {
      noLaneFoundCounter = 0;
    }
    if (noLaneFoundCounter >= num) {
      searchForLane = false;
      break;
    }
    iteration++;
  }
}

Vec2i LaneDetectionNode::getNextStartPoint(
  tuple<Vec2i, double> lastSegment,
  int noLaneFoundCounter, double splitAngle, int width,
  int height)
{
  double scale = 0.5;
  int x_start = get<0>(lastSegment)[0] - width / 2;
  int y_start = get<0>(lastSegment)[1] - height / 2;
  double laneAngle = get<1>(lastSegment);
  //RCLCPP_INFO(this->get_logger(), "laneAngle :%f", laneAngle);
  //RCLCPP_INFO(this->get_logger(), "splitAngle :%f", splitAngle);
  //cout << "Compute next Start Point from given langeAngle =" << laneAngle << "\n";
  if (laneAngle == CV_PI) {
    return Vec2i(x_start - (1 + noLaneFoundCounter) * width, y_start);
  }
  if (laneAngle == 0) {
    return Vec2i(x_start + (1 + noLaneFoundCounter) * width, y_start);
  }
  if (laneAngle < CV_PI) {
    if (laneAngle == splitAngle) {
      return Vec2i(
        x_start + (1 + noLaneFoundCounter) * width,
        y_start - (1 + noLaneFoundCounter) * height);
    }
    if (laneAngle < splitAngle) {
      return Vec2i(
        x_start + (1 + noLaneFoundCounter) * height / tan(laneAngle) * scale,
//          x_start + (1 + noLaneFoundCounter) * width / tan(laneAngle) * scale,
//        y_start - (1 + noLaneFoundCounter) * tan(laneAngle) * width * scale);
        y_start - (1 + noLaneFoundCounter) * height * scale);
    }
    if (laneAngle > splitAngle) {
      //cout << "laneAngle > splitAngle\n";
      //RCLCPP_INFO(this->get_logger(), "checkpoint1");
      return Vec2i(
        x_start + (1 + noLaneFoundCounter) * height / tan(laneAngle) * scale,
        y_start - (1 + noLaneFoundCounter) * height * scale);
    }

  } else {
    //RCLCPP_INFO(this->get_logger(), "checkpoint3");
    if (laneAngle == splitAngle + CV_PI / 2) {
      return Vec2i(
        x_start - (1 + noLaneFoundCounter) * width,
        y_start - (1 + noLaneFoundCounter) * height);

    }
    if (laneAngle < splitAngle + CV_PI / 2) {
      return Vec2i(
        x_start + (1 + noLaneFoundCounter) * height / tan(laneAngle),
        y_start - (1 + noLaneFoundCounter) * height);
    } else {  // laneAngle>splitAngle
      return Vec2i(
        x_start - (1 + noLaneFoundCounter) * width, y_start + (1 + noLaneFoundCounter) * tan(
          laneAngle) * width);
    }
  }
  //warning: control reaches end of non-void function [-Wreturn-type]
  //Is the same return value how in the last else statement
  return Vec2i(
    x_start - (1 + noLaneFoundCounter) * width, y_start + (1 + noLaneFoundCounter) * tan(
      laneAngle) * width);
}

bool LaneDetectionNode::pointOutOfImage(Vec2i & point, int windowWidth, int windowHeight)
{
  if (point[0] >= 480 - windowWidth - 1) {
    point[0] = 480 - 2 - windowWidth;
  }
  if (point[0] < 0) {
    point[0] = 0;
  }
  if (point[1] < 0 or point[1] > 1280 - windowHeight) {
    return true;
  }
  return false;
}

double LaneDetectionNode::calculateAngles(Point startPoint, Point endPoint)
{
  int x_start = startPoint.x;
  int x_end = endPoint.x;
  int y_start = startPoint.y;
  int y_end = endPoint.y;
  if (endPoint.x > startPoint.x and endPoint.y > startPoint.y) {
    x_end = startPoint.x;
    x_start = endPoint.x;
  }
  double distance = sqrt(pow((x_end - x_start), 2) + pow((y_end - y_start), 2));
  double angle = acos((x_end - x_start) / distance);
  if (angle > CV_PI) {
    angle -= CV_PI;
  }
  return angle;
}
/*
Point LaneDetectionNode::getPointFormAngleAndDistance(
  Point point, double angle, double distance,
  bool rightLine)
{
  if (rightLine) {
    //double x2 = point.x + distance * cos(angle + CV_PI / 2);
    //double y2 = point.y - distance * sin(angle + CV_PI / 2);
    //return Point(x2, y2);
    return Point(
      point.x + distance * cos(angle + CV_PI / 2),
      point.y - distance * sin(angle + CV_PI / 2));
  } else {
    //double x2 = point.x + distance * cos(angle - CV_PI / 2);
    //double y2 = point.y - distance * sin(angle - CV_PI / 2);

    //return Point(x2, y2);
    return Point(
      point.x + distance * cos(angle - CV_PI / 2),
      point.y - distance * sin(angle - CV_PI / 2));
  }
}
*/
