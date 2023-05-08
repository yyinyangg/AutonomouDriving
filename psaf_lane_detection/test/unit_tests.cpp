/**
 * @file unit_tests.cpp
 * @brief implementation of the unit_tests
 * @author PSAF
 * @date 2022-06-01
 */
#include <iostream>
#include <stdexcept>
#include <fstream>
#include <vector>
#include <string>
#include <regex>
#include <cmath>
#include <map>

#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "psaf_lane_detection/lane_detection_node.hpp"
#include "include/test_util.hpp"
#include "opencv4/opencv2/opencv.hpp"

/**
 * Generate the lookup table for the lane marking points
 * @param[in] curve_type the name of the curve type
 * @return a vector containing a map with the x and y coordinates of the lane marking points
 */
std::vector<std::map<int, int>> generate_lookup(std::string curve_type);


/**
 * @class LaneDetectionNodeTests
 * @brief Testsuite for the unit tests of the lane detection node
 * @details This testsuite contains the unit tests for the lane detection node.
 */
class LaneDetectionUnitTests : public LaneDetectionNode, public ::testing::Test
{
};

/**
 * Test if the node can resize an image
 */
TEST_F(LaneDetectionUnitTests, TestResizeImage)
{
  std::string base_path = std::string(std::getenv("TEST_DATA_DIR"));
  cv::Mat img, resized;
  img = cv::imread(base_path + "/images/testImageResize.jpg");
  resizeImage(img, resized);
  int cols = resized.cols;
  int rows = resized.rows;
  ASSERT_EQ(cols, 640) << "Expected width: 640";
  ASSERT_EQ(rows, 480) << "Expected height: 480";
}

/**
 * Test if the node does not change image with correct size
 */
TEST_F(LaneDetectionUnitTests, TestResizeImageNoChange)
{
  std::string base_path = std::string(std::getenv("TEST_DATA_DIR"));

  cv::Mat img, resized;
  img = cv::imread(base_path + "/images/testImageResize.jpg");
  // resize img to 640x480 with opencv
  cv::resize(img, img, cv::Size(640, 480));

  resizeImage(img, resized);
  int cols = resized.cols;
  int rows = resized.rows;
  ASSERT_EQ(cols, 640) << "Expected width: 640";
  ASSERT_EQ(rows, 480) << "Expected height: 480";
}

/**
 * Test if the resize function can handle an empty image
 */
TEST_F(LaneDetectionUnitTests, TestDoesNotResizeEmptyImage)
{
  cv::Mat img, resized;
  resizeImage(img, resized);
  ASSERT_TRUE(resized.empty()) << "Expected resized image to be empty";
}

/**
 * Test if the image is grayscale
 */
TEST_F(LaneDetectionUnitTests, TestGrayScaleImage)
{
  std::string base_path = std::string(std::getenv("TEST_DATA_DIR"));
  cv::Mat img, gray;
  img = cv::imread(base_path + "/images/testImageResize.jpg");
  grayscaleImage(img, gray);
  ASSERT_EQ(gray.channels(), 1) << "Expected grayscale image to have only one channel";
}

/**
 * Test the reaction to an empty image in the gray scale image function
 */
TEST_F(LaneDetectionUnitTests, TestDoesNotGrayScaleEmptyImage)
{
  cv::Mat img, gray;
  grayscaleImage(img, gray);
  ASSERT_TRUE(gray.empty()) << "Expected grayscale image to be empty";
}

/**
 * Test if the image is not changed if it is already grayscale
 */
TEST_F(LaneDetectionUnitTests, TestCanHandleGrayscaleImageAsInput)
{
  cv::Mat img, gray, returned;
  img = cv::imread(std::string(std::getenv("TEST_DATA_DIR")) + "/images/color_calib.png");
  cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
  grayscaleImage(gray, returned);
  // calcualte the absolute difference between the two images
  cv::Mat diff;
  cv::absdiff(returned, gray, diff);
  // check if the difference is zero
  ASSERT_EQ(cv::countNonZero(diff), 0) <<
    "Expected grayscale image to be equal to the original image";
}

/**
 * Test the gray scale image function
 */
TEST_F(LaneDetectionUnitTests, TestDoesGrayScaleCorrectly)
{
  std::string base_path = std::string(std::getenv("TEST_DATA_DIR"));
  cv::Mat img, gray;
  img = cv::imread(base_path + "/images/color_calib.png");
  grayscaleImage(img, gray);
  // Read colors.txt from resources folder
  std::string file_path = base_path + "/images/colors.txt";
  std::ifstream file(file_path);
  // iterate over each line in the file
  std::string line;
  while (std::getline(file, line)) {
    // line format is x, y ; r, g, b ; gray
    std::vector<std::string> tokens;
    std::regex rgx("[^;]+");
    std::sregex_token_iterator iter(line.begin(), line.end(), rgx, -1);
    std::sregex_token_iterator end;
    while (iter != end) {
      tokens.push_back(*iter++);
    }
    // get x, y
    int x = std::stoi(tokens[0]);
    int y = std::stoi(tokens[1]);
    // get r, g, b
    int r = std::stoi(tokens[2]);
    int g = std::stoi(tokens[3]);
    int b = std::stoi(tokens[4]);
    // get gray
    int gray_value = std::stoi(tokens[5]);
    // get pixel values r,g,b from original image
    int r_value = img.at<cv::Vec3b>(y, x)[2];
    int g_value = img.at<cv::Vec3b>(y, x)[1];
    int b_value = img.at<cv::Vec3b>(y, x)[0];

    // get pixel value
    int pixel_value = gray.at<uchar>(y, x);
    // check r, g, b values
    ASSERT_EQ(r_value, r) << "Expected r value: " << r;
    ASSERT_EQ(g_value, g) << "Expected g value: " << g;
    ASSERT_EQ(b_value, b) << "Expected b value: " << b;
    // compare pixel value with gray value
    ASSERT_EQ(
      pixel_value,
      gray_value) << "Expected pixel value: " << gray_value << " but got: " << pixel_value;
  }
}

/**
 * Test if image transform can handle an empty image
 */
TEST_F(LaneDetectionUnitTests, TestDoesNotTransformEmptyImage)
{
  double homography_params[9] =
  {0.000000000000000000e+00, 1.333333333333333259e+00, 0.000000000000000000e+00,
    -7.500000000000000000e-01, 0.000000000000000000e+00, 4.800000000000000000e+02,
    0.000000000000000000e+00, -0.000000000000000000e+00, 1.000000000000000000e+00};

  cv::Mat homography = cv::Mat(3, 3, CV_64F, homography_params);

  cv::Mat img, transformed;
  transformImage(img, homography, transformed);
  ASSERT_TRUE(transformed.empty()) << "Expected transformed image to be empty";
}

/**
 * Test if image transform can handle an empty homography
 */
TEST_F(LaneDetectionUnitTests, TestDoesNotTransformEmptyHomography)
{
  cv::Mat img, transformed, homography;
  transformImage(img, homography, transformed);
  ASSERT_TRUE(transformed.empty()) <<
    "Expected transformed image to be empty because homography was empty";
}

/**
 * Test homography that is not 3x3
 */
TEST_F(LaneDetectionUnitTests, TestDoesNotTransformHomographyNot3x3)
{
  double homography_params[9] =
  {0.000000000000000000e+00, 1.333333333333333259e+00, 0.000000000000000000e+00,
    -7.500000000000000000e-01, 0.000000000000000000e+00, 4.800000000000000000e+02};

  cv::Mat homography = cv::Mat(2, 3, CV_64F, homography_params);

  cv::Mat img, transformed;
  transformImage(img, homography, transformed);
  ASSERT_TRUE(transformed.empty()) <<
    "Expected transformed image to be empty because homography was not 3x3";
}

/**
 * Test if the binarization can handle an empty image
 */
TEST_F(LaneDetectionUnitTests, TestDoesNotBinarizeEmptyImage)
{
  cv::Mat img, binarized;
  binarizeImage(img, binarized);
  ASSERT_TRUE(binarized.empty()) << "Expected binarized image to be empty";
}

/**
 * Test if the binarization outputs an empty image is the lower threshold is greater than the upper threshold
 */
TEST_F(LaneDetectionUnitTests, TestResultEmptyIfLowerGreaterThanUpper)
{
  cv::Mat img, result;
  img = cv::Mat::zeros(10, 10, CV_8UC1);
  binarizeImage(img, result, 255, 0);
  ASSERT_TRUE(result.empty()) << "Expected binarized image to be empty";
}

/**
 * Test if the binarization does not outut an empty image if the lower threshold is
 * equal to the upper threshold
 */
TEST_F(LaneDetectionUnitTests, TestResultEmptyIfLowerIsEqUpper)
{
  cv::Mat img, result;
  img = cv::Mat::zeros(10, 10, CV_8UC1);
  binarizeImage(img, result, 255, 255);
  ASSERT_TRUE(result.empty()) << "Expected binarized image to be empty";
}

/**
 * Test if the binarization works correctly
*/
TEST_F(LaneDetectionUnitTests, TestDoesCreateBinaryImage)
{
  // Create a test matrix as an upper triangular matrix of size 20x20
  cv::Mat img = cv::Mat::zeros(20, 20, CV_8UC1);
  cv::Mat expected = cv::Mat::zeros(20, 20, CV_8UC1);
  for (int i = 0; i < 20; i++) {
    for (int j = 0; j < 20; j++) {
      if (i < j) {
        img.at<uchar>(i, j) = 180;
        expected.at<uchar>(i, j) = 255;
      }
    }
  }
  cv::Mat result;
  binarizeImage(img, result);
  std::vector<int> expected_vec;
  std::vector<int> result_vec;
  for (int i = 0; i < 20; i++) {
    for (int j = 0; j < 20; j++) {
      expected_vec.push_back(expected.at<uchar>(i, j));
      result_vec.push_back(result.at<uchar>(i, j));
    }
  }
  ASSERT_EQ(expected_vec, result_vec) << "Expected vector and result vector did not match";
}

/**
 * Test if the binarization does return a black image if all elements are below the threshold
 */
TEST_F(LaneDetectionUnitTests, TestDoesReturnEmptyImageIfElementsBelowThresh)
{
  // Create a test matrix as an upper triangular matrix of size 20x20
  cv::Mat img = cv::Mat::zeros(20, 20, CV_8UC1);
  cv::Mat expected = cv::Mat::zeros(20, 20, CV_8UC1);
  for (int i = 0; i < 20; i++) {
    for (int j = 0; j < 20; j++) {
      if (i < j) {
        img.at<uchar>(i, j) = 126;
      }
    }
  }
  cv::Mat result;
  binarizeImage(img, result);
  std::vector<int> expected_vec;
  std::vector<int> result_vec;
  for (int i = 0; i < 20; i++) {
    for (int j = 0; j < 20; j++) {
      expected_vec.push_back(expected.at<uchar>(i, j));
      result_vec.push_back(result.at<uchar>(i, j));
    }
  }
  ASSERT_EQ(expected_vec, result_vec) << "Expected vector and result vector did not match";
}

/**
 * Test if the lane extraction can handle an empty image
 */
TEST_F(LaneDetectionUnitTests, TestDoesNotExtractLanesEmptyImage)
{
  cv::Mat img;
  std::vector<std::vector<cv::Point>> result;
  extractLaneMarkings(img);
  result = last_lane_markings_positions_;
  ASSERT_TRUE(result.empty()) << "Expected extracted lanes to be empty";
}

/**
 * Test if the lane extraction can detect three straight lanes
 */
TEST_F(LaneDetectionUnitTests, TestExtractLanesThreeStraightLanes)
{
  std::vector<std::map<int, int>> expected_lane_markings;
  expected_lane_markings = generate_lookup("straight");
  std::string base_path = std::string(std::getenv("TEST_DATA_DIR"));
  cv::Mat img, gray, binary;
  img = cv::imread(base_path + "/images/straight.png");
  grayscaleImage(img, gray);
  binarizeImage(gray, binary);
  std::vector<std::vector<cv::Point>> result;
  extractLaneMarkings(binary);
  result = last_lane_markings_positions_;
  for (size_t i = 0; i < result.size(); i++) {
    std::vector<cv::Point> lane = result[i];
    for (size_t j = 0; j < lane.size(); j++) {
      int x, y;
      x = lane[j].x;
      y = lane[j].y;
      int expected_x = expected_lane_markings[i][y];
      ASSERT_TRUE(
        x >= expected_x - 5 &&
        x <= expected_x + 5) << "Expected point " << x <<
        " to be within 10 pixels of the lane" << expected_x;
    }
  }
}

/**
 * Test if the lane extraction can detect a double solid center lane = no overtaking
 */
TEST_F(LaneDetectionUnitTests, TestCanDetectDoubleSolidMiddle)
{
  std::string base_path = std::string(std::getenv("TEST_DATA_DIR"));
  cv::Mat img, gray, binary;
  img = cv::imread(base_path + "/images/double_solid.png");
  grayscaleImage(img, gray);
  binarizeImage(gray, binary);
  bool result;
  extractLaneMarkings(binary);
  result = no_overtaking_;
  ASSERT_TRUE(result) << "Expected no overtaking to be true";
}

/** Test if the lane extraction can detect a right curve
 * The lane markings image was generated using the following formula:
 * y = 58 * e^(-0.015 * x) and offseting the lane markings by 130/290/450 pixels
 for left/center/right lane markings respectively
 * Then, the y and x vales were swapped
 * To avoid constant recalculations, the expected x values are stored in a lookup table
 * This testcase checks, if a x coordinate of a given point matches the expected value
 * in the lookup table by checking the y value
 */
TEST_F(LaneDetectionUnitTests, TestExtractLanesRightCurve)
{
  std::vector<std::map<int, int>> expected_lane_markings;
  expected_lane_markings = generate_lookup("right_curve");
  std::string base_path = std::string(std::getenv("TEST_DATA_DIR"));
  cv::Mat img, gray, binary;
  img = cv::imread(base_path + "/images/right_curve.png");
  grayscaleImage(img, gray);
  binarizeImage(gray, binary);
  std::vector<std::vector<cv::Point>> result;
  extractLaneMarkings(binary);
  result = last_lane_markings_positions_;
  for (size_t i = 0; i < result.size(); i++) {
    std::vector<cv::Point> lane = result[i];
    for (size_t j = 0; j < lane.size(); j++) {
      int x, y;
      x = lane[j].x;
      y = lane[j].y;
      int expected_x = expected_lane_markings[i][y];
      ASSERT_TRUE(
        x >= expected_x - 5 &&
        x <=
        expected_x + 5) << "Expected point " << x <<
        " to be within 10 pixels of the lane" << expected_x;
    }
  }
}

/**
 * Test an S curve shaped layout
 * The lane markings image was generated using the following formula:
 * y = 30 * sin(x/100) +30 and offsetting the lane markings by 10/220/450 pixels
 * for left/center/right lane markings respectively
 * The x value for each y value is stored in a lookup table and used to check the result
 */
TEST_F(LaneDetectionUnitTests, TestExtractLaneSnakeCurve)
{
  std::vector<std::map<int, int>> expected_lane_markings;
  expected_lane_markings = generate_lookup("snake_curve");
  std::string base_path = std::string(std::getenv("TEST_DATA_DIR"));
  cv::Mat img, gray, binary;
  img = cv::imread(base_path + "/images/snake_curve.png");
  grayscaleImage(img, gray);
  binarizeImage(gray, binary);
  std::vector<std::vector<cv::Point>> result;
  extractLaneMarkings(binary);
  result = last_lane_markings_positions_;
  for (size_t i = 0; i < result.size(); i++) {
    std::vector<cv::Point> lane = result[i];
    for (size_t j = 0; j < lane.size(); j++) {
      int x, y;
      x = lane[j].x;
      y = lane[j].y;
      int expected_x = expected_lane_markings[i][y];
      ASSERT_TRUE(
        x >= expected_x - 5 &&
        x <=
        expected_x + 5) << "Expected point " << x <<
        " to be within 10 pixels of the lane" << expected_x;
    }
  }
}

/**
 * Test if the lane extraction does not return a lane marking if the image is empty
 */
TEST_F(LaneDetectionUnitTests, TestDoesReturnEmptyForNoMarkings)
{
  // Create an empty image 640x480x1
  cv::Mat img = cv::Mat::zeros(480, 640, CV_8UC1);
  std::vector<std::vector<cv::Point>> result;
  extractLaneMarkings(img);
  result = last_lane_markings_positions_;
  for (size_t i = 0; i < result.size(); i++) {
    ASSERT_TRUE(result[i].empty()) << "Expected extracted lane markings to be empty";
  }
}

/**
 * Test returns default values if no stopline was present in an image
 */
TEST_F(LaneDetectionUnitTests, TestDoesNotFindStopLineInEmptyImage)
{
  // Create an empty image
  cv::Mat img;
  extractStopLine(img);
  ASSERT_TRUE(last_stop_line_position_.x == -1 && last_stop_line_position_.y == -1) <<
    "Expected stop line to be not found";
  ASSERT_FALSE(stop_line_found_) << "Expected stop line to be not found";
}

/**
 * Test if a solid stop line is detected
 */
TEST_F(LaneDetectionUnitTests, TestDoesDetectSolidStopline)
{
  std::string base_path = std::string(std::getenv("TEST_DATA_DIR"));
  cv::Mat img;
  img = cv::imread(base_path + "/images/stopline_solid.png");
  extractStopLine(img);

  ASSERT_TRUE(stop_line_found_) << "Expected stop line to be found";
  ASSERT_TRUE(
    last_stop_line_position_.x < 320 && last_stop_line_position_.x > 299 &&
    last_stop_line_position_.y < 86 && last_stop_line_position_.y > 66) <<
    "Expected stop line to be found";
}

/**
 * Tests if a dashed stop line is detected (yield)
 */
TEST_F(LaneDetectionUnitTests, TestDoesDetectDashedStopLine)
{
  std::string base_path = std::string(std::getenv("TEST_DATA_DIR"));
  cv::Mat img;
  img = cv::imread(base_path + "/images/stopline_dotted.png");
  extractStopLine(img);

  ASSERT_TRUE(stop_line_found_) << "Expected stop line to be found";
  ASSERT_TRUE(
    last_stop_line_position_.x < 320 && last_stop_line_position_.x > 299 &&
    last_stop_line_position_.y < 86 && last_stop_line_position_.y > 66) <<
    "Expected stop line to be found";
}


/**
 * Test transformation with different homographies
 */
TEST_F(LaneDetectionUnitTests, TestDoesTransformCorrectly)
{
  // Create a 3x3 int matrix with entries 1-9
  double input_params[9] =
  {1, 2, 3, 4, 5, 6, 7, 8, 9};

  cv::Mat input = cv::Mat(3, 3, CV_64F, input_params);

  // Define the entries for the homography matrices
  double h_1[9] = {0.0, 1.0, 0.0,
    -1.0, 0.0, 1.0,
    0.0, -0.0, 1.0};

  double h_2[9] = {-1.0, 0.0, 1.0,
    0.0, -1.0, 1.0,
    -0.0, -0.0, 1.0};

  double h_3[9] = {0.0, -1.0, 1.0,
    1.0, 0.0, 0.0,
    -0.0, 0.0, 1.0};

  double h_4[9] = {-1.0, 0.0, 1.0,
    0.0, 1.0, 0.0,
    0.0, -0.0, 1.0};

  double h_5[9] = {0.0, 1.0, 0.0,
    -1.0, 0.0, 1.0,
    0.0, -0.0, 1.0};

  // Define the entries for expected output matrices
  double r_1[9] = {2.0, 5.0, 8.0,
    1.0, 4.0, 7.0,
    0.0, 0.0, 0.0};

  double r_2[9] = {5.0, 4.0, 0.0,
    2.0, 1.0, 0.0,
    0.0, 0.0, 0.0};

  double r_3[9] = {4.0, 1.0, 0.0,
    5.0, 2.0, 0.0,
    6.0, 3.0, 0.0};

  double r_4[9] = {2.0, 1.0, 0.0,
    5.0, 4.0, 0.0,
    8.0, 7.0, 0.0, };

  double r_5[9] = {2.0, 5.0, 8.0,
    1.0, 4.0, 7.0,
    0.0, 0.0, 0.0};

  // Create the homography matrices
  cv::Mat homograhpy_1 = cv::Mat(3, 3, CV_64F, h_1);
  cv::Mat homograhpy_2 = cv::Mat(3, 3, CV_64F, h_2);
  cv::Mat homograhpy_3 = cv::Mat(3, 3, CV_64F, h_3);
  cv::Mat homograhpy_4 = cv::Mat(3, 3, CV_64F, h_4);
  cv::Mat homograhpy_5 = cv::Mat(3, 3, CV_64F, h_5);

  // Create the result matrices
  cv::Mat expected_result_1 = cv::Mat(3, 3, CV_64F, r_1);
  cv::Mat expected_result_2 = cv::Mat(3, 3, CV_64F, r_2);
  cv::Mat expected_result_3 = cv::Mat(3, 3, CV_64F, r_3);
  cv::Mat expected_result_4 = cv::Mat(3, 3, CV_64F, r_4);
  cv::Mat expected_result_5 = cv::Mat(3, 3, CV_64F, r_5);

  cv::Mat actual_result_1, actual_result_2, actual_result_3, actual_result_4, actual_result_5;

  // Run the method
  transformImage(input, homograhpy_1, actual_result_1);
  transformImage(input, homograhpy_2, actual_result_2);
  transformImage(input, homograhpy_3, actual_result_3);
  transformImage(input, homograhpy_4, actual_result_4);
  transformImage(input, homograhpy_5, actual_result_5);

  // Create an empty vector to store the results
  std::vector<double> result_1_vec;
  std::vector<double> result_2_vec;
  std::vector<double> result_3_vec;
  std::vector<double> result_4_vec;
  std::vector<double> result_5_vec;

  // Convert the results to vectors
  for (int i = 0; i < actual_result_1.rows; i++) {
    for (int j = 0; j < actual_result_1.cols; j++) {
      result_1_vec.push_back(actual_result_1.at<double>(i, j));
      result_2_vec.push_back(actual_result_2.at<double>(i, j));
      result_3_vec.push_back(actual_result_3.at<double>(i, j));
      result_4_vec.push_back(actual_result_4.at<double>(i, j));
      result_5_vec.push_back(actual_result_5.at<double>(i, j));
    }
  }

  // Create an empty vector to store the expected results
  std::vector<double> expected_result_1_vec;
  std::vector<double> expected_result_2_vec;
  std::vector<double> expected_result_3_vec;
  std::vector<double> expected_result_4_vec;
  std::vector<double> expected_result_5_vec;

  // Convert the expected results to vectors
  for (int i = 0; i < expected_result_1.rows; i++) {
    for (int j = 0; j < expected_result_1.cols; j++) {
      expected_result_1_vec.push_back(expected_result_1.at<double>(i, j));
      expected_result_2_vec.push_back(expected_result_2.at<double>(i, j));
      expected_result_3_vec.push_back(expected_result_3.at<double>(i, j));
      expected_result_4_vec.push_back(expected_result_4.at<double>(i, j));
      expected_result_5_vec.push_back(expected_result_5.at<double>(i, j));
    }
  }

  // Compare the results
  ASSERT_EQ(result_1_vec, expected_result_1_vec) <<
    "Expected result 1 to be equal to actual result 1";
  ASSERT_EQ(result_2_vec, expected_result_2_vec) <<
    "Expected result 2 to be equal to actual result 2";
  ASSERT_EQ(result_3_vec, expected_result_3_vec) <<
    "Expected result 3 to be equal to actual result 3";
  ASSERT_EQ(result_4_vec, expected_result_4_vec) <<
    "Expected result 4 to be equal to actual result 4";
  ASSERT_EQ(result_5_vec, expected_result_5_vec) <<
    "Expected result 5 to be equal to actual result 5";
}


/**
 * @brief reads from the lookup table and returns a vector containing a map for each lane
 * @param curve_type the start of the filename, i.e left_curve
 * @return vector of maps with the lookup values
 */
std::vector<std::map<int, int>> generate_lookup(std::string curve_type)
{
  std::string base_path = std::string(std::getenv("TEST_DATA_DIR"));
  std::string path_left = base_path + "/" + curve_type + "_left.txt";
  std::string path_center = base_path + "/" + curve_type + "_center.txt";
  std::string path_right = base_path + "/" + curve_type + "_right.txt";

  std::vector<std::map<int, int>> lookup_table;
  std::ifstream file_left(path_left);
  std::ifstream file_center(path_center);
  std::ifstream file_right(path_right);

  std::string line;
  std::string line_left;
  std::string line_center;
  std::string line_right;

  std::map<int, int> map_left;
  std::map<int, int> map_center;
  std::map<int, int> map_right;

  // Read the file line by line, line format is x,y
  while (std::getline(file_left, line_left)) {
    std::istringstream iss_left(line_left);
    int x, y;
    iss_left >> x >> y;
    map_left[x] = y;
  }

  while (std::getline(file_center, line_center)) {
    std::istringstream iss_center(line_center);
    int x, y;
    iss_center >> x >> y;
    map_center[x] = y;
  }

  while (std::getline(file_right, line_right)) {
    std::istringstream iss_right(line_right);
    int x, y;
    iss_right >> x >> y;
    map_right[x] = y;
  }

  lookup_table.push_back(map_left);
  lookup_table.push_back(map_center);
  lookup_table.push_back(map_right);

  return lookup_table;
}


int main(int argc, char ** argv)
{
  rclcpp::init(0, nullptr);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
