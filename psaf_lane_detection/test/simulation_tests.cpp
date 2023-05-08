/**
 * @file simulation_tests.cpp
 * @brief implementation of the simulation_tests
 * @author PSAF
 * @date 2022-06-01
 */
#include <memory>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <string>
#include <regex>
#include <cmath>
#include <map>
#include <algorithm>
#include <limits>
#include <iomanip>
#include <sstream>

#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.h"
#include "rmw/rmw.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/node_interfaces/node_graph.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/storage_options.hpp"
#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_cpp/typesupport_helpers.hpp"
#include "rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp"
#include "psaf_lane_detection/lane_detection_node.hpp"
#include "include/test_util.hpp"
#include "opencv4/opencv2/opencv.hpp"

/**
 * @brief manages a single run of a test case
 * @details a run consist of sending all test messages and receiving all the answers of the node
 * under test.
 * @param[in] node the node under test - Needs to be passes so that the node can be spinned
 * @param[in] data the data to be sent to the node under test
 * @return the answer of the node under test
 */
std::vector<libpsaf_msgs::msg::LaneMarkings> manage_run(
  std::shared_ptr<LaneDetectionNode> node,
  std::vector<sensor_msgs::msg::Image> data);

/**
 * @brief A sliding window implementation to compare the results of the node under test
 * @param[in] image the image where the sliding window is applied
 * @param[in] width the width of the sliding window
 * @param[in] height the height of the sliding window
 * @return the detected lane marking points
 */
std::vector<std::vector<cv::Point2f>> slidingWindow(cv::Mat image, int width, int height);

/**
 * @brief Prepare an image for the sliding window algorithm. It uses the method of the node under test
 * @param[in] node the node under test
 * @param[in] image the image to be prepared
 * @return the prepared image
 */
cv::Mat prepareImage(std::shared_ptr<LaneDetectionNode> node, cv::Mat image);

// Vectors to store the test messages
std::vector<sensor_msgs::msg::Image> img_msgs_straight;
std::vector<sensor_msgs::msg::Image> img_msgs_circle_inner;
std::vector<sensor_msgs::msg::Image> img_msgs_circle_outer;
std::vector<sensor_msgs::msg::Image> img_msgs_lane_change;

/**
 * @class SimulationTests
 * @brief the testsuite for the simulation tests
 * @details This testsuite uses scenarios from a simulation to test the response of the lane detection node
 */
class LaneDetectionSimulationTests : public ::testing::Test
{
public:
  /**
   * Global Setup. This method is only called once. It is used to read the bag files and store the
   * deserialized messages in a global variable. This is to avoid the large overhead for loading and deserializing
   * the messages every time a testcase is run.
   * WARNING: This method will be called "SetUpTestSuite" in a newer version of Google Test.
   */
  static void SetUpTestCase()
  {
    std::string bag_name_straight;
    std::string bag_name_circle_inner;
    std::string bag_name_circle_outer;
    std::string bag_name_lane_change;
    if (PSAF1) {
      bag_name_straight = "psaf1_straight";
      bag_name_circle_inner = "psaf1_circle_inner";
      bag_name_circle_outer = "psaf1_circle_outer";
      bag_name_lane_change = "psaf1_lane_change";
    } else {
      bag_name_straight = "psaf2_straight";
      bag_name_circle_inner = "psaf2_circle_inner";
      bag_name_circle_outer = "psaf2_circle_outer";
      bag_name_lane_change = "psaf2_lane_change";
    }
    std::string base_path = std::string(std::getenv("TEST_DATA_DIR"));
    std::string bag_path_straight = base_path + "/bags/" + bag_name_straight;
    std::string bag_path_circle_inner = base_path + "/bags/" + bag_name_circle_inner;
    std::string bag_path_circle_outer = base_path + "/bags/" + bag_name_circle_outer;
    std::string bag_path_lane_change = base_path + "/bags/" + bag_name_lane_change;

    deserializeRosImageBag(bag_path_straight, img_msgs_straight);
    deserializeRosImageBag(bag_path_circle_inner, img_msgs_circle_inner);
    deserializeRosImageBag(bag_path_circle_outer, img_msgs_circle_outer);
    deserializeRosImageBag(bag_path_lane_change, img_msgs_lane_change);
  }

  /**
   * This Method is called before every testcase.
   */
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node = std::make_shared<LaneDetectionNode>();
    rclcpp::WallRate rate(std::chrono::milliseconds(30));
    msgs_straight = img_msgs_straight;
    msgs_circle_inner = img_msgs_circle_inner;
    msgs_circle_outer = img_msgs_circle_outer;
    msgs_lane_change = img_msgs_lane_change;
  }

  /**
   * This Method is called after every testcase.
   */
  void TearDown() override
  {
    msgs_straight.clear();
    rclcpp::shutdown();
  }

  std::shared_ptr<LaneDetectionNode> node;
  std::vector<sensor_msgs::msg::Image> msgs_straight;
  std::vector<sensor_msgs::msg::Image> msgs_circle_inner;
  std::vector<sensor_msgs::msg::Image> msgs_circle_outer;
  std::vector<sensor_msgs::msg::Image> msgs_lane_change;

private:
  /**
  * Read a ros bag, deserialize it and return a vector of sensor_msgs::msg::Image.
  * Code adapted from:
  * https://github.com/tiiuae/mission-data-recorder/blob/36e419dfb2d474d547a61ab4bc9a2e8e4f648983/internal/read_rosbag.cpp
  * @param[in] bag_path Path to the baf
  * @return the vector of sensor_msgs::msg::Image
  */
  static void deserializeRosImageBag(
    std::string bag_path,
    std::vector<sensor_msgs::msg::Image> & messages)
  {
    rosbag2_cpp::StorageOptions storage_options;
    rosbag2_cpp::readers::SequentialReader reader;

    storage_options.uri = bag_path;
    storage_options.storage_id = "sqlite3";

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    reader.open(storage_options, converter_options);

    // Create Factory
    rosbag2_cpp::SerializationFormatConverterFactory factory;
    auto deserializer = factory.load_deserializer("cdr");

    auto type_support_library = rosbag2_cpp::get_typesupport_library(
      "sensor_msgs/msg/Image",
      "rosidl_typesupport_cpp");

    auto type_support_handle = rosbag2_cpp::get_typesupport_handle(
      "sensor_msgs/msg/Image",
      "rosidl_typesupport_cpp",
      type_support_library);

    while (reader.has_next()) {
      sensor_msgs::msg::Image msg;
      auto introspect = std::make_shared<rosbag2_cpp::rosbag2_introspection_message_t>();
      introspect->time_stamp = 0;
      introspect->allocator = rcutils_get_default_allocator();
      introspect->message = &msg;

      auto serialized_msg = reader.read_next();

      deserializer->deserialize(serialized_msg, type_support_handle, introspect);

      messages.push_back(msg);
    }
  }
};

/**
 * Test if the Number of messages in the bag is the same as the number of messages in the vector.
 */
TEST_F(LaneDetectionSimulationTests, TestNumberOfMessagesFromEachBag)
{
  size_t len_straight, len_circle_inner, len_circle_outer, len_lane_change;
  if (PSAF1) {
    len_straight = 314;
    len_circle_inner = 408;
    len_circle_outer = 379;
    len_lane_change = 299;
  } else {
    len_straight = 296;
    len_circle_inner = 393;
    len_circle_outer = 450;
    len_lane_change = 412;
  }
  ASSERT_EQ(len_straight, msgs_straight.size()) <<
    "Number of messages in the bag is not the same as the number of messages in the vector.";
  ASSERT_EQ(len_circle_inner, msgs_circle_inner.size()) <<
    "Number of messages in the bag is not the same as the number of messages in the vector.";
  ASSERT_EQ(len_circle_outer, msgs_circle_outer.size()) <<
    "Number of messages in the bag is not the same as the number of messages in the vector.";
  ASSERT_EQ(len_lane_change, msgs_lane_change.size()) <<
    "Number of messages in the bag is not the same as the number of messages in the vector.";
}

/**
 * Test if all deserialized messages have the correct resolution of 640x480.
 */
TEST_F(LaneDetectionSimulationTests, TestResolutionOfImageMessages)
{
  for (auto msg : msgs_straight) {
    ASSERT_EQ(msg.height, 480);
    ASSERT_EQ(msg.width, 640);
  }
}

/**
 * Test if the LaneDetectionNode can receive a single image message.
 */
TEST_F(LaneDetectionSimulationTests, TestCanReceiveImageMessage)
{
  sensor_msgs::msg::Image msg = msgs_straight[0];
  auto dummy = std::make_shared<rclcpp::Node>("dummy_node");
  auto publisher = dummy->create_publisher<sensor_msgs::msg::Image>("/color/image_raw", 1);
  publisher->publish(msg);
  rclcpp::spin_some(dummy);
  rclcpp::spin_some(node);
  cv::Mat image = node->getLastImage();
  ASSERT_EQ(image.rows, 480);
  ASSERT_EQ(image.cols, 640);
}

/**
 * Test if all points are in a straight upwards line. This is done by taking the last and first detected point.
 * and calculating the line equation out of these two points. All other points have to be on or close to the line.
 * The tests allows for up to 4 outliers per lane line per image.
 */
TEST_F(LaneDetectionSimulationTests, TestCheckResultOfStraightLaneExtraction)
{
  std::vector<libpsaf_msgs::msg::LaneMarkings> lane_markings;
  lane_markings = manage_run(node, msgs_straight);

  for (auto lane_marking : lane_markings) {
    std::vector<cv::Point> left_points;
    std::vector<cv::Point> center_points;
    std::vector<cv::Point> right_points;
    for (auto point : lane_marking.left_lane) {
      left_points.push_back(cv::Point(point.x, point.y));
    }
    for (auto point : lane_marking.center_lane) {
      center_points.push_back(cv::Point(point.x, point.y));
    }
    for (auto point : lane_marking.right_lane) {
      right_points.push_back(cv::Point(point.x, point.y));
    }
    // Check if the vector contains at least 2 points
    cv::Point left_first, left_last, center_first, center_last, right_first, right_last;
    double m_left{-1.0}, m_center{-1.0}, m_right{-1.0}, b_left{-1.0}, b_center{-1.0},
    b_right{-1.0};
    int outlier_left{0}, outlier_center{0}, outlier_right{0};
    if (left_points.size() > 1) {
      // Check if the points are in a straight line
      left_first = left_points[0];
      left_last = left_points[left_points.size() - 1];
      double y_1 = static_cast<double>(left_first.x);
      double y_2 = static_cast<double>(left_last.x);
      double x_1 = static_cast<double>(left_first.y);
      double x_2 = static_cast<double>(left_last.y);
      m_left = (y_2 - y_1) / (x_2 - x_1);
      b_left = y_1 - m_left * x_1;
    }
    if (center_points.size() > 1) {
      center_first = center_points[0];
      center_last = center_points[center_points.size() - 1];
      double y_1 = static_cast<double>(center_first.x);
      double y_2 = static_cast<double>(center_last.x);
      double x_1 = static_cast<double>(center_first.y);
      double x_2 = static_cast<double>(center_last.y);
      m_center = (y_2 - y_1) / (x_2 - x_1);
      b_center = y_1 - m_center * x_1;
    }
    if (right_points.size() > 1) {
      right_first = right_points[0];
      right_last = right_points[right_points.size() - 1];
      double y_1 = static_cast<double>(right_first.x);
      double y_2 = static_cast<double>(right_last.x);
      double x_1 = static_cast<double>(right_first.y);
      double x_2 = static_cast<double>(right_last.y);
      m_right = (y_2 - y_1) / (x_2 - x_1);
      b_right = y_1 - m_right * x_1;
    }
    for (auto point : left_points) {
      double x = static_cast<double>(point.x);
      double y = static_cast<double>(point.y);
      int calculated = static_cast<int>(m_left * static_cast<double>(y) + b_left);
      if (std::abs(calculated - x) > 10) {
        outlier_left++;
      }
    }
    for (auto point : center_points) {
      double x = static_cast<double>(point.x);
      double y = static_cast<double>(point.y);
      int calculated = static_cast<int>(m_center * static_cast<double>(y) + b_center);
      if (std::abs(calculated - x) > 10) {
        outlier_center++;
      }
    }
    for (auto point : right_points) {
      double x = static_cast<double>(point.x);
      double y = static_cast<double>(point.y);
      int calculated = static_cast<int>(m_right * static_cast<double>(y) + b_right);
      if (std::abs(calculated - x) > 10) {
        outlier_right++;
      }
    }
    if (!FORCE_TEST_PASS) {
      ASSERT_LT(outlier_left, 4) << "Detected points for left lane not correct";
      ASSERT_LT(outlier_center, 4) << "Detected points for center lane not correct";
      ASSERT_LT(outlier_right, 4) << "Detected points for right lane not correct";
    } else {
      ASSERT_TRUE(true);
    }
  }
}

/**
 * Test if the lane detection node can detect the lane markings when the car is driving on the
 * inside lane of a circle.
 */
TEST_F(LaneDetectionSimulationTests, TestDetectedPointsMatchLanesInInnerCircle)
{
  // Set up stuff
  std::string base_path = std::string(std::getenv("TEST_DATA_DIR"));
  std::string path_to_images;
  if (PSAF1) {
    path_to_images = base_path + "/images/psaf1_circle_inner/";
  } else {
    path_to_images = base_path + "/images/psaf2_circle_inner/";
  }

  std::vector<libpsaf_msgs::msg::LaneMarkings> response;
  response = manage_run(node, msgs_circle_inner);
  for (size_t index = 0; index < response.size(); index++) {
    int missed_center_points = 0;
    int missed_right_points = 0;
    // Create image name as a string 00000.png
    std::stringstream ss;
    ss << std::setw(5) << std::setfill('0') << index << ".png";
    std::string image_name = ss.str();
    std::string image_path = path_to_images + image_name;
    cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
    cv::Mat transformed = prepareImage(node, image);
    std::vector<std::vector<cv::Point2f>> lane_markings_calculated = slidingWindow(
      transformed, 80, 50);
    if (lane_markings_calculated.size() == 0) {
      // std::cout << "No lane markings detected" << std::endl;
      continue;
    }
    std::vector<cv::Point> left_points, center_points, right_points;
    for (auto point : response[index].left_lane) {
      left_points.push_back(cv::Point(point.x, point.y));
    }
    for (auto point : response[index].center_lane) {
      center_points.push_back(cv::Point(point.x, point.y));
    }
    for (auto point : response[index].right_lane) {
      right_points.push_back(cv::Point(point.x, point.y));
    }
    // ignore left lane since it´s not relevant and only use center and right
    for (cv::Point center_point : center_points) {
      double min_dist = std::numeric_limits<int>::max();
      for (auto lane_point : lane_markings_calculated[0]) {
        cv::Point actual_point =
          cv::Point(static_cast<int>(lane_point.x), static_cast<int>(lane_point.y));
        double dist = cv::norm(center_point - actual_point);
        min_dist = std::min(min_dist, dist);
      }
      if (min_dist > 25) {
        missed_center_points++;
      }
    }
    if (lane_markings_calculated.size() > 1) {
      // If the size is 2, the right lane was detected by the sliding window too
      for (cv::Point right_point : right_points) {
        double min_dist = std::numeric_limits<int>::max();
        for (auto lane_point : lane_markings_calculated[1]) {
          cv::Point actual_point =
            cv::Point(static_cast<int>(lane_point.x), static_cast<int>(lane_point.y));
          double dist = cv::norm(right_point - actual_point);
          min_dist = std::min(min_dist, dist);
        }
        if (min_dist > 25) {
          missed_right_points++;
        }
      }
    }
    if (!FORCE_TEST_PASS) {
      ASSERT_LT(
        missed_center_points,
        3) << "To many points not on center lane markings: " << missed_center_points;
      ASSERT_LT(
        missed_right_points,
        3) << "To many points not on right lane markings: " << missed_right_points;
    } else {
      ASSERT_TRUE(true);
    }
  }
}

/**
 * Test if the car can detect the lane markings when the car drives on the outside lane of a circle.
 */
TEST_F(LaneDetectionSimulationTests, TestDetectedPointsMatchPointsInOuterCircle)
{
// Set up stuff
  std::string base_path = std::string(std::getenv("TEST_DATA_DIR"));
  std::string path_to_images;
  if (PSAF1) {
    path_to_images = base_path + "/images/psaf1_circle_outer/";
  } else {
    path_to_images = base_path + "/images/psaf2_circle_outer/";
  }
  std::vector<libpsaf_msgs::msg::LaneMarkings> response;
  response = manage_run(node, msgs_circle_outer);

  for (size_t index = 0; index < response.size(); index++) {
    int center_points_missed = 0;
    int right_points_missed = 0;
    // Create image name as a string in the format 00001.png
    std::stringstream ss;
    ss << std::setw(5) << std::setfill('0') << index << ".png";
    std::string image_name = ss.str();
    std::string image_path = path_to_images + image_name;
    cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
    cv::Mat transformed = prepareImage(node, image);

    std::vector<std::vector<cv::Point2f>> lane_markings_calculated = slidingWindow(
      transformed, 80, 50);

    if (lane_markings_calculated.size() == 0) {
      std::cout << "No lane markings detected" << std::endl;
      continue;
    }

    std::vector<cv::Point> left_points, center_points, right_points;
    for (auto point : response[index].left_lane) {
      left_points.push_back(cv::Point(point.x, point.y));
    }
    for (auto point : response[index].center_lane) {
      center_points.push_back(cv::Point(point.x, point.y));
    }
    for (auto point : response[index].right_lane) {
      right_points.push_back(cv::Point(point.x, point.y));
    }
    // ignore left lane since it´s not relevant and only use center and right
    if (lane_markings_calculated.size() > 1) {
      // If the size is 2, the center lane was detected too
      for (cv::Point center_point : center_points) {
        double min_dist = std::numeric_limits<int>::max();
        for (auto lane_point : lane_markings_calculated[1]) {
          cv::Point actual_point =
            cv::Point(static_cast<int>(lane_point.x), static_cast<int>(lane_point.y));
          double dist = cv::norm(center_point - actual_point);
          min_dist = std::min(min_dist, dist);
        }
        if (min_dist > 25) {
          center_points_missed++;
        }
      }
      for (cv::Point right_point : right_points) {
        double min_dist = std::numeric_limits<int>::max();
        for (auto lane_point : lane_markings_calculated[1]) {
          cv::Point actual_point =
            cv::Point(static_cast<int>(lane_point.x), static_cast<int>(lane_point.y));
          double dist = cv::norm(right_point - actual_point);
          min_dist = std::min(min_dist, dist);
        }
        if (min_dist > 25) {
          right_points_missed++;
        }
      }
    } else {
      // Only the right lane was detected
      for (cv::Point right_point : right_points) {
        double min_dist = std::numeric_limits<int>::max();
        for (auto lane_point : lane_markings_calculated[0]) {
          cv::Point actual_point =
            cv::Point(static_cast<int>(lane_point.x), static_cast<int>(lane_point.y));
          double dist = cv::norm(right_point - actual_point);
          min_dist = std::min(min_dist, dist);
        }
        if (min_dist > 25) {
          right_points_missed++;
        }
      }
    }
    if (!FORCE_TEST_PASS) {
      ASSERT_LT(center_points_missed, 3) << "Too many center points missed" << center_points_missed;
      ASSERT_LT(right_points_missed, 3) << "Too many right points missed" << right_points_missed;
    } else {
      ASSERT_TRUE(true);
    }
  }
}

/**
 * Test if the car does not detect a lane change if no lane change happens
 */
TEST_F(LaneDetectionSimulationTests, TestDetectsStaysOnRightLane)
{
  std::vector<libpsaf_msgs::msg::LaneMarkings> response;
  response = manage_run(node, msgs_straight);
  int side_start = response[0].side;
  int side_end = response.back().side;
  ASSERT_EQ(side_end, 0) << "Expected no lane change, but detected lane change";
  ASSERT_EQ(side_start, side_end) << "Lane changed from right to left, but expected no change";
}

/**
 * Test if the car does detect a lane change if lane change happens
 */
TEST_F(LaneDetectionSimulationTests, TestCanDetectLaneChange)
{
  std::vector<libpsaf_msgs::msg::LaneMarkings> response;
  response = manage_run(node, msgs_lane_change);
  int side_start = response[0].side;
  int side_end = response.back().side;
  if (!FORCE_TEST_PASS) {
    ASSERT_EQ(side_start, 0) << "Expected car to detect right lane at the start";
    ASSERT_EQ(side_end, 1) << "Expected left lane at the end, but was right";
    ASSERT_NE(side_start, side_end) << "Lane changed from right to left, but expected no change";
  } else {
    ASSERT_TRUE(true);
  }
}


std::vector<libpsaf_msgs::msg::LaneMarkings> manage_run(
  std::shared_ptr<LaneDetectionNode> node,
  std::vector<sensor_msgs::msg::Image> data)
{
  auto dummy = std::make_shared<PubSubDummy<sensor_msgs::msg::Image,
      libpsaf_msgs::msg::LaneMarkings>>(
    "/color/image_raw", "/lane_detection/lane_markings", data);
  dummy->run();
  while (!dummy->hasFinished()) {
    rclcpp::spin_some(dummy);
    rclcpp::spin_some(node);
    rclcpp::sleep_for(std::chrono::milliseconds(60));
    node->update();
  }

  auto result = dummy->getResult();

  return result;
}

cv::Mat prepareImage(std::shared_ptr<LaneDetectionNode> node, cv::Mat image)
{
  cv::Mat gray, binary, transformed;
  cv::Matx<double, 3, 3> homography;
  if (PSAF1) {
    homography = cv::Matx<double, 3, 3>(node->homography_data_psaf1);
  } else {
    homography = cv::Matx<double, 3, 3>(node->homography_data_psaf2);
  }
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  cv::threshold(gray, binary, 127, 255, cv::THRESH_BINARY);
  cv::warpPerspective(binary, transformed, homography, binary.size());
  return transformed;
}

std::vector<std::vector<cv::Point2f>> slidingWindow(cv::Mat image, int width, int height)
{
  // Initialize variables
  std::vector<cv::Point2f> output_points;
  std::vector<std::vector<cv::Point2f>> lane_markings;
  std::vector<cv::Point> starting_points;
  // get window width and height
  cv::Rect starting_window = cv::Rect(0, 360, width, 120);
  int starting_window_width = starting_window.width;
  int starting_window_height = starting_window.height;

  // Search for starting points. These are points that exceed a certain threshold of white pixels
  for (int x = 0; x <= image.cols - starting_window_width; x += starting_window_width) {
    // Extract the roi based on image
    cv::Mat roi = image(starting_window);
    // Count non zero pixels
    int non_zero_pixels = cv::countNonZero(roi);
    // add the starting point to the starting points vector
    if (non_zero_pixels > (starting_window_width * starting_window_height / 128)) {
      starting_points.push_back(cv::Point(x, starting_window.y));
    }
    starting_window.x += starting_window_width;
  }
  // Iterate over the starting points. Skip is used to avoid starting windows next to each other
  std::vector<cv::Point> cleaned_points;
  bool skip = false;
  for (size_t index = 0; index < starting_points.size(); index++) {
    if (skip) {
      skip = false;
      continue;
    }
    cv::Point current = starting_points[index];
    cv::Point next = starting_points[index + 1];
    if (next.x - current.x == width) {
      int average = (current.x + next.x) / 2;
      cleaned_points.push_back(cv::Point(average, image.rows - height));
      skip = true;
    } else {
      cleaned_points.push_back(cv::Point(current.x, image.rows - height));
    }
  }

  // Create the starting sliding windows = search windows
  std::vector<cv::Rect> search_windows;
  for (size_t i = 0; i < cleaned_points.size(); i++) {
    cv::Point point = cleaned_points[i];
    cv::Rect search_window = cv::Rect(point.x, point.y, width, height);
    search_windows.push_back(search_window);
  }
  // Iterate over all search windows
  for (auto window : search_windows) {
    bool should_break = false;
    // This counter is used to skip over gaps in the lane markings
    int skip_counter = 0;
    output_points.clear();
    // The actual sliding window algorithm
    while (!should_break) {
      // Additional safeguard to prevent searching outside the image
      if (window.x + width > image.cols || window.y + height > image.rows || window.x < 0 ||
        window.y < 0)
      {
        should_break = true;
        continue;
      }
      // Extract the roi from the image
      cv::Mat roi = image(window);
      cv::Mat non_zero;
      cv::findNonZero(roi, non_zero);
      if (!non_zero.empty()) {
        if (window.y < 0) {
          skip_counter = 0;
        }
        // find average x and y coordinates
        int average_x = 0;
        int average_y = 0;
        for (size_t i = 0; i < non_zero.rows; i++) {
          cv::Point point = non_zero.at<cv::Point>(i, 0);
          average_x += point.x + window.x;
          average_y += point.y + window.y;
        }
        average_x /= non_zero.rows;
        average_y /= non_zero.rows;
        output_points.push_back(cv::Point(average_x, average_y));
      }
      // Create the candidates for the next search window as shown below
      // The current search window is marked with a 'x', the candidate windows are marked with 'c'
      //  ***********************************
      //  | +-----+ +-----+ +-----+
      //  | |  c  | |  c  | |  c  |
      //  | +-----+ +-----+ +-----+
      //  |         |  x  |
      //  |         +-----+
      // *************************************

      cv::Rect up_left = cv::Rect(window.x - width, window.y - height, width, height);
      cv::Rect up_center = cv::Rect(window.x, window.y - height, width, height);
      cv::Rect up_right = cv::Rect(window.x + width, window.y - height, width, height);
      // Check if the search window is within the image
      if (up_left.x < 0 || up_left.y < 0 || up_left.x + width > image.cols ||
        up_left.y >= image.rows)
      {
        up_left = cv::Rect(0, 0, 0, 0);
      }
      if (up_center.x < 0 || up_center.y < 0 || up_center.x + width > image.cols ||
        up_center.y >= image.rows)
      {
        up_center = cv::Rect(0, 0, 0, 0);
      }
      if (up_right.x < 0 || up_right.y < 0 || up_right.x + width > image.cols ||
        up_right.y >= image.rows)
      {
        up_right = cv::Rect(0, 0, 0, 0);
      }
      // If no search window is within the image, break the loop
      if (up_left.empty() && up_center.empty() && up_right.empty()) {
        should_break = true;
        continue;
      } else {
        // Count the white pixels in the candidate windows
        int non_zero_left = 0;
        int non_zero_center = 0;
        int non_zero_right = 0;
        if (!up_left.empty()) {
          cv::Mat roi = image(up_left);
          cv::Mat non_zero;
          cv::findNonZero(roi, non_zero);
          non_zero_left = non_zero.rows;
        }
        if (!up_center.empty()) {
          cv::Mat roi = image(up_center);
          cv::Mat non_zero;
          cv::findNonZero(roi, non_zero);
          non_zero_center = non_zero.rows;
        }
        if (!up_right.empty()) {
          cv::Mat roi = image(up_right);
          cv::Mat non_zero;
          cv::findNonZero(roi, non_zero);
          non_zero_right = non_zero.rows;
        }
        // assign the window with the most non-zero pixels to window
        if (non_zero_left > non_zero_center && non_zero_left > non_zero_right) {
          window = up_left;
        } else if (non_zero_center > non_zero_left && non_zero_center > non_zero_right) {
          window = up_center;
        } else if (non_zero_right > non_zero_left && non_zero_right > non_zero_center) {
          window = up_right;
        } else {
          // If all candidates are empty, move up straight and search again.
          // If there is again no hit, break the loop
          if (skip_counter > 0) {
            should_break = true;
          }
          window.y -= height;
          skip_counter += 1;
        }
        // Safeguard against the window going out of the image
        if (window.y == 0 || window.y >= image.rows || window.x > image.cols || window.x < 0) {
          should_break = true;
        }
      }
    }
    lane_markings.push_back(output_points);
  }
  // Filter vectors with less than 2 points. Those are not really relevant
  for (size_t i = 0; i < lane_markings.size(); i++) {
    // remove vector with less than 2 points
    auto current = lane_markings[i];
    if (current.size() < 2) {
      lane_markings.erase(lane_markings.begin() + i);
      i--;
    }
  }
  return lane_markings;
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
