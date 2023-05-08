/**
 * @file psaf_lane_detection/test/integration_tests.cpp
 * @brief Integration tests for the lane detection node
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

#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node_interfaces/node_graph.hpp"
#include "psaf_lane_detection/lane_detection_node.hpp"
#include "include/test_util.hpp"
#include "opencv4/opencv2/opencv.hpp"


/**
 * @class IntegrationTests
 * @brief Integration tests for the lane detection node
 * @details This test suite will test the publisher and subscriber of the lane detection node
 */
class LaneDetectionIntegrationTests : public ::testing::Test
{
public:
  /**
   * @brief Setup the test suite. Will be called before the execution every test case.
   */
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node = std::make_shared<LaneDetectionNode>();
    rclcpp::WallRate rate(std::chrono::milliseconds(30));
  }

  /**
   * @brief Tear down the test suite. Will be called after the execution every test case.
   */
  void TearDown() override
  {
    rclcpp::shutdown();
  }
  // The test object = node to test
  std::shared_ptr<LaneDetectionNode> node;
};

/**
 * Test if the lane_detection node can be created
 */
TEST(LaneDetectionStartUpTest, TestNodeCanBeCreated)
{
  // Init ros
  rclcpp::init(0, nullptr);
  // Create a ros graph
  rclcpp::node_interfaces::NodeGraph * node_graph_;
  // Create the lane detection node
  std::shared_ptr<LaneDetectionNode> node = std::make_shared<LaneDetectionNode>();
  // Initialize the node graph
  node_graph_ =
    dynamic_cast<rclcpp::node_interfaces::NodeGraph *>(node->get_node_graph_interface().get());
  // Check if the node graph is valid -> not a nullptr
  ASSERT_NE(nullptr, node_graph_);
  // Set the spin rate to 30ms
  rclcpp::WallRate rate(30.0);
  // Create a vector to store the node names
  std::vector<std::string> node_names;
  // get current time, used for shutdown
  auto now = std::chrono::system_clock::now();
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    node->update();
    node_names = node_graph_->get_node_names();
    rate.sleep();
    // after 3 seconds, stop the node
    if (std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now() -
        now).count() > 3)
    {
      break;
    }
  }
  rclcpp::shutdown();

  // Check if /lane_detection is in the node names
  ASSERT_NE(
    std::find(node_names.begin(), node_names.end(), "/lane_detection"),
    node_names.end()) << "Expected the /lane_detection in node names";
}

/**
 * Test if the lane_detection has the right amount of topics
 */
TEST_F(LaneDetectionIntegrationTests, TestTopicCount)
{
  std::map<std::string, std::vector<std::string>> topic_infos = node->get_topic_names_and_types();
  int count = static_cast<int>(topic_infos.size());
  ASSERT_EQ(count, 7) << "Expected a total topic count of 7, including rosout and parameter_events";
}

/**
 * Test if the received message by the lane detection node is correct for a black image
 */
TEST_F(LaneDetectionIntegrationTests, TestCanReceiveImageMessages)
{
  // Create an empty image
  cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
  std::vector<sensor_msgs::msg::Image> images;
  for (int i = 0; i < 10; i++) {
    std_msgs::msg::Header header;
    header.frame_id = "dummy_camera";
    header.stamp = rclcpp::Time(i);
    std::string encoding = "bgr8";
    cv_bridge::CvImage cvBridgeImage(header, encoding, image);
    sensor_msgs::msg::Image image_msg = *(cvBridgeImage.toImageMsg());
    images.push_back(image_msg);
  }

  auto dummy_node = std::make_shared<PubSubDummy<sensor_msgs::msg::Image,
      libpsaf_msgs::msg::LaneMarkings>>(
    "/color/image_raw", "/lane_detection/lane_markings", images);

  dummy_node->run();
  rclcpp::WallRate rate(std::chrono::milliseconds(30));
  while (!dummy_node->hasFinished()) {
    rclcpp::spin_some(dummy_node);
    rclcpp::spin_some(node);
    if (!dummy_node->canSend()) {
      node->update();
    }
    rate.sleep();
  }

  auto result = dummy_node->getResult();
  ASSERT_EQ(static_cast<int>(result.size()), 10);
  for (size_t i = 0; i < result.size(); i++) {
    auto left = result[i].left_lane;
    auto right = result[i].right_lane;
    auto center = result[i].center_lane;
    size_t expected = 0;
    ASSERT_EQ(left.size(), expected);
    ASSERT_EQ(right.size(), expected);
    ASSERT_EQ(center.size(), expected);
  }
}

/**
 * Test if the lane_detection node can receive state messages
 */
TEST_F(LaneDetectionIntegrationTests, TestCanReceiveStateChange)
{
  auto dummy = std::make_shared<rclcpp::Node>("dummy_node");
  auto publisher = dummy->create_publisher<std_msgs::msg::Int64>("/state_machine/state", 10);
  std_msgs::msg::Int64 state;
  state.data = 10;
  publisher->publish(state);
  rclcpp::spin_some(dummy);
  rclcpp::spin_some(node);

  ASSERT_EQ(node->getCurrentState(), 10) << "Expected the current state to be 10";
}

/**
 * Test if the lane_detection node can not send a message if it is not in the correct state
 */
TEST_F(LaneDetectionIntegrationTests, TestDoesNotSendStatusInfoWithoutStateChange)
{
  int status;
  status = -10;

  auto dummy = std::make_shared<rclcpp::Node>("dummy_node");
  auto subscriber = dummy->create_subscription<libpsaf_msgs::msg::StatusInfo>(
    "/status/status_info", 10, [&status](libpsaf_msgs::msg::StatusInfo::SharedPtr msg) {
      status = msg->type;
    });
  // Spin for 3 seconds
  rclcpp::WallRate rate(std::chrono::milliseconds(30));
  auto now = std::chrono::system_clock::now();
  while (rclcpp::ok()) {
    rclcpp::spin_some(dummy);
    rclcpp::spin_some(node);
    node->update();
    // after 3 seconds, stop the node

    if (std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now() -
        now).count() > 3)
    {
      break;
    }
  }
  ASSERT_EQ(status, -10) << "Expected no change to the initial value of -10";
}

/**
 * Test if the lane_detection node can send a status info message
 */
TEST_F(LaneDetectionIntegrationTests, TestDoesSendStatusInfoInStateTen)
{
  int status;
  status = -10;

  auto dummy = std::make_shared<rclcpp::Node>("dummy_node");
  auto subscriber = dummy->create_subscription<libpsaf_msgs::msg::StatusInfo>(
    "/status/status_info", 10, [&status](libpsaf_msgs::msg::StatusInfo::SharedPtr msg) {
      status = msg->type;
    });
  auto publisher = dummy->create_publisher<std_msgs::msg::Int64>("/state_machine/state", 10);
  std_msgs::msg::Int64 state;
  state.data = 10;
  publisher->publish(state);
  rclcpp::spin_some(dummy);
  // Spin for 3 seconds
  rclcpp::WallRate rate(std::chrono::milliseconds(30));
  auto now = std::chrono::system_clock::now();
  while (rclcpp::ok()) {
    rclcpp::spin_some(dummy);
    rclcpp::spin_some(node);
    node->update();
    rate.sleep();
    // after 3 seconds, stop the node

    if (std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now() -
        now).count() > 3)
    {
      break;
    }
  }

  ASSERT_EQ(status, 127) << "Expected 0 to be the current status";
}

/**
 * Test if the stop_line message is sent in state 10 13 14
 */
TEST_F(LaneDetectionIntegrationTests, TestDoesSendStopLineIfInCorrectStates)
{
  bool recv_stop = false;

  auto dummy = std::make_shared<rclcpp::Node>("dummy_node");
  auto subscriber = dummy->create_subscription<libpsaf_msgs::msg::StopLine>(
    "/lane_detection/stop_line", 10, [&recv_stop](libpsaf_msgs::msg::StopLine::SharedPtr msg) {
      (void) msg;
      recv_stop = true;
    });
  auto publisher = dummy->create_publisher<std_msgs::msg::Int64>("/state_machine/state", 10);

  // Spin for 3 seconds
  rclcpp::WallRate rate(std::chrono::milliseconds(30));
  auto now = std::chrono::system_clock::now();
  int counter = 0;
  while (rclcpp::ok()) {
    if (counter % 3 == 0) {
      std_msgs::msg::Int64 state;
      state.data = 10;
      publisher->publish(state);
    } else if (counter % 3 == 1) {
      std_msgs::msg::Int64 state;
      state.data = 13;
      publisher->publish(state);
    } else if (counter % 3 == 2) {
      std_msgs::msg::Int64 state;
      state.data = 14;
      publisher->publish(state);
    }
    rclcpp::spin_some(dummy);
    rclcpp::spin_some(node);
    node->update();
    rate.sleep();
    // after 3 seconds, stop the node
    if (std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now() -
        now).count() > 3)
    {
      break;
    }
  }

  ASSERT_TRUE(recv_stop) << "Expected to receive a message when in state 10 || 13 || 14";
}

/**
 * Test if an image in the wrong dimensions is resized
 */
TEST_F(LaneDetectionIntegrationTests, TestCanResizeImage)
{
  std::string base_path = std::string(std::getenv("TEST_DATA_DIR"));
  cv::Mat img;
  img = cv::imread(base_path + "/images/color_calib.png");

  // Create an empty image

  sensor_msgs::msg::Image image_msg;
  std_msgs::msg::Header header;
  header.frame_id = "test_image_resize";
  header.stamp = rclcpp::Time(0);
  std::string encoding = "bgr8";
  cv_bridge::CvImage cvBridgeImage(header, encoding, img);
  image_msg = *(cvBridgeImage.toImageMsg());

  auto dummy = std::make_shared<rclcpp::Node>("dummy_node");
  auto publisher = dummy->create_publisher<sensor_msgs::msg::Image>(
    "/color/image_raw", 10);
  publisher->publish(image_msg);
  rclcpp::WallRate rate(std::chrono::milliseconds(30));
  auto now = std::chrono::system_clock::now();
  while (rclcpp::ok()) {
    rclcpp::spin_some(dummy);
    rclcpp::spin_some(node);
    // after 1 seconds, stop the node

    if (std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now() -
        now).count() > 1)
    {
      break;
    }
  }

  cv::Mat result = node->getLastImage();
  // Get width and height of the result image
  int width = result.cols;
  int height = result.rows;
  // Check if the image is resized
  ASSERT_EQ(width, 640) << "Expected width to be 320";
  ASSERT_EQ(height, 480) << "Expected height to be 480";
}


int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
