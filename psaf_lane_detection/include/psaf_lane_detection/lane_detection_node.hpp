/**
 * @file lane_detection_node.hpp
 * @brief this class is the lane detection for the car
 * @author PSAF
 * @date 2022-06-01
 */
#ifndef PSAF_LANE_DETECTION__LANE_DETECTION_NODE_HPP_
#define PSAF_LANE_DETECTION__LANE_DETECTION_NODE_HPP_

#include <string>
#include <vector>

#include "opencv4/opencv2/opencv.hpp"
#include "opencv4/opencv2/core/cvdef.h"

#include "libpsaf/interface/lane_detection_interface.hpp"
#include "psaf_configuration/configuration.hpp"
#include "libpsaf_msgs/msg/status_info.hpp"
#include <sstream>
#include <cmath>
using namespace std;
using namespace cv;


/**
 * @class LaneDetectionNode
 * @implements LaneDetectionInterface
 * @brief The Lane detection for the car
 * @details This class is the node implementation of the lane detection.
 * It has 3 tasks:
 * 1. Calculate the position of the lane markings (left, center, right) in the image.
 * 2. Detect the start line (this is only necessary in discipline one of the carolo cup)
 * 3. Detect stop lines (this is only necessary in discipline two of the carolo cup)
 */
class LaneDetectionNode : public libpsaf::LaneDetectionInterface
{
public:
  LaneDetectionNode();

  /**
   * Use this flag to swap to a secondary algorithm at runtime. Use the following command
   * @code ros2 param set /lane_detection use_secondary_algorithm true _ @endcode
   */
  bool use_secondary_algorithm_;

  /**
   * This method is used to publish the results. It is called periodically by the main loop.
   * Call the publishers in this method.
   */
  void update();

  /**
   * This method returns the last received state of the car.
   * If no state was received, it returns -1.
   * @return the last received state
   */
  int getCurrentState();

  /**
   * This method returns the last received image. It´s used for debugging purposes.
   * @return the last received image
   */
  Mat getLastImage();

  /**
   * Homography parameters for the psaf 1 car
   * @todo replace with your own homography
   */
  double homography_data_psaf1[9] = {
    -5.945220678747534, -8.221731543083614, 2029.2412091750095, 2.159328469836063,
    -57.07733531018901, 5685.467602003643, 0.0016374458883317959, -0.036883354445061224,
    0.9999999999999999};

  /**
   * The homography parameters for the psaf 2 car
   * @todo replace with your own homography
   */
  double homography_data_psaf2[9] = {

    -5.945220678747534, -8.221731543083614, 2029.2412091750095, 2.159328469836063,
    -57.07733531018901, 5685.467602003643, 0.0016374458883317959, -0.036883354445061224,
    0.9999999999999999};

protected:
  // Variable to store the last received image
  Mat last_image_;
  // Variable to store the last calculated lane markings
  vector<vector<Point>> last_lane_markings_positions_;
  // Variable for the current driving side
  volatile int side_{-1};
  // Variable to store the information if overtaking is prohibited
  volatile bool no_overtaking_{false};
  // Variable to store the information if there is a blocked off area
  volatile bool has_blocked_area_{false};
  // Variable to store the status info to be sent next
  volatile int status_info_{127};
  // Variable to store the information if a stop line was detected
  bool stop_line_found_{false};
  // Variable to store the position of the stop line
  Point last_stop_line_position_{-1, -1};
  // Type of the stop line
  volatile unsigned int stop_line_type_{0};
  // Variable to store the current state of the state_machine
  volatile int current_state_{-1};


  /**
   * @brief Callback method for the state of the state machine
   * @param[in] state the state of the state machine
   */
  void updateState(std_msgs::msg::Int64::SharedPtr state) override;

  /**
   * @brief Callback method for the image
   * @param[in] img the camera image
   * @param[in] sensor: the position of the topic name in the topic vector
   * see configuration.hpp for more details
   */
  void processImage(Mat & img, int sensor) final;


  /**
  * Grayscale image
  * @param[in] img a color image
  * @param[out] result the grayscale image
  */
  void grayscaleImage(Mat & img, Mat & result);

  /**
   * @brief convert the input grayscale image into a binary image
   * @details this method creates a grayscale image into a binary image. Every pixel inside the
   * lower and upper threshold is set to 255, every pixel outside the threshold is set to 0.
   * @param[in] img the input grayscale image
   * @param[out] result the ouput binary image
   * @param[in] threshold_low the lower bound of the threshold. Default: 127
   * @param[in] threshold_high the upper bound of the threshold. Default: 255
   */
  void binarizeImage(
    Mat & img, Mat & result, int threshold_low = 127,
    int threshold_high = 255);

  /**
  * Transform an image, i.e birdseye view
  * @param[in] img the image to be transformed
  * @param[in] homography the homography matrix
  * @param[out] result the transformed image
  */
  void transformImage(Mat & img, Mat & homography, Mat & result);

  /**
   * Resize a given image to 640x480 Pixels
   * @param[in] image the image to be resized
   * @param[out] result the image where the result will be stored in
   */
  void resizeImage(Mat & image, Mat & result);

  /**
   * Extract the lane markings from the image
   * The results is a vector of vectors. The inner vectors contain the points of the lane markings
   * They should be left, center, right. If a certain lane is not detected, the vector will be empty
   * The inner vectors are allowed to have different sizes.
   * @param[in] img the image to be processed. This should already be the preprocessed image, i.e binarized
   */
  void extractLaneMarkings(Mat & img);

  /**
   * @brief A secondary version of the lane detection. You can swap between the used methods via
   * parameters
   * @param[in] img the image to be processed.
   */
  void extractLaneMarkingsSecondary(Mat & img);

  /**
   * Method to extract the stop line from the image
   * @param[in] img an input image, probably the binary image
   */
  void extractStopLine(Mat & img);

  /**
   * Method to apply sliding window in order to to detect curves with hough transform
   * @param[in] img input image that contains the edges that should be recognized as lines
   * @param[in] hlines vector for storing all found lines
   * @param[in] heightProp number of segments the height of the image is divided to
   * @param[in] widthProp number of segments the widht of the image is divided to
   */
  void slidingWindowHoughTransform(
    Mat & img, vector<Vec4i> & hlines, int heightProp,
    int widthProp);

  /**
 * Method that search for one coherent lane (left, right or middle)
 * @param[in] img input image that contains the edges that should be recognized as lines
 * @param[in] startPoint vector for storing all found lines
 * @param[in] windowWidth widht of window in which the method looks for a line
 * @param[in] windowHeight widht of window in which the method looks for a line
 */

  void searchForLine(Mat & img, Vec2i startPoint, int windowWidth, int windowHeight);

  /**
   * After one line segment is found this function estimates where the next lane segment should be and tries to find the next segment of the line
   * @param[in] lanePoint Point of an already found lane segment
   * @param[in] directionOfLane direction of current lane where the next lane segment is expected
   * @param[in] distance distance from current line to expected next line segment
   */
  Vec2i findNextPointOfLine(Vec2i lanePoint, int directionOfLane, int distance);

  /**
   * This method searchs for the next lane based on the location of a previously found lane
   * @param[in] currentLane vector of points with angle that represents a coherent lane
   * @param[in] distanceToNextLane estimated distance to the next lane from currentLane
   */
  void findNextLane(vector<Vec4i> currentLane, int distanceToNextLane);

  bool findLineInWindow(
    Mat & img, Mat & img2, vector<tuple<Vec2i,
    double>> & lane, int width, int height, Vec2i startPoint,
    double min_theta = 0, double max_theta = CV_PI);

  void findLane(
    Mat & img, Mat & img2, vector<vector<tuple<Vec2i, double>>> & lanes);

  void processImage(int idOfImage);
  bool findRightLane(
    Mat & img, Mat & img2, vector<tuple<Vec2i,
    double>> & rightLane, int width, int height, int search_step);
  bool findLeftLane(
    Mat & img, Mat & img2, vector<tuple<Vec2i,
    double>> & leftLane, int width, int height, int search_step);
  void findRestOfLane(
    Mat & img, Mat & img2, vector<tuple<Vec2i,
    double>> & startOfLane, int width, int height, int num);
  Vec2i getNextStartPoint(
    tuple<Vec2i, double> lastSegment, int noLaneFoundCounter,
    double splitAngle, int width, int height);
  bool pointOutOfImage(Vec2i & point, int windowWidth, int windowHeight);
  double calculateAngles(Point startPoint, Point endPoint);
  Point getPointFormAngleAndDistance(Point point, double angle, double distance, bool rightLine);

private:
};

#endif  // PSAF_LANE_DETECTION__LANE_DETECTION_NODE_HPP_
