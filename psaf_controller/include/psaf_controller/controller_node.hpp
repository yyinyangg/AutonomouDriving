/**
 * @file controller_node.hpp
 * @brief Controller for the psaf 1 cars
 * @author PSAF
 * @date 2022-06-01
 */
#ifndef PSAF_CONTROLLER__CONTROLLER_NODE_HPP_
#define PSAF_CONTROLLER_CONTROLLER_NODE_HPP_

#include <string>
#include <vector>

#include "libpsaf/interface/controller_interface.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "psaf_configuration/configuration.hpp"

#define CONTROLLER_DEBUG true
#define OMEGA 1
#define THETA 2

/**
 * @class ControllerNode
 * @implements ControllerInterface
 * @brief This class is responsible for calculating the control signals for the psaf cars
 * @details This class is the main class of the controller node. It is responsible for
 * controlling the car, i.e controlling the speed and steering angle.
 * To do so, it uses the calculated trajectory and the current speed and steering angle of the
 * car to calculate the control signals.
 */
class ControllerNode : public libpsaf::ControllerInterface
{
public:
  ControllerNode();

  /**
   * @brief Method in which the results get published
   * @details This method is called periodically by the main method of the node.
   */
  void update();

protected:
  /**
   * @brief Callback method for the Trajectory message
   * @param[in] trajectory the calculated trajectory
   */
  void processTrajectory(libpsaf_msgs::msg::Trajectory::SharedPtr trajectory) override;

  /**
   * @brief Callback method for the state
   * @param[in] state the current state of the state machine
   */
  void updateState(std_msgs::msg::Int64::SharedPtr state) override;

  /**
   * @brief Callback method for the current sign
   * @param[in] sign the current sign
   */
  void processSign(libpsaf_msgs::msg::Sign::SharedPtr sign) override;

  /**
   * @brief Callback method for the stop line
   * @param line the stop line
   */
  void processStopLine(libpsaf_msgs::msg::StopLine::SharedPtr line) override;

  /**
   * @brief Callback method for the measured steering angle
   * @details Please note. At this point the requested steering angle is returned by the
   * uc_bridge instead of the measured steering angle
   * @param[in] p the current steering angle of the car
   */
  void updateSteering(std_msgs::msg::Int16::SharedPtr p) override;

  /**
   * @brief Callback method for the measured speed
   * @param[in] p the current speed of the in cm/s
   */
  void updateSpeed(std_msgs::msg::Int16::SharedPtr p) override;

  /**
  * @brief abstract function interface for receiving new sensor values
  * @param p pointer to message with new value
  */
  void updateSensorValue(sensor_msgs::msg::Imu::SharedPtr p);

  /**
   * @brief read the settings file
   * */
  void initPIController();

  /**
   *@brief transform the velocity unit from PWM frequency into cm/s
   *@param velocity, output signal from controller in the unit of cm/s
   *@param flag, if true transform velocity from km/h into cm/s
   */
  int speed_transform(double velocity, bool flag);

  /**
   *@brief initialise the parameter setting of velocity controller
   *@param ki, coefficient of Integral Part
   *@param kp, coefficient of Proportion Part
   *@param kv, scale factor of velocity, according to steering curvature
   */
  void pi2_init(double ki, double kp, double kv);

  /**
   * @brief update the current information of velocity controller
   */
  void pi2_update();

  /**
   * @brief invoke the nose filter
   * */
  void filter_imu();

  void filterOmegaWant();

  /**
   * @brief initialise the parameter setting of noise filter
   * @param sample_rate, update frequency of IMU sensor
   * @param time_coefficient, factor of noise reduction
   * */
  void filter_init(double sample_rate, double time_coefficient);

  void filterForOmegaWant_init(double sample_rate, double time_coefficient);

  /**
   * @brief transform the unit of steering angle velocity from rad/s to PWM frequency
   * @param val, angle velocity in rad/s
   * @param flag, if true, indicates that unit of val is rad/s, if false, indicates the unit of val is rad
   * */
  int steering_transform(double val, int flag);

  /**
   * @brief calculate the square distance from the interesting points to the orgin
   * @param points, the set of interesting points
   * */
  double distance_square(double * points);

  /**
   * @brief calculate the target point, to which the vehicle drives to
   * */
  void pure_pursuit();

  /**
   * @brief initialise the parameter setting of steering controller
   * @param kp, coefficient of Proportion Part
   * @param kd, coefficient of Differential Part
   * @param ki, coefficient of Integral Part
   * */
  void pid_init(double ki, double kp, double kd);

  /**
   * @brief update the current information of steering controller
   * */
  void pid_update();

  /**
   * @brief This method lets the car drive.
   */
  void Driving();

  /**
   * @brief This method print debug information.
   */
  void debug_info();

private:
  /**
   * @param imu_subscriber_, is the subscriber for the imu
   * @param current_state, current state form updateState
   * @param current_velocity, current value from velocity sensor
   * @param current_w, current value of angle velocity from low pass filter
   * @param points, set of the coordinates of interesting points
   * @param numer_of_points, the length of points
   * @param Target_points, the coordinates of target point according to pure pursuit
   * @param VMAX, the maximal allowed speed
   * @param debug_print_speed, is the speed for the debug prints
   * @param debug_print_count, is the count for the debug prints
   * */
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  int current_state;
  double current_velocity;
  double current_w;
  double points[20][2];
  const int number_of_points = 20;
  double Target_points[2];
  double VMAX;
  double debug_print_speed = 0.0;
  int debug_print_count = 0;

  /**
   * @brief PI controller, which guarantees velocity in cm/s could be reached
   * @param kv, scale factor of velocity according to curvature
   * @param error, the difference between wanted v and current v
   * @param integral, integral of error
   * @param ki, coefficient of Integral part in controller
   * @param kp, coefficient of Proportion part in controller
   * @param v_is, the current velocity of the car
   * @param v_want, velocity goal
   * @param v_out, PWM value from the controller
   * @param AW, the windup error of Integral Part
   * @param aw, coefficient of anti-windup function
   */
  struct PI_2
  {
    double kv;
    double error;
    double integral;
    double ki;
    double kp;
    double v_is;
    double v_want;
    double v_out;
    double AW;
    double aw;
  } pi2;

  /**
   * @brief PID controller of steering
   * @param error[0] current error, error[1] error of last update time
   * @param kd coefficient of Differential Part
   * @param omega_is actual steering angle
   * @param Omega_want target value of steering angle
   * @param Omega_out PWM value from the controller
   * */
  struct PID
  {
    double error[2];
    double integral;
    double ki;
    double kp;
    double kd;
    double Omega_is;
    double Omega_want;
    double Omega_out;
  } pid;

  /**
   * @brief filter out the noise of IMU sample signal
   * @param unfiltered_w[0] current data before denoise, unfiltered_w[1] row data from last sample time
   * @param filtered_w[0] current filtered data, filtered_w[1] filtered data from last update time
   * @param sample_rate frequency of sampling
   * @param time_coefficient factor describes to what extend the filter filters noise
   * */
  struct low_pass
  {
    double unfiltered_w[2];
    double filtered_w[2];
    double sample_rate;
    double time_coefficient;
  } filter1, filterForOmegaWant;
};

#endif  // PSAF_CONTROLLER_CONTROLLER_NODE_HPP_
