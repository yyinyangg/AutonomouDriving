/**
 * @file controller_node.cpp
 * @brief implementation of the controller
 * @author PSAF
 * @date 2022-06-01
 */
#include "psaf_controller/controller_node.hpp"
#include <vector>
#include <string>
#include <iostream>

ControllerNode::ControllerNode()
: ControllerInterface(
    CONTROLLER_NODE_NAME,
    STATE_TOPIC,
    TRAJECTORY_TOPIC,
    STOP_LINE_TOPIC,
    SIGN_TOPIC,
    GET_STEERING_TOPIC,
    GET_SPEED_TOPIC,
    SET_STEERING_TOPIC,
    SET_SPEED_TOPIC,
    SET_LIGHT_TOPIC,
    rclcpp::QoS(rclcpp::KeepLast {1}).best_effort().durability_volatile()
)
{
  std::function<void(sensor_msgs::msg::Imu::SharedPtr)> func =
    std::bind(&ControllerNode::updateSensorValue, this, std::placeholders::_1);
  imu_subscriber_ =
    this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu_data", rclcpp::QoS(rclcpp::KeepLast {1}).best_effort().durability_volatile(), func);

  double sample_rate = 1.0 / 30.0;
  double time_coefficient = sample_rate / 6.0;
  filter_init(sample_rate, time_coefficient);

  time_coefficient = sample_rate / 6.0;
  filterForOmegaWant_init(sample_rate, time_coefficient);

  initPIController();

  // Init points
  for (int i = 0; i < number_of_points; ++i) {
    points[i][0] = 0.5;
    points[i][1] = 100;
  }
  current_velocity = 0;
  current_state = -1;
}

void ControllerNode::update()
{
  switch (current_state) {
    case 0:
      // STARTBOX
      publishSpeed(0);
      publishSteering(39, false);
      break;
    case 10:
      // DR_NORMAL
      Driving();
      break;
    default:
      if (CONTROLLER_DEBUG) {
        Driving();
      }
      break;
  }
}

void ControllerNode::initPIController()
{
  double v_ki = declare_parameter("v_ki", 0.02);
  double v_kp = declare_parameter("v_kp", 1.0);
  double s_ki = declare_parameter("s_ki", 0.09);
  double s_kp = declare_parameter("s_kp", 1.0);
  double s_kd = declare_parameter("s_kd", 15);
  VMAX = declare_parameter("VMAX", 100.0);
  debug_print_speed = declare_parameter("debug_print_speed", 0.0);
  double kv = 0;
  pi2_init(v_ki, v_kp, kv);
  pid_init(s_ki, s_kp, s_kd);
}

void ControllerNode::processTrajectory(libpsaf_msgs::msg::Trajectory::SharedPtr trajectory)
{
  points[0][0] = trajectory.get()->points.at(0).x;
  points[0][1] = trajectory.get()->points.at(0).y;
}

void ControllerNode::updateState(std_msgs::msg::Int64::SharedPtr state)
{
  current_state = state->data;
}

void ControllerNode::processSign(libpsaf_msgs::msg::Sign::SharedPtr sign)
{
  (void)sign;
}

void ControllerNode::processStopLine(libpsaf_msgs::msg::StopLine::SharedPtr line)
{
  (void)line;
}

void ControllerNode::updateSteering(std_msgs::msg::Int16::SharedPtr p)
{
  (void)p;
}

void ControllerNode::updateSensorValue(sensor_msgs::msg::Imu::SharedPtr p)
{
  filter1.unfiltered_w[0] = p->angular_velocity.z;
  filter_imu();
  current_w = filter1.filtered_w[0];
}

void ControllerNode::updateSpeed(std_msgs::msg::Int16::SharedPtr p)
{
  current_velocity = p->data + 10 * filter1.filtered_w[0];
}

void ControllerNode::pi2_init(double ki, double kp, double kv)
{
  pi2.kv = kv;
  pi2.ki = ki;
  pi2.kp = kp;
  pi2.aw = 1 / ki;
  pi2.v_is = 0;
  pi2.v_want = 0;
  pi2.error = 0;
  pi2.integral = 0;
  pi2.v_out = 0;
}

void ControllerNode::pi2_update()
{
  double kv = pi2.kv;
  double e = pi2.error;
  double InT = pi2.integral;
  double Anti = pi2.AW;
  pi2.v_is = current_velocity;
  double v_is = pi2.v_is;
  double v_want = kv * VMAX;

  e = v_want - v_is;

  pi2.v_out = (InT * pi2.ki) + (e * pi2.kp);
  if (pi2.v_out > 2 * v_want) {
    Anti = 2 * v_want - pi2.v_out;
    InT = InT + e + Anti * pi2.aw;
    pi2.v_out = 2 * v_want;
  } else {
    InT += e;
  }
  pi2.integral = InT;
  pi2.AW = Anti;
}

int ControllerNode::speed_transform(double velocity, bool flag)
{
  if (flag == true) {
    velocity = velocity * 100 / 3.6;  // transform km/h in cm/s
  }
  if (velocity < 35.0) {
    velocity = 35.0;
  }
  return velocity * 4.33 + 98;
}
void ControllerNode::filter_init(double sample_rate, double time_coefficient)
{
  filter1.sample_rate = sample_rate;
  filter1.time_coefficient = time_coefficient;
  filter1.filtered_w[0] = 0;
  filter1.filtered_w[1] = 0;
  filter1.unfiltered_w[0] = 0;
  filter1.unfiltered_w[1] = 0;
}
void ControllerNode::filterForOmegaWant_init(double sample_rate, double time_coefficient)
{
  filterForOmegaWant.sample_rate = sample_rate;
  filterForOmegaWant.time_coefficient = time_coefficient;
  filterForOmegaWant.filtered_w[0] = 0;
  filterForOmegaWant.filtered_w[1] = 0;
  filterForOmegaWant.unfiltered_w[0] = 0;
  filterForOmegaWant.unfiltered_w[1] = 0;
}

void ControllerNode::filter_imu()
{
  double tau_s = filter1.time_coefficient;
  double tau_sw = filter1.sample_rate;

  filter1.filtered_w[0] = tau_s / (tau_s + 2 * tau_sw) *
    (filter1.unfiltered_w[0] + filter1.unfiltered_w[1]) -
    ((tau_s - 2 * tau_sw) / (tau_s + 2 * tau_sw)) * filter1.filtered_w[1];
  filter1.filtered_w[1] = filter1.filtered_w[0];
  filter1.unfiltered_w[1] = filter1.unfiltered_w[0];
}

void ControllerNode::filterOmegaWant()
{
  double tau_s = filterForOmegaWant.time_coefficient;
  double tau_sw = filterForOmegaWant.sample_rate;

  filterForOmegaWant.filtered_w[0] = tau_s / (tau_s + 2 * tau_sw) *
    (filterForOmegaWant.unfiltered_w[0] + filterForOmegaWant.unfiltered_w[1]) -
    ((tau_s - 2 * tau_sw) / (tau_s + 2 * tau_sw)) * filterForOmegaWant.filtered_w[1];
  filterForOmegaWant.filtered_w[1] = filterForOmegaWant.filtered_w[0];
  filterForOmegaWant.unfiltered_w[1] = filterForOmegaWant.unfiltered_w[0];
}

int ControllerNode::steering_transform(double val, int flag)
{
  int rtn = 0;
  if (flag == OMEGA) {
    rtn = static_cast<int>(-436.43 * val + 34.12);
  } else if (flag == THETA) {
    rtn = static_cast<int>(-877.74 * val + 33.98);
  }
  return rtn;
}

double ControllerNode::distance_square(double * points)
{
  return points[0] * points[0] + points[1] * points[1];
}

void ControllerNode::pure_pursuit()
{
  double v = current_velocity;
  Target_points[0] = points[0][0];
  Target_points[1] = points[0][1];

  filterForOmegaWant.unfiltered_w[0] = -(2 * Target_points[0] * v) / distance_square(Target_points);
  filterOmegaWant();
  pid.Omega_want = filterForOmegaWant.filtered_w[0];
}

void ControllerNode::pid_init(double ki, double kp, double kd)
{
  pid.ki = ki;
  pid.kp = kp;
  pid.kd = kd;
  pid.error[0] = 0;
  pid.error[1] = 0;
  pid.integral = 0;
}

void ControllerNode::pid_update()
{
  double e = pid.error[0];
  double e_pre = pid.error[1];
  double InT = pid.integral;
  pid.Omega_is = filter1.filtered_w[0];
  double Omega_is = pid.Omega_is;
  double Omega_want = pid.Omega_want;
  double curvature = 0.0, d_curvature = 0.0;
  double d_w = Omega_want - current_w;
  if (current_velocity == 0.0) {d_curvature = 0.0;} else {
    curvature = std::abs(Omega_want) / current_velocity;
    d_curvature = std::abs(d_w) / current_velocity;
  }

  pi2.kv = 1.0;
  if (curvature > 0.001) {
    pi2.kv *= 0.65;
  }
  if (d_curvature > 0.001) {
    pi2.kv *= 0.65;
  }
  e = Omega_want - Omega_is;
  InT += e;
  pid.Omega_out = InT * pid.ki + e * pid.kp + pid.kd * (e - e_pre);

  pid.error[1] = e;
  pid.integral = InT;
}

void ControllerNode::Driving()
{
  pi2_update();
  pure_pursuit();
  pid_update();
  publishSteering(steering_transform(pid.Omega_out, OMEGA), false);
  publishSpeed(speed_transform(pi2.v_out, false));
  debug_info();
}

void ControllerNode::debug_info()
{
  if (CONTROLLER_DEBUG && (debug_print_speed >= 1.0)) {
    if (debug_print_count > debug_print_speed) {
      RCLCPP_INFO(this->get_logger(), "filtered_Omega: %f", current_w);
      RCLCPP_INFO(this->get_logger(), "pid_update InT is : %f", pid.integral);
      RCLCPP_INFO(this->get_logger(), "pid_update pi2.kv : %f", pi2.kv);
      RCLCPP_INFO(this->get_logger(), "control signal: %f", pi2.v_out);
      RCLCPP_INFO(this->get_logger(), "Omega_want: %f", pid.Omega_want);
      RCLCPP_INFO(this->get_logger(), "Omega_out: %f", pid.Omega_out);
      RCLCPP_INFO(this->get_logger(), "Target_points(%f, %f)", Target_points[0], Target_points[1]);
      RCLCPP_INFO(this->get_logger(), "Measured speed :%f", current_velocity);
      debug_print_count = 0;
    }
    debug_print_count++;
  }
}
