// You can remove or add any includes you need
#define _USE_MATH_DEFINES
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <cmath>
#include <queue>
#include <utility>
namespace solution {

class Solution final : public rclcpp::Node {
public:
  explicit Solution(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : rclcpp::Node("solution", options) {
    preempt_mode = declare_parameter<bool>("preempt", false);
    RCLCPP_INFO(get_logger(), "%s", preempt_mode ? "Preempt" : "Queue");
    _motor1 = create_publisher<std_msgs::msg::Int8>("motor_1/control", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());
    _motor2 = create_publisher<std_msgs::msg::Int8>("motor_2/control", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());

    _encoder1 = create_subscription<std_msgs::msg::UInt16>("motor_1/encoder", 10,
      [this](const std_msgs::msg::UInt16::SharedPtr msg) {
        if (preempt_mode) {
          driveMotor(_motor1, msg->data, motor_1_points_to_move, kToleranceMotor1, kGainMotor1);
        } else {
          nextPointIfDone();
          if (driveMotor(_motor1, msg->data, current_point.first, kToleranceMotor1, kGainMotor1)) {
            counter1++;
          }
          else { 
            counter1 = 0; }
          if (counter1 >= kSettleSamplesMotor1) {
            done1 = true;
          }
        }
      });

    _encoder2 = create_subscription<std_msgs::msg::UInt16>("motor_2/encoder", 10,
      [this](const std_msgs::msg::UInt16::SharedPtr msg) {
        if (preempt_mode) {
          driveMotor(_motor2, msg->data, motor_2_points_to_move, kToleranceMotor2, kGainMotor2);
        } else {
          nextPointIfDone();
          if (driveMotor(_motor2, msg->data, current_point.second, kToleranceMotor2, kGainMotor2)) {
            counter2++;
          }
          else { counter2 = 0; 
          }
          if (counter2 >= kSettleSamplesMotor2) {
            done2 = true;
          }
        }
      });

    _target = create_subscription<geometry_msgs::msg::Point>("target", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local(),
      [this](const geometry_msgs::msg::Point::SharedPtr point) {
        RCLCPP_INFO(get_logger(), "Target: (%g, %g, %g)", point->x, point->y, point->z);
        motor_1_points_to_move = -(atan2(point->y, point->x) / (2 * M_PI)) * kEncoderTicks;
        motor_2_points_to_move = (atan2(point->z, sqrt(point->x * point->x + point->y * point->y)) / (2 * M_PI)) * kEncoderTicks;
        if (motor_1_points_to_move < 0)
        {
          motor_1_points_to_move += kEncoderTicks;
        }
        if (motor_2_points_to_move < 0)
        {
          motor_2_points_to_move += kEncoderTicks;
        }
        if (!preempt_mode)
        {
          std::pair<double, double> point_calculated = {motor_1_points_to_move, motor_2_points_to_move};
          points.push(point_calculated);
        }
      });
  }

private:
  // Pozycje enkodera na obrot
  static constexpr double kEncoderTicks = 4096.0;
  static constexpr double kHalfTurn = kEncoderTicks / 2;
  //max sygnal
  static constexpr int8_t kSpeed = 100;

// Tolerancja dojazdu
  static constexpr double kToleranceMotor1 = 5.0;
  static constexpr double kToleranceMotor2 = 5.0;  
  //czas hamowania silnik 1: 10 probek
  static constexpr int kSettleSamplesMotor1 = 10;
  //czas hamowania silnik 2: 5 probek
  static constexpr int kSettleSamplesMotor2 = 6;
  //silnik 1: pelna predkosc do ok 100 tickow od celu
  static constexpr double kGainMotor1 = 1.0;
  //silnik 2: pelna predkosc do ok 60 tickow od celu
  static constexpr double kGainMotor2 = 1.5;
  //ponizej za wolno
  static constexpr int kMinSpeed = 50;

  bool driveMotor(const rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr& motor,
                  uint16_t position, double target, double tolerance, double gain)
  {
    double difference = position - target;
    if (difference > kHalfTurn)
    {
      difference -= kEncoderTicks;
    }
    else if (difference < -kHalfTurn)
    {
      difference += kEncoderTicks;
    }

    std_msgs::msg::Int8 cmd;
    if (std::abs(difference) < tolerance)
    {
      cmd.data = 0;
    }
    else
    {
      double signal = -difference * gain;
      if (signal > kSpeed)
      {
        signal = kSpeed;
      }
      else if (signal < -kSpeed)
      {
        signal = -kSpeed;
      }
      if (signal > 0 && signal < kMinSpeed)
{
  signal = kMinSpeed;
}
else if (signal < 0 && signal > -kMinSpeed)
{
  signal = -kMinSpeed;
}
      cmd.data = static_cast<int8_t>(std::round(signal));
    }

    // Poprzednie rozwiazanie (stala predkosc +/- kSpeed):
    // else if (difference > 0)
    // {
    //   cmd.data = -kSpeed;
    // }
    // else
    // {
    //   cmd.data = kSpeed;
    // }

    motor->publish(cmd);
    return std::abs(difference) < tolerance;
  }
  void nextPointIfDone()
  {
    if (done2 && done1 && !points.empty())
    {
      done1 = false;
      done2 = false;
      current_point = points.front();
      points.pop();
      counter1 = 0;
      counter2 = 0;
    }
  }

  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr _motor1;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr _motor2;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr _encoder1;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr _encoder2;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr _target;
  double motor_1_points_to_move = 0.0;
  double motor_2_points_to_move = 0.0;
  int counter1 = 0;
  int counter2 = 0;
  bool done1 = true;
  bool done2 = true;
  std::pair<double, double> current_point;
  std::queue<std::pair<double, double>> points;
  bool preempt_mode;
};

}

RCLCPP_COMPONENTS_REGISTER_NODE(solution::Solution)