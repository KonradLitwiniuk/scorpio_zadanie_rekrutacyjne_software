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
        if(preempt_mode){
          uint16_t position_motor_1 = msg->data;
          double difference = position_motor_1 - motor_1_points_to_move;
          if(difference > 2048)
          {
            difference -= 4096;
          }
          else if(difference < -2048)
          {
            difference += 4096;
          }
          if(abs(difference) < 10)
          {
           std_msgs::msg::Int8 cmd;
           cmd.data = 0;
            _motor1->publish(cmd);
          }
          else if(difference > 0)
          {
            std_msgs::msg::Int8 cmd;
           cmd.data = -100;
            _motor1->publish(cmd);
          }
          else{
            std_msgs::msg::Int8 cmd;
            cmd.data = 100;
            _motor1->publish(cmd);
          }
        }
        else{
          uint16_t position_motor_1 = msg->data;
          if(done2 && done1 && !points.empty())
          {
            done1 = false;
            done2 = false;
            current_point = points.front();
            points.pop();
            counter1 = 0; 
            counter2 = 0; 
          }
          double difference = position_motor_1 - current_point.first;
          if(difference > 2048)
          {
            difference -= 4096;
          }
          else if(difference < -2048)
          {
            difference += 4096;
          }
          if(abs(difference) < 10)
          {
           std_msgs::msg::Int8 cmd;
           cmd.data = 0;
            _motor1->publish(cmd);
            counter1++;
          }
          else if(difference > 0)
          {
            std_msgs::msg::Int8 cmd;
           cmd.data = -100;
            _motor1->publish(cmd);
           
          }
          else{
            std_msgs::msg::Int8 cmd;
            cmd.data = 100;
            _motor1->publish(cmd);
          }
          if(counter1 > 9){
            done1 = true;
          }
        }
      });

    _encoder2 = create_subscription<std_msgs::msg::UInt16>("motor_2/encoder", 10,
      [this](const std_msgs::msg::UInt16::SharedPtr msg) {
        if(preempt_mode){
          uint16_t position_motor_2 = msg->data;
          double difference = position_motor_2 - motor_2_points_to_move;
          if(difference > 2048)
          {
            difference -= 4096;
          }
          else if(difference < -2048)
          {
            difference += 4096;
          }
          if(abs(difference) < 10)
          {
           std_msgs::msg::Int8 cmd;
           cmd.data = 0;
            _motor2->publish(cmd);
          }
          else if(difference > 0)
          {
            std_msgs::msg::Int8 cmd;
           cmd.data = -100;
            _motor2->publish(cmd);
          
          }
          else{
            std_msgs::msg::Int8 cmd;
            cmd.data = 100;
            _motor2->publish(cmd);
          }
        }
        else{
          uint16_t position_motor_2 = msg->data;
          if(done2 && done1 && !points.empty())
          {
            done1 = false;
            done2 = false;
            current_point = points.front();
            points.pop();
            counter1 = 0; 
            counter2 = 0; 
          }
          double difference = position_motor_2 - current_point.second;
          if(difference > 2048)
          {
            difference -= 4096;
          }
          else if(difference < -2048)
          {
            difference += 4096;
          }
          if(abs(difference) < 10)
          {
           std_msgs::msg::Int8 cmd;
           cmd.data = 0;
            _motor2->publish(cmd);
                       counter2++;
          }
          else if(difference > 0)
          {
            std_msgs::msg::Int8 cmd;
           cmd.data = -100;
            _motor2->publish(cmd);

          }
          else{
            std_msgs::msg::Int8 cmd;
            cmd.data = 100;
            _motor2->publish(cmd);
          }
          if(counter2 > 9){
            done2 = true;
          }
        }
        
        
      });
    _target = create_subscription<geometry_msgs::msg::Point>("target", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local(),
      [this](const geometry_msgs::msg::Point::SharedPtr point) {
        RCLCPP_INFO(get_logger(), "Target: (%g, %g, %g)", point->x, point->y, point->z);
        motor_1_points_to_move = -(atan2(point->y, point->x) / (2 * M_PI)) * 4096; //todo zmienic to bo t nie bedzie dzialac z queue
        motor_2_points_to_move = (atan2(point->z, sqrt(point->x * point->x + point->y * point->y)) / (2 * M_PI)) * 4096;
        if (motor_1_points_to_move < 0)
        {
          motor_1_points_to_move += 4096; 
        }
        if(!preempt_mode)
        {  
          std::pair<double, double> point_calculated = {motor_1_points_to_move, motor_2_points_to_move};
          points.push(point_calculated);
        }
      });
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr _motor1;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr _motor2;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr _encoder1;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr _encoder2;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr _target;
  double motor_1_points_to_move;
  double motor_2_points_to_move;
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
