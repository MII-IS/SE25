#include <chrono>
#include <cmath>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class RobotBailarin : public rclcpp::Node
{
public:
  RobotBailarin() : Node("robot_bailarin_node")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    timer_ = this->create_wall_timer(
      50ms, std::bind(&RobotBailarin::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "¡Música maestro! El robot empieza a bailar 💃");
  }

private:
  void timer_callback()
  {
    auto message = sensor_msgs::msg::JointState();
    message.header.stamp = this->now();
    
    // CORRECCIÓN: Añadido "base_slider_joint" al principio (Total: 7 articulaciones)
    message.name = {"base_slider_joint", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

    double time = this->now().seconds();
    
    // Movimiento senoidal (baile)
    message.position = {
        0.0,                    // base_slider_joint (Lo dejamos quieto en 0.0, como en Python)
        sin(time),              // J1
        cos(time * 0.5) * 0.5,  // J2
        sin(time * 2.0) * 0.3,  // J3
        0.0,                    // J4
        sin(time),              // J5
        0.0                     // J6
    };

    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotBailarin>());
  rclcpp::shutdown();
  return 0;
}
