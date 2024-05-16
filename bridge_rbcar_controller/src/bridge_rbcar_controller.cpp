#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
using std::placeholders::_1;

#include <cstdio>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include <async-sockets/udpsocket.hpp>
#include <iostream>

class MinimalSubscriber : public rclcpp::Node
{
  private:
    UDPSocket<512> * udpSocket; // "true" to use Connection on UDP. Default is "false".

    std::string sub_topic;
    std::string ros_1_server_ip;
    uint16_t ros_1_server_port;
    uint16_t ros_2_server_port;
    uint16_t Hz;

    void _sub_callback_(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr _sub_msg_) const
    {
        json _sub_json_;
        _sub_json_["drive"]["steering_angle"] = (float) _sub_msg_->drive.steering_angle;
        _sub_json_["drive"]["speed"] = (float) _sub_msg_->drive.speed;
        udpSocket->Send(_sub_json_.dump());       
    }
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr _sub_;

  public:
    MinimalSubscriber(const rclcpp::NodeOptions &options)
        : Node("bridge_rbcar_controller_node", options)
    {
        this->declare_parameter("sub_topic", "command");
        this->declare_parameter("ros_1_server_ip", "localhost");
        this->declare_parameter("ros_1_server_port", 8888);
        this->declare_parameter("ros_2_server_port", 8889);
        this->declare_parameter("Hz", 25);

        this->get_parameter("sub_topic", sub_topic);
        this->get_parameter("ros_1_server_ip", ros_1_server_ip);
        this->get_parameter("ros_1_server_port", ros_1_server_port);
        this->get_parameter("ros_2_server_port", ros_2_server_port);
        this->get_parameter("Hz", Hz);

        _sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        sub_topic, 1, std::bind(&MinimalSubscriber::_sub_callback_, this, _1));

        udpSocket = new UDPSocket<512>(true);
        udpSocket->Connect(ros_1_server_ip.c_str(), ros_1_server_port);
    }
  
    ~MinimalSubscriber()
    {
      udpSocket->Close();
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<MinimalSubscriber>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}