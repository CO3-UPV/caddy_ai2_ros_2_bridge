#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/imu.hpp"
using std::placeholders::_1;

#include <cstdio>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include <async-sockets/udpsocket.hpp>
#include <async-sockets/udpserver.hpp>
#include <iostream>
#include <chrono>
#include <mutex>

class MinimalNode : public rclcpp::Node
{
  private:
    UDPSocket<512> * udpSocket; // "true" to use Connection on UDP. Default is "false".

    UDPServer<512> * udpServer_1;
    UDPServer<512> * udpServer_2;
    UDPServer<512> * udpServer_3;

    bool _ros_1_server_binded_1_ = true;
    std::mutex mtx_1;
    bool _ros_1_server_binded_2_ = true;
    std::mutex mtx_2;
    bool _ros_1_server_binded_3_ = true;
    std::mutex mtx_3;

    bool _ros_1_new_message_1_ = true;
    json _pub_json_1_;
    bool _ros_1_new_message_2_ = true;
    json _pub_json_2_;
    bool _ros_1_new_message_3_ = true;
    json _pub_json_3_;

    std::string sub_topic;
    std::string ros_1_server_ip;
    uint16_t ros_1_server_port;
    std::string pub_topic_1;
    uint16_t ros_2_server_port_1;
    std::string pub_topic_2;
    uint16_t ros_2_server_port_2;
    std::string pub_topic_3;
    uint16_t ros_2_server_port_3;
    uint16_t Hz;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _pub_1_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _pub_2_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _pub_3_;

    void timer_callback()
    {
      if(_ros_1_new_message_1_){
        auto message_1 = std_msgs::msg::Float32();
        mtx_1.lock();
        message_1.data = _pub_json_1_["speed"];
        _ros_1_new_message_1_ = false;
        mtx_1.unlock();
        _pub_1_->publish(message_1);
      }
      if(_ros_1_new_message_2_){
        auto message_2 = sensor_msgs::msg::Imu();
        mtx_2.lock();
        message_2.orientation.x = _pub_json_2_["orientation"]["x"];
        message_2.orientation.y = _pub_json_2_["orientation"]["y"];
        message_2.orientation.z = _pub_json_2_["orientation"]["z"];
        message_2.orientation.w = _pub_json_2_["orientation"]["w"];
        message_2.angular_velocity.x = _pub_json_2_["angular_velocity"]["x"];
        message_2.angular_velocity.y = _pub_json_2_["angular_velocity"]["y"];
        message_2.angular_velocity.z = _pub_json_2_["angular_velocity"]["z"];
        message_2.linear_acceleration.x = _pub_json_2_["linear_acceleration"]["x"];
        message_2.linear_acceleration.y = _pub_json_2_["linear_acceleration"]["y"];
        message_2.linear_acceleration.z = _pub_json_2_["linear_acceleration"]["z"];
        _ros_1_new_message_2_ = false;
        mtx_2.unlock();
        _pub_2_->publish(message_2);
      }
      if(_ros_1_new_message_3_){
        auto message_3 = std_msgs::msg::Float32();
        mtx_3.lock();
        message_3.data = _pub_json_3_["steering"];
        _ros_1_new_message_3_ = false;
        mtx_3.unlock();
        _pub_3_->publish(message_3);
      }
    }

    void _sub_callback_(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr _sub_msg_) const
    {
        json _sub_json_;
        _sub_json_["drive"]["steering_angle"] = (float) _sub_msg_->drive.steering_angle;
        _sub_json_["drive"]["speed"] = (float) _sub_msg_->drive.speed;
        udpSocket->Send(_sub_json_.dump());       
    }
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr _sub_;

  public:
    MinimalNode(const rclcpp::NodeOptions &options)
        : Node("bridge_rbcar_controller_node", options)
    {
        this->declare_parameter("sub_topic", "command");
        this->declare_parameter("ros_1_server_ip", "localhost");
        this->declare_parameter("ros_1_server_port", 8888);
        this->declare_parameter("pub_topic_1", "speed");
        this->declare_parameter("ros_2_server_port_1", 8889);
        this->declare_parameter("pub_topic_2", "imu");
        this->declare_parameter("ros_2_server_port_2", 8890);
        this->declare_parameter("pub_topic_3", "steering");
        this->declare_parameter("ros_2_server_port_3", 8890);
        this->declare_parameter("Hz", 100);

        this->get_parameter("sub_topic", sub_topic);
        this->get_parameter("ros_1_server_ip", ros_1_server_ip);
        this->get_parameter("ros_1_server_port", ros_1_server_port);
        this->declare_parameter("pub_topic_1", pub_topic_1);
        this->declare_parameter("ros_2_server_port_1", ros_2_server_port_1);
        this->declare_parameter("pub_topic_2", pub_topic_2);
        this->declare_parameter("ros_2_server_port_2", ros_2_server_port_2);
        this->declare_parameter("pub_topic_3", pub_topic_3);
        this->declare_parameter("ros_2_server_port_3", ros_2_server_port_3);
        this->get_parameter("Hz", Hz);

        _sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(sub_topic, 1, std::bind(&MinimalNode::_sub_callback_, this, _1));

        udpSocket = new UDPSocket<512>(true);
        udpSocket->Connect(ros_1_server_ip.c_str(), ros_1_server_port);

        udpServer_1->Bind(_ros_1_server_port_1_, [](int errorCode, std::string errorMessage) {
            _ros_1_server_binded_1_ = false; // Error binding socket
        });
        if (!_ros_1_server_binded_1_) 
        { 
          //RCLCPP_ERROR("No se ha podido iniciar correctamente el servidor UDP - ha fallado el BINDING - posiblemente puerto ocupado por otro proceso."); 
          return -1;
        }

        udpServer_2->Bind(_ros_1_server_port_2_, [](int errorCode, std::string errorMessage) {
            _ros_1_server_binded_2_ = false; // Error binding socket
        });
        if (!_ros_1_server_binded_2_) 
        { 
          //RCLCPP_ERROR("No se ha podido iniciar correctamente el servidor UDP - ha fallado el BINDING - posiblemente puerto ocupado por otro proceso."); 
          return -1;
        }

        udpServer_3->Bind(_ros_1_server_port_3_, [](int errorCode, std::string errorMessage) {
            _ros_1_server_binded_3_ = false; // Error binding socket
        });
        if (!_ros_1_server_binded_3_) 
        { 
          //RCLCPP_ERROR("No se ha podido iniciar correctamente el servidor UDP - ha fallado el BINDING - posiblemente puerto ocupado por otro proceso."); 
          return -1;
        }

        udpServer_1->onRawMessageReceived = [&](const char* message, int length, std::string ipv4, uint16_t port) {
            mtx_1.lock();
            _pub_json_1_ = json::parse(message);
            _ros_1_new_message_1_ = true;
            mtx_1.unlock();
        };

        udpServer_2->onRawMessageReceived = [&](const char* message, int length, std::string ipv4, uint16_t port) {
            mtx_2.lock();
            _pub_json_2_ = json::parse(message);
            _ros_1_new_message_2_ = true;
            mtx_2.unlock();
        };

        udpServer_3->onRawMessageReceived = [&](const char* message, int length, std::string ipv4, uint16_t port) {
            mtx_3.lock();
            _pub_json_3_ = json::parse(message);
            _ros_1_new_message_3_ = true;
            mtx_3.unlock();
        };

        // Calcular el período en segundos como un double
        double period_seconds = 1.0 / Hz;

        // Convertir el período en microsegundos para crear una std::chrono::duration
        auto period_duration = std::chrono::duration<double>(period_seconds);

        _pub_1_ = this->create_publisher<std_msgs::msg::Float32>(pub_topic_1, 1);
        _pub_2_ = this->create_publisher<sensor_msgs::msg::Imu>(pub_topic_2, 1);
        _pub_3_ = this->create_publisher<sensor_msgs::msg::Imu>(pub_topic_3, 1);
        timer_ = this->create_wall_timer(period_duration, std::bind(&MinimalPublisher::timer_callback, this));
    }
  
    ~MinimalNode()
    {
      udpServer_1->Close();
      udpServer_2->Close();
      udpServer_3->Close();
      udpSocket->Close();
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<MinimalNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}