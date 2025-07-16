#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "serial_driver/serial_comm.hpp"

using std::placeholders::_1;

class SerialCmdSender : public rclcpp::Node
{
public:
    SerialCmdSender() : Node("serial_cmd_sender")
    {
        // 声明参数
        this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baudrate", 115200);

        // 获取参数
        std::string port = this->get_parameter("port").as_string();
        int baudrate = this->get_parameter("baudrate").as_int();

        // 初始化串口通信类
        comm_ = std::make_unique<SerialComm>(port, baudrate);

        // 创建订阅者
        sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&SerialCmdSender::cmdVelCallback, this, _1));
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        float vx = msg->linear.x;
        float wz = msg->angular.z;

        std::vector<float> speeds = {vx, wz};
        bool success = comm_->sendFloatArrayCommand(speeds);
        if (!success)
        {
            RCLCPP_WARN(this->get_logger(), "Send Error");
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
    std::unique_ptr<SerialComm> comm_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialCmdSender>());
    rclcpp::shutdown();
    return 0;
}
