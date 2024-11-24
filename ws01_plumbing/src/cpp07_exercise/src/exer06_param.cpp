/*
    需求：修改 turtlesim_node 的背景色
    步骤：
        1.包含头文件；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.创建参数客户端
            3-2.连接参数服务端
            3-3.更新参数
        4.创建节点对象指针，并调用其函数；
        5.释放资源。
*/

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
// 3.定义节点类；
class Exer06Param : public rclcpp::Node{
public:
    Exer06Param():Node("exer06_param_node_cpp"){
        RCLCPP_INFO(this->get_logger(), "参数客户端！");
        // 3-1.创建参数客户端
        client_ = std::make_shared<rclcpp::SyncParametersClient>(this, "/turtlesim");
    }
    // 3-2.连接参数服务端
    bool connect_server(){
        while (!client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "客户端强制退出");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "服务连接中！");
        }
        
        return true;
    }
    // 3-3.更新参数
    void update_param(){
        // 背景色要渐变
        // background_r [0, 255]，以5为步进，逐渐变大、变小
        // 1.获取参数
        int red = client_->get_parameter<int>("background_r");
        // 2.循环修改参数（通过休眠控制修改频率）
        rclcpp::Rate rate(30.0);
        // 需求：背景色由浅变深，再由深变浅
        int flag = 1;
        while (rclcpp::ok())
        {
            red += flag * 5;
            if(red <= 0 || red >= 255) flag *= -1;

            // 修改服务端参数
            client_->set_parameters({rclcpp::Parameter("background_r", red)});
            rate.sleep();
        }
        
    }
private:
    rclcpp::SyncParametersClient::SharedPtr client_;
};

int main(int argc, char const *argv[])
{
    // 2.初始化 ROS2 客户端；
    rclcpp::init(argc, argv);
    // 4.创建节点对象指针，并调用其函数
    auto client = std::make_shared<Exer06Param>();
    if(!client->connect_server()) return 1;
    client->update_param();
    // 调用函数
    // 5.释放资源。
    rclcpp::shutdown();
    return 0;
}
