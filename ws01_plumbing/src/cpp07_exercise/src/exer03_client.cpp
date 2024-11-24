/*
    需求：客户端需要提交目标点坐标，并解析响应结果
    步骤：
        0.解析动态传入的数据，作为目标点坐标
        1.包含头文件；
        2.初始化 ROS2 客户端；
        3.自定义节点类；
            3-1.构造函数创建客户端对象
            3-2.客户端连接服务端
            3-3.发送请求数据
        4.调用节点对象指针的相关函数；
        5.释放资源。
*/

#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/srv/distance.hpp"

using base_interfaces_demo::srv::Distance;
using namespace std::chrono_literals;

// 3.定义节点类；
class Exer03Client : public rclcpp::Node{
public:
    Exer03Client():Node("Exer03_client_node_cpp"){
        RCLCPP_INFO(this->get_logger(), "案例2客户端创建了！");
        // 3-1.构造函数创建客户端对象
        client_ = this->create_client<Distance>("distance");
    }
    // 3-2.客户端连接服务端
    bool connect_server(){
        while (!client_->wait_for_service(1s)){
            if (!rclcpp::ok())
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "节点被强制退出！");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "服务连接中");
        }
        return true;
    }
    // 3-3.发送请求数据
    rclcpp::Client<Distance>::FutureAndRequestId send_goal(float x, float y, float theta){
        auto request = std::make_shared<base_interfaces_demo::srv::Distance_Request>();
        // auto request = std::make_shared<Distance::Request>();
        request->x = x;
        request->y = y;
        request->theta = theta;
        return client_->async_send_request(request);
    }
private:
    rclcpp::Client<Distance>::SharedPtr client_;
};

int main(int argc, char const *argv[])
{
    // 0.解析动态传入的数据，作为目标点坐标
    if (argc != 5)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "请提交x坐标、y坐标与theta三个参数");
        return 1;
    }
    // 解析提交的参数
    float goal_x = atof(argv[1]);
    float goal_y = atof(argv[2]);
    float goal_theta = atof(argv[3]);
    
    // 2.初始化 ROS2 客户端；
    rclcpp::init(argc, argv);
    // 4.调用节点对象指针的相关函数；
    auto client = std::make_shared<Exer03Client>();
    bool flag = client->connect_server();
    if (!flag){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "服务连接失败！");
    }
    // 发送请求并处理响应
    auto future = client->send_goal(goal_x, goal_y, goal_theta);
    // 判断响应结果状态
    if (rclcpp::spin_until_future_complete(client, future) == rclcpp::FutureReturnCode::SUCCESS){
        RCLCPP_INFO(client->get_logger(), "两只乌龟距离%.2f米", future.get()->distance);
    }
    else{
        RCLCPP_ERROR(client->get_logger(), "服务响应失败！");
    }

    // 5.释放资源。
    rclcpp::shutdown();
    return 0;
}
