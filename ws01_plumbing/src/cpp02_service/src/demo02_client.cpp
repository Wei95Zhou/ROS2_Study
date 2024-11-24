/*
    需求：创建客户端，组织数据并提交，然后处理响应结果（需要关注业务流程）
    步骤：
        前提：判断main函数提交的参数是否正确
        1.包含头文件；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.创建客户端
            3-2.连接服务器，如果连接不到服务器，则不能发送请求
            3-3.发送请求
        4.创建对象指针；
            调用连接服务的函数，根据连接结果做下一步处理
            连接服务后，调用请求发送函数
            再处理响应结果
        5.释放资源。
*/

#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/srv/add_ints.hpp"

using base_interfaces_demo::srv::AddInts;
using namespace std::chrono_literals;

// 3.定义节点类；
class AddIntsClient : public rclcpp::Node{
public:
    AddIntsClient():Node("add_ints_client_node_cpp"){
        RCLCPP_INFO(this->get_logger(), "客户端创建");
        // 3-1.创建客户端
        client_ = this->create_client<AddInts>("add_ints");
    }
    // 3-2.连接服务器，如果连接不到服务器，则不能发送请求
    // 连接成功返回true，否则返回false
    bool connect_server(){
        // 在指定超时时间内连接服务器，如果连接上了，返回true，否则返回false
        // client_->wait_for_service(1s);
        while (!client_->wait_for_service(1s)) //循环以1s为超时时间连接服务器，直到连接成功才退出循环
        {
            // 对 ctrl+c 操作做出特殊处理
            // 1. 判断ctrl+c按下，按下后要结束程序，意味着要释放资源，关闭context
            if (!rclcpp::ok())
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "强行终止客户端");
                return false;
            }
            
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "服务连接中");
        }
        
        return true;
    }
    // 3-3.发送请求
    // 编写发送请求函数 ———— 参数是两个整型数据，返回值是提交请求后服务端的返回结果
    rclcpp::Client<AddInts>::FutureAndRequestId send_request(int num1, int num2){
        // 组织请求数据

        // 发送
        /*
            rclcpp::Client<base_interfaces_demo::srv::AddInts>::FutureAndRequestId 
            async_send_request(std::shared_ptr<base_interfaces_demo::srv::AddInts_Request> request) // AddInts::Request
        */
        auto request = std::make_shared<AddInts::Request>();
        request->num1 = num1;
        request->num2 = num2;
        return client_->async_send_request(request);
    }
private:
    rclcpp::Client<AddInts>::SharedPtr client_;
};

int main(int argc, char const *argv[])
{
    if(argc != 3) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "请提交两个整型数字！");
        return 1;
    }
    // 2.初始化 ROS2 客户端；
    rclcpp::init(argc, argv);
    // 创建客户端对象
    auto client = std::make_shared<AddIntsClient>();
    // 调用客户端对象的连接服务器功能
    bool flag = client->connect_server();
    // 根据连接结果做进一步处理
    if(!flag){
        /* 
            rclcpp::get_logger("name") 创建logger对象不依赖于context
         */
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "服务器连接失败！");
        // RCLCPP_INFO(client->get_logger(), "服务器连接失败！");
        return 0;
    }
    // 执行后续操作
    // 调用请求提交函数，接收并处理响应结果
    auto future = client->send_request(atoi(argv[1]), atoi(argv[2]));
    // 处理响应
    if (rclcpp::spin_until_future_complete(client, future) == rclcpp::FutureReturnCode::SUCCESS) //成功
    {
        RCLCPP_INFO(client->get_logger(), "响应成功！sum = %d", future.get()->sum);
    }
    else
    {
        RCLCPP_INFO(client->get_logger(), "响应失败");
    }

    // 5.释放资源。
    rclcpp::shutdown();
    return 0;
}
