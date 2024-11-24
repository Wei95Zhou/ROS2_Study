/*
    需求：编写服务端实现，解析提交的请求数据，将解析的数据相加并响应到客户端
    步骤：
        1.包含头文件；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.创建服务端
            3-2.回调函数解析请求并发送响应
        4.调用spin函数，并传入自定义类对象指针；
        5.释放资源。
*/

#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/srv/add_ints.hpp"

using base_interfaces_demo::srv::AddInts;
using std::placeholders::_1;
using std::placeholders::_2;

// 3.定义节点类；
class AddIntsServer : public rclcpp::Node{
public:
    AddIntsServer():Node("add_ints_server_node_cpp"){
        RCLCPP_INFO(this->get_logger(), "服务端节点创建");
        // 3-1.创建服务端
        server_ = this->create_service<AddInts>("add_ints", std::bind(&AddIntsServer::add, this, _1, _2));
    }
private:
    // 3-2.回调函数解析请求并发送响应
    void add(const AddInts::Request::SharedPtr req, const AddInts::Response::SharedPtr res){
        // 解析提交的数据
        res->sum = req->num1 + req->num2;
        RCLCPP_INFO(this->get_logger(), "%d + %d = %d", req->num1, req->num2, res->sum);
    }
    rclcpp::Service<AddInts>::SharedPtr server_;
};

int main(int argc, char const *argv[])
{
    // 2.初始化 ROS2 客户端；
    rclcpp::init(argc, argv);
    // 4.调用spin函数，并传入自定义类对象指针；
    rclcpp::spin(std::make_shared<AddIntsServer>());
    // 5.释放资源。
    rclcpp::shutdown();
    return 0;
}
