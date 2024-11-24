/*
    需求：以某个固定频率发送文本“hello world!”，文本后缀编号，每发送一条消息，编号递增1。
    步骤：
        1.包含头文件；
        2.初始化 ROS2 客户端；
        3.定义节点类；
        4.调用spin函数，并传入自定义类对象指针；
        5.释放资源。
*/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// 3.定义节点类；
class MyNode : public rclcpp::Node{
public:
    MyNode():Node("zhen_keng", "zuoxie"){
        // 全局话题：和命名空间、节点名称无关
        // pub_ = this->create_publisher<std_msgs::msg::String>("/shi", 10);
        // 相对话题
        // pub_ = this->create_publisher<std_msgs::msg::String>("kaihui", 10);
        // 私有话题
        pub_ = this->create_publisher<std_msgs::msg::String>("~/vip", 10);
    }
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

int main(int argc, char const *argv[])
{
    // 2.初始化 ROS2 客户端；
    rclcpp::init(argc, argv);
    // 4.调用spin函数，并传入自定义类对象指针；
    rclcpp::spin(std::make_shared<MyNode>());
    // 5.释放资源。
    rclcpp::shutdown();
    return 0;
}
