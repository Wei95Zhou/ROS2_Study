/*
  需求：订阅学生信息并打印在终端
  步骤：
    1.包含头文件；
    2.初始化 ROS2 客户端；
    3.定义节点类；
        3-1.创建订阅方
        3-2.回调函数订阅并解析数据
    4.调用spin函数，并传入自定义类对象指针；
    5.释放资源。
*/

#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/msg/student.hpp"

using base_interfaces_demo::msg::Student;

// 3.定义节点类；
class ListenerStu : public rclcpp::Node{
public:
    ListenerStu():Node("listenerstu_node_cpp"){
        // 3-1.创建订阅方
        subscription_ = this->create_subscription<Student>("chatter_stu", 10, std::bind(&ListenerStu::do_cb, this, std::placeholders::_1));
    }
private:
    // 3-2.回调函数订阅并解析数据
    void do_cb(const Student &stu){
        RCLCPP_INFO(this->get_logger(), "订阅的学生信息：name=%s, age=%d, height=%.2f", stu.name.c_str(), stu.age, stu.height);
    }
    rclcpp::Subscription<Student>::SharedPtr subscription_;
};

int main(int argc, char const *argv[])
{
  // 2.初始化 ROS2 客户端；
  rclcpp::init(argc, argv);
  // 4.调用spin函数，并传入自定义类对象指针；
  rclcpp::spin(std::make_shared<ListenerStu>());
  // 5.释放资源。
  rclcpp::shutdown();
  return 0;
}
