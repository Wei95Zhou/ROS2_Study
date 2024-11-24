/*
  需求：以固定频率发布学生信息
  步骤：
    1.包含头文件；
    2.初始化 ROS2 客户端；
    3.定义节点类；
        3-1.创建发布方
        3-2.创建定时器
        3-3.组织并发布消息
    4.调用spin函数，并传入自定义类对象指针；
    5.释放资源。
*/

#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/msg/student.hpp"

using base_interfaces_demo::msg::Student;
using namespace std::chrono_literals;
// 3.定义节点类；
class TalkerStu : public rclcpp::Node{
public:
    TalkerStu():Node("talkerstu_node_cpp"){
        // 3-1.创建发布方
        publisher_ = this->create_publisher<Student>("chatter_stu", 10);
        // 3-2.创建定时器
        timer_ = this->create_wall_timer(1s, std::bind(&TalkerStu::on_timer, this));
        
    }
private:
    // 3-3.组织并发布消息
    void on_timer(){
        auto stu = Student();
        stu.name = "zw";
        stu.age = 8;
        stu.height = 1.8;
        publisher_->publish(stu);
        RCLCPP_INFO(this->get_logger(), "发布的消息：(%s, %d, %.2f)", stu.name.c_str(), stu.age, stu.height);
    }
    rclcpp::Publisher<Student>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char const *argv[])
{
    // 2.初始化 ROS2 客户端；
    rclcpp::init(argc, argv);
    // 4.调用spin函数，并传入自定义类对象指针；
    rclcpp::spin(std::make_shared<TalkerStu>());
    // 5.释放资源。
    rclcpp::shutdown();
    return 0;
}
