/*
    需求：监听坐标变换广播数据，生成turtle1相当于turtle2的坐标数据，进而生成turtle2运动的速度指令
    步骤：
        1.包含头文件；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.声明参数服务
            3-2.创建缓存
            3-3.创建监听器
            3-4.创建速度发布方
            3-5.创建定时器，实现坐标的变换，并且生成速度指令并发布
        4.调用spin函数，并传入自定义类对象指针；
        5.释放资源。
*/

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
// 3.定义节点类；
class Exer03TFListener : public rclcpp::Node{
public:
    Exer03TFListener():Node("exer03_tf_listener_node_cpp"){
        // 3-1.声明参数服务
        this->declare_parameter("father_frame", "turtle2");
        this->declare_parameter("child_frame", "turtle1");
        father_frame = this->get_parameter("father_frame").as_string();
        child_frame = this->get_parameter("child_frame").as_string();
        // 3-2.创建缓存
        buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        // 3-3.创建监听器
        listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
        // 3-4.创建速度发布方
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/" + father_frame + "/cmd_vel", 10);
        // 3-5.创建定时器，实现坐标的变换，并且生成速度指令并发布
        timer_ = this->create_wall_timer(1s, std::bind(&Exer03TFListener::on_timer, this));
    }
private:
    void on_timer(){
        try
        {
            // 1.实现坐标变换
            auto ts = buffer_->lookupTransform(father_frame, child_frame, tf2::TimePointZero);
            // 2.组织并发布速度指令
            geometry_msgs::msg::Twist twist;
            // 2-1.明确要设置的字段
            // 线速度的x，和角速度的z
            // 2-2.计数规则
            // ts包含两个坐标系的x距离和y距离
            // 线速度 = 系数 * sqrt(x*x + y*y)
            // 角速度 = 系数 * atan2(y/x)
            twist.linear.x = 0.5 * sqrt(pow(ts.transform.translation.x, 2) + pow(ts.transform.translation.y, 2));
            twist.angular.z = 1.0 * atan2(ts.transform.translation.y, ts.transform.translation.x);
            // 3.发布
            cmd_pub_->publish(twist);
        }
        catch(const tf2::LookupException& e)
        {
            RCLCPP_ERROR(this->get_logger(), "异常提示：%s", e.what());
        }
    }
    std::string father_frame, child_frame;
    std::shared_ptr<tf2_ros::Buffer> buffer_;
    std::shared_ptr<tf2_ros::TransformListener> listener_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char const *argv[])
{
    // 2.初始化 ROS2 客户端；
    rclcpp::init(argc, argv);
    // 4.调用spin函数，并传入自定义类对象指针；
    rclcpp::spin(std::make_shared<Exer03TFListener>());
    // 5.释放资源。
    rclcpp::shutdown();
    return 0;
}
