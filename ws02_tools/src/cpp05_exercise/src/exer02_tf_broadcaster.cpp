/*
    需求：广播不同乌龟相对于world的坐标系相对关系
    步骤：
        1.包含头文件；
        2.初始化 ROS2 客户端；
        3.定义节点类；
        4.调用spin函数，并传入自定义类对象指针；
        5.释放资源。
*/

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

// 3.定义节点类；
class Exer02TFBroadcaster : public rclcpp::Node{
public:
    Exer02TFBroadcaster():Node("tf_dynamic_broadcaster_node_cpp"){
        this->declare_parameter("turtle", "turtle1");
        turtle = this->get_parameter("turtle").as_string();
        // 3-1.创建一个动态广播器
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        // 3-2.创建一个乌龟位姿订阅方
        pose_sub_ = this->create_subscription<turtlesim::msg::Pose>("/" + turtle + "/pose", 10, 
            std::bind(&Exer02TFBroadcaster::do_pose, this, std::placeholders::_1));
        
    }
private:
    std::string turtle;
    // 3-3.回调函数中，获取乌龟位姿，并生成相对关系，然后发布
    void do_pose(const turtlesim::msg::Pose & pose){
        // 组织消息
        geometry_msgs::msg::TransformStamped ts;
        ts.header.stamp = this->now();
        ts.header.frame_id = "world";
        ts.child_frame_id = turtle;
        ts.transform.translation.x = pose.x;
        ts.transform.translation.y = pose.y;
        ts.transform.translation.z = 0.0;
        // 从欧拉角转换成四元数
        // 乌龟的欧拉角只有 yaw 的取值，没有 roll 和 pitch
        tf2::Quaternion qtn;
        qtn.setRPY(0, 0, pose.theta);
        ts.transform.rotation.x = qtn.x();
        ts.transform.rotation.y = qtn.y();
        ts.transform.rotation.z = qtn.z();
        ts.transform.rotation.w = qtn.w();

        // 发布
        broadcaster_->sendTransform(ts);
    }
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
};

int main(int argc, char const *argv[])
{
    // 2.初始化 ROS2 客户端；
    rclcpp::init(argc, argv);
    // 4.调用spin函数，并传入自定义类对象指针；
    rclcpp::spin(std::make_shared<Exer02TFBroadcaster>());
    // 5.释放资源。
    rclcpp::shutdown();
    return 0;
}
