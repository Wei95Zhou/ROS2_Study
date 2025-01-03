/*
    需求：启动 turtlesim_node 节点，编写程序，发布乌龟(turtle1)相对于窗体(world)的位姿
    步骤：
        1.包含头文件；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.创建一个动态广播器
            3-2.创建一个乌龟位姿订阅方
            3-3.回调函数中，获取乌龟位姿，并生成相对关系，然后发布
        4.调用spin函数，并传入自定义类对象指针；
        5.释放资源。
*/

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

// 3.定义节点类；
class TFDynamicBroadcaster : public rclcpp::Node{
public:
    TFDynamicBroadcaster():Node("tf_dynamic_broadcaster_node_cpp"){
        // 3-1.创建一个动态广播器
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        // 3-2.创建一个乌龟位姿订阅方
        pose_sub_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, 
            std::bind(&TFDynamicBroadcaster::do_pose, this, std::placeholders::_1));
        
    }
private:
    // 3-3.回调函数中，获取乌龟位姿，并生成相对关系，然后发布
    void do_pose(const turtlesim::msg::Pose & pose){
        // 组织消息
        geometry_msgs::msg::TransformStamped ts;
        ts.header.stamp = this->now();
        ts.header.frame_id = "world";
        ts.child_frame_id = "turtle1";
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
    rclcpp::spin(std::make_shared<TFDynamicBroadcaster>());
    // 5.释放资源。
    rclcpp::shutdown();
    return 0;
}
