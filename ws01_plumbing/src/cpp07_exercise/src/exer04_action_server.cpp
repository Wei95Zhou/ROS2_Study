/*
    需求：处理客户端发送的请求数据（目标点），控制乌龟向目标点运动，且要连续反馈剩余距离
    步骤：
        1.包含头文件；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.创建原生乌龟位姿订阅方，获取当前乌龟坐标
            3-2.创建速度指令发布方，控制乌龟运动
            3-3.创建一个动作服务端
            3-4.解析动作客户端提交的数据
            3-5.处理客户端的取消请求操作
            3-6.实现主逻辑（耗时操作），启动子线程
            3-7.子线程中，发布速度指令，产生连续反馈，并响应最终结果
        4.调用spin函数，并传入自定义类对象指针；
        5.释放资源。
*/

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces_demo/action/nav.hpp"
#include <cmath>

using base_interfaces_demo::action::Nav;
using std::placeholders::_1;
using std::placeholders::_2;

// 3.定义节点类；
class Exer04ActionServer : public rclcpp::Node{
public:
    Exer04ActionServer():Node("exer04_action_server_node_cpp"), x(0.0), y(0.0){
        RCLCPP_INFO(this->get_logger(), "动作服务端！");
        // 3-1.创建原生乌龟位姿订阅方，获取当前乌龟坐标
        sub_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, std::bind(&Exer04ActionServer::pose_cb, this, _1));
        // 3-2.创建速度指令发布方，控制乌龟运动
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        // 3-3.创建一个动作服务端
        /*  
            rclcpp_action::Server<ActionT>::SharedPtr 
            create_server<ActionT, NodeT>
            (NodeT node, const std::string &name, 
            rclcpp_action::Server<ActionT>::GoalCallback handle_goal, 
            rclcpp_action::Server<ActionT>::CancelCallback handle_cancel, 
            rclcpp_action::Server<ActionT>::AcceptedCallback handle_accepted, ...)
        */
        action_server_ = rclcpp_action::create_server<Nav>(
            this, 
            "nav",
            std::bind(&Exer04ActionServer::handle_goal, this, _1, _2),
            std::bind(&Exer04ActionServer::handle_cancel, this, _1),
            std::bind(&Exer04ActionServer::handle_accepted, this, _1)
            );
    }
private:
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp_action::Server<Nav>::SharedPtr action_server_;
    float x, y, theta;
    void pose_cb(const turtlesim::msg::Pose & pose){
        x = pose.x;
        y = pose.y;
        theta = pose.theta;
    }
    // 3-4.解析动作客户端提交的数据
    // GoalResponse(const GoalUUID &, std::shared_ptr<const typename ActionT::Goal>)
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Nav::Goal> goal){
        (void)uuid;
        // 取出目标中的 x y 坐标，分别判断是否超出了 [0, 11.08]，如果超出，则认为非法，否则合法
        if (goal->goal_x < 0 || goal->goal_x > 11.08 || goal->goal_y < 0 || goal->goal_y > 11.08)
        {
            RCLCPP_INFO(this->get_logger(), "目标点超出正常取值范围！");
            return rclcpp_action::GoalResponse::REJECT;
        }
        RCLCPP_INFO(this->get_logger(), "目标点合法！");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    // 3-5.处理客户端的取消请求操作
    // CancelResponse(std::shared_ptr<ServerGoalHandle<ActionT>>)
    rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<rclcpp_action::ServerGoalHandle<Nav>> goal_handle){
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "取消任务！");
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    // 主逻辑处理
    void execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<Nav>> goal_handle){
        // 3-7.子线程处理主逻辑
        RCLCPP_INFO(this->get_logger(), "主逻辑开始执行！");
        // 最终结果
        auto result = std::make_shared<Nav::Result>();
        auto feedback = std::make_shared<Nav::Feedback>();
        geometry_msgs::msg::Twist twist;
        // 1.产生连续反馈
        rclcpp::Rate rate(1.0);
        while (true)
        {
            // 如果要取消任务，需要特殊处理
            if (goal_handle->is_canceling())
            {
                // 设置取消后的最终结果
                goal_handle->canceled(result);
                return;
            }
            // 解析目标点坐标与原生乌龟实时坐标
            float goal_x = goal_handle->get_goal()->goal_x;
            float goal_y = goal_handle->get_goal()->goal_y;
            // 计算剩余距离并发布
            float distance_x = goal_x - x;
            float distance_y = goal_y - y;
            float distance = std::sqrt(distance_x * distance_x + distance_y * distance_y);
            double distance_theta = atan2(distance_y, distance_x) - theta;
            feedback->distance = distance;
            goal_handle->publish_feedback(feedback);
            // 2.发布速度指令
            // 根据剩余距离计算速度指令并发布
            float scale = 0.5;
            float linear_x = scale * distance;
            float linear_y = 0;
            twist.linear.x = linear_x;
            twist.linear.y = linear_y;
            twist.angular.z = 2 * distance_theta;
            RCLCPP_INFO(this->get_logger(), "atan2(distance_y, distance_x):%.2f, theta:%.2f", 
                atan2(distance_y, distance_x), theta);
            cmd_pub_->publish(twist);
            // 循环结束条件
            if (distance <= 0.1)
            {
                // 与目标点的剩余距离小于0.1m，就结束导航
                RCLCPP_INFO(this->get_logger(), "乌龟已经导航至目标点！");
                break;
            }
            
            rate.sleep();
        }
        // 3.响应最终结果
        if (rclcpp::ok())
        {
            result->turtle_x = x;
            result->turtle_y = y;
            goal_handle->succeed(result);
        }
    }
    // void (std::shared_ptr<ServerGoalHandle<ActionT>>)
    void handle_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<Nav>> goal_handle){
        // 3-6.实现主逻辑（耗时操作），启动子线程
        std::thread(std::bind(&Exer04ActionServer::execute, this, goal_handle)).detach();
    }
};

int main(int argc, char const *argv[])
{
    // 2.初始化 ROS2 客户端；
    rclcpp::init(argc, argv);
    // 4.调用spin函数，并传入自定义类对象指针；
    rclcpp::spin(std::make_shared<Exer04ActionServer>());
    // 5.释放资源。
    rclcpp::shutdown();
    return 0;
}
