/*  
    问题：Time 与 Duration 有什么区别？
    答：
        1.二者只是API使用类似；
        2.二者有本质区别：
            rclcpp::Time t2(2,500000000L); --- 指的是一个具体时刻 --- 1970-01-01 00:00:02.500  UTC
            rclcpp::Duration d2(2, 500000000); --- 指的是一个时间段，持续2.5s
*/
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
// 3.定义节点类；
class MyNode : public rclcpp::Node{
public:
    MyNode():Node("time_node_cpp"){
        // demo_rate();
        // demo_time();
        // demo_duration();
        demo_opt();
    }
private:
    // 演示运算符的使用
    void demo_opt(){
        rclcpp::Time t1(10, 0);
        rclcpp::Time t2(30, 0);
        rclcpp::Duration d1(8, 0);
        rclcpp::Duration d2(17, 0);
        // 运算
        // 比较运算
        RCLCPP_INFO(this->get_logger(), "t1 >= t2 ? %d", t1 >= t2);
        // 数学运算
        rclcpp::Duration d3 = t2 - t1;
        rclcpp::Time t3 = t1 + d1;
        rclcpp::Time t4 = t1 - d1;
        RCLCPP_INFO(this->get_logger(), "d3 = %.2f", d3.seconds());
        RCLCPP_INFO(this->get_logger(), "t3 = %.2f", t3.seconds());
        RCLCPP_INFO(this->get_logger(), "t3 = %.2f", t4.seconds());

        RCLCPP_INFO(this->get_logger(), "d1 >= d2 ? %d", d1 >= d2);
        rclcpp::Duration d4 = d1 * 3;
        rclcpp::Duration d5 = d1 + d2;
        rclcpp::Duration d6 = d1 - d2;
        RCLCPP_INFO(this->get_logger(), "d4 = %.2f", d4.seconds());
        RCLCPP_INFO(this->get_logger(), "d5 = %.2f", d5.seconds());
        RCLCPP_INFO(this->get_logger(), "d6 = %.2f", d6.seconds());
    }
    // 演示 Duration 的使用
    void demo_duration(){
        // 1.创建 Duration 对象
        rclcpp::Duration d1(1s);
        rclcpp::Duration d2(2, 500000000);
        // 2.调用函数
        RCLCPP_INFO(this->get_logger(), "s = %.2f, ns = %ld", d1.seconds(), d1.nanoseconds());
        RCLCPP_INFO(this->get_logger(), "s = %.2f, ns = %ld", d2.seconds(), d2.nanoseconds());
    }
    // 演示 Time 使用
    void demo_time(){
        // 1.创建 Time 对象
        rclcpp::Time t1(500000000L);
        rclcpp::Time t2(2,500000000L);
        // rclcpp::Time right_now = this->get_clock()->now();
        rclcpp::Time right_now = this->now();

        // 2.调用 Time 对象的函数
        RCLCPP_INFO(this->get_logger(), "s = %.2f, ns = %ld", t1.seconds(), t1.nanoseconds());
        RCLCPP_INFO(this->get_logger(), "s = %.2f, ns = %ld", t2.seconds(), t2.nanoseconds());
        RCLCPP_INFO(this->get_logger(), "s = %.2f, ns = %ld", right_now.seconds(), right_now.nanoseconds());
    }
    // 演示 Rate 的使用
    void demo_rate(){
        // 1.创建 Rate 对象
        rclcpp::Rate rate1(500ms); //设置休眠时间（500ms）
        rclcpp::Rate rate2(1.0); //设置执行频率（1Hz）
        // 2.调用 Rate 的 sleep 函数
        while (rclcpp::ok())
        {
            RCLCPP_INFO(this->get_logger(), "---------------");
            // rate1.sleep();
            rate2.sleep();
        }
        
    }
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
