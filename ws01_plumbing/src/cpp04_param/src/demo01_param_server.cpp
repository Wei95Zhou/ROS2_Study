/*
    需求：创建参数服务端并操作数据
    步骤：
        1.包含头文件；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.增
            3-2.查
            3-3.改
            3-4.删
        4.调用spin函数，并传入自定义类对象指针；
        5.释放资源。
*/

#include "rclcpp/rclcpp.hpp"

// 3.定义节点类；
class ParamServer : public rclcpp::Node{
public:
    // 如果允许参数删除，那么需要通过NodeOptions声明
    // Node(const std::string &node_name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
    ParamServer():Node("param_server_node_cpp", rclcpp::NodeOptions().allow_undeclared_parameters(true)){
        RCLCPP_INFO(this->get_logger(), "参数服务端创建了！");
    }
    // 3-1.增
    void declare_param(){
        RCLCPP_INFO(this->get_logger(), "----------------增----------------");
        this->declare_parameter("car_name", "tiger");
        this->declare_parameter("width", 1.55);
        this->declare_parameter("wheels", 5);
        // 也可以用于设置新参数，但必须保证：rclcpp::NodeOptions().allow_undeclared_parameters(true) 被调用
        this->set_parameter(rclcpp::Parameter("height", 2.00));
    }
    // 3-2.查
    void get_param(){
        RCLCPP_INFO(this->get_logger(), "----------------查----------------");
        // 获取指定参数
        auto car = this->get_parameter("car_name");
        RCLCPP_INFO(this->get_logger(), "key = %s, value = %s", car.get_name().c_str(), car.as_string().c_str());
        // 获取一些参数
        auto params = this->get_parameters({"car_name", "width", "wheels"});
        for (auto &&param : params)
        {
            RCLCPP_INFO(this->get_logger(), "(%s = %s)", param.get_name().c_str(), param.value_to_string().c_str());
        }
        // 判断是否包含
        RCLCPP_INFO(this->get_logger(), "是否包含car_name? %d", this->has_parameter("car_name"));
        RCLCPP_INFO(this->get_logger(), "是否包含height? %d", this->has_parameter("height"));
    }
    // 3-3.改
    void update_param(){
        RCLCPP_INFO(this->get_logger(), "----------------改----------------");
        this->set_parameter(rclcpp::Parameter("width", 1.75));
        RCLCPP_INFO(this->get_logger(), "width = %.2f", this->get_parameter("width").as_double());
    }
    // 3-4.删
    void delete_param(){
        RCLCPP_INFO(this->get_logger(), "----------------删----------------");
        // this->undeclare_parameter("car_name"); 不能删除声明的参数 
        // 可以删除未声明然后设置的参数 
        this->undeclare_parameter("height");

    }
};

int main(int argc, char const *argv[])
{
    // 2.初始化 ROS2 客户端；
    rclcpp::init(argc, argv);
    // 4.调用spin函数，并传入自定义类对象指针；
    auto node = std::make_shared<ParamServer>();
    node->declare_param();
    node->get_param();
    node->update_param();
    node->delete_param();
    rclcpp::spin(node);
    // 5.释放资源。
    rclcpp::shutdown();
    return 0;
}
