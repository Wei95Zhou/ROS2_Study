"""
需求：创建参数服务端并操作数据
流程：
    1.导包
    2.初始化ros2客户端
    3.自定义节点类
        3-1.增
        3-2.查
        3-3.改
        3-4.删
    4.调用spin函数，并传入节点对象
    5.资源释放 
"""

# 1.导包
import rclpy
from rclpy.node import Node

# 3.自定义节点类
class ParamServer(Node):
    def __init__(self):
        # 如果允许删除参数，需要提前声明
        super().__init__("param_server_node_py", allow_undeclared_parameters=True)
        self.get_logger().info("参数服务端！")

    # 3-1.增
    def declare_param(self):
        self.get_logger().info("---------------新增参数---------------")
        self.declare_parameter("car_name", "tiger")
        self.declare_parameter("width", 1.55)
        self.declare_parameter("wheels", 5)

        self.set_parameters([rclpy.Parameter("haha", value = "xixi")])
    # 3-2.查
    def get_param(self):
        self.get_logger().info("---------------查询参数---------------")
        # 获取单个参数
        car_name = self.get_parameter("car_name")
        self.get_logger().info("%s = %s" % (car_name.name, car_name.value))
        # 获取多个参数
        params = self.get_parameters(["car_name", "wheels", "width"])
        for param in params:
            self.get_logger().info("%s ====== %s" % (param.name, param.value))
        # 判断是否包含某个参数
        self.get_logger().info("包含 car_name 吗？%d" % self.has_parameter("car_name"))
        self.get_logger().info("包含 height 吗？%d" % self.has_parameter("height"))
    # 3-3.改
    def update_param(self):
        self.get_logger().info("---------------修改参数---------------")
        self.set_parameters([rclpy.Parameter("car_name", value="Mouse")])
        car_name = self.get_parameter("car_name")
        self.get_logger().info("修改后%s = %s" % (car_name.name, car_name.value))
    # 3-4.删
    def delete_param(self):
        self.get_logger().info("---------------删除参数---------------")
        self.get_logger().info("删除前，包含 car_name 吗？%d" % self.has_parameter("car_name"))
        self.undeclare_parameter("car_name")
        self.get_logger().info("删除后，包含 car_name 吗？%d" % self.has_parameter("car_name"))

def main():
    # 2.初始化ros2客户端
    rclpy.init()
    # 4.调用spin函数，并传入节点对象
    node = ParamServer()
    node.declare_param()
    node.get_param()
    node.update_param()
    node.delete_param()
    rclpy.spin(node)
    # 5.资源释放
    rclpy.shutdown()

if __name__ == '__main__':
    main()
