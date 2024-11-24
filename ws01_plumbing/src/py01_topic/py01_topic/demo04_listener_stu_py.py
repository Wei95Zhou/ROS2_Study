"""
需求：订阅学生信息并打印在终端
流程：
    1.导包
    2.初始化ros2客户端
    3.自定义节点类
        3-1.创建订阅方
        3-2.回调函数订阅并解析数据
    4.调用spin函数，并传入节点对象
    5.资源释放 
"""

# 1.导包
import rclpy
from rclpy.node import Node
from base_interfaces_demo.msg import Student

# 3.自定义节点类
class ListenerStu(Node):
    def __init__(self):
        super().__init__("listenerstu_node_py")
        # 3-1.创建订阅方
        self.subscription = self.create_subscription(Student, "chatter_stu", self.do_cb, 10)

    def do_cb(self, stu):
        # 3-2.回调函数订阅并解析数据
        self.get_logger().info("接收到的学生信息：(%s, %d, %.2f)" % (stu.name, stu.age, stu.height))

def main():
    # 2.初始化ros2客户端
    rclpy.init()
    # 4.调用spin函数，并传入节点对象
    rclpy.spin(ListenerStu())
    # 5.资源释放
    rclpy.shutdown()

if __name__ == '__main__':
    main()
