"""
需求：订阅发布方发布的消息，并在终端输出
流程：
    1.导包
    2.初始化ros2客户端
    3.自定义节点类
    4.调用spin函数，并传入节点对象
    5.资源释放 
"""

# 1.导包
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# 3.自定义节点类
class MyNode(Node):
    def __init__(self):
        super().__init__("zhenkeng_py", namespace="zuoxie_py")
        # 全局话题
        # self.pub = self.create_publisher(String, "/shi", 10)
        # 相对话题
        # self.pub = self.create_publisher(String, "kaihui", 10)
        # 私有话题
        self.pub = self.create_publisher(String, "~/vip", 10)

def main():
    # 2.初始化ros2客户端
    rclpy.init()
    # 4.调用spin函数，并传入节点对象
    rclpy.spin(MyNode())
    # 5.资源释放
    rclpy.shutdown()

if __name__ == '__main__':
    main()
