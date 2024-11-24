"""
需求：创建客户端，组织数据并提交，然后处理响应结果
流程：
    前提：判断main函数提交的参数是否正确
    1.导包
    2.初始化ros2客户端
    3.自定义节点类
        3-1.创建客户端
        3-2.连接服务器，如果连接不到服务器，则不能发送请求
        3-3.发送请求
    4.调用spin函数，并传入节点对象
        调用连接服务的函数，根据连接结果做下一步处理
        连接服务后，调用请求发送函数
        再处理响应结果
    5.资源释放 
"""

# 1.导包
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
import sys
from base_interfaces_demo.srv import AddInts

# 3.自定义节点类
class AddIntsClient(Node):
    def __init__(self):
        super().__init__("add_ints_client_node_py")
        self.get_logger().info("客户端创建！(python)")
        # 3-1.创建客户端
        self.client = self.create_client(AddInts, "add_ints")
        # 3-2.连接服务器
        while not self.client.wait_for_service(1.0):
            self.get_logger().info("服务连接中！")
        
    # 3-3.发送请求
    def send_request(self):
        request = AddInts.Request()
        request.num1 = int(sys.argv[1])
        request.num2 = int(sys.argv[2])
        self.future = self.client.call_async(request)

def main():
    # 校验操作
    if len(sys.argv) != 3:
        get_logger("rclpy").error("请提交两个整型数字！")
        return
    # 2.初始化ros2客户端
    rclpy.init()
    client = AddIntsClient()
    # 发送请求
    client.send_request()
    # 处理响应
    rclpy.spin_until_future_complete(client, client.future)
    try:
        response = client.future.result()
        client.get_logger().info("响应结果：sum = %d" % response.sum)
    except Exception:
        client.get_logger().error("服务响应失败！")

    # 5.资源释放
    rclpy.shutdown()

if __name__ == '__main__':
    main()
