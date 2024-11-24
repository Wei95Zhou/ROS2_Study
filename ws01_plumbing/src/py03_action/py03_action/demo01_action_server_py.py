"""
需求：编写动作服务端，需要解析客户端提交的数字，遍历该数字并累加求和，最终结果需要响应回客户端，
    且请求响应过程中需要生成连续反馈
流程：
    1.导包
    2.初始化ros2客户端
    3.自定义节点类
        3-1.创建动作服务端对象
        3-2.处理提交的目标值（回调函数）--- 有默认实现
        3-3.处理取消请求（回调函数） --- 有默认实现
        3-4.生成连续反馈与最终响应（回调函数）
    4.调用spin函数，并传入节点对象
    5.资源释放 
"""

# 1.导包
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from base_interfaces_demo.action import Progress
import time

# 3.自定义节点类
class ProgressActionServer(Node):
    def __init__(self):
        super().__init__("progress_action_server_node_py")
        self.get_logger().info("动作通信服务端创建！")
        # 3-1.创建动作服务端对象
        """ 
        node: Any,
        action_type: Any,
        action_name: Any,
        execute_callback: Any,
        """
        self.server = ActionServer(
            self,
            Progress,
            "get_sum",
            self.execute_callback,
            # self.accept_callback,
            # self.cancle_callback
        )

    # 3-4.生成连续反馈与最终响应（回调函数）
    def execute_callback(self, goal_handle):
        # 1.生成连续反馈
        # 首先获取目标值，然后遍历，遍历中进行累加，且每循环一次就计算进度，并作为连续反馈发布
        num = goal_handle.request.num
        sum = 0
        for i in range(1, num + 1):
            sum += i
            feedback = Progress.Feedback()
            feedback.progress = i / num
            goal_handle.publish_feedback(feedback)
            self.get_logger().info("连续反馈：%.2f" % feedback.progress)
            # if (goal_handle.cancled()):
            #     self.get_logger().info("目标取消！")
            time.sleep(0.5)
        # 2.响应最终结果
        goal_handle.succeed()
        result = Progress.Result()
        result.sum = sum
        self.get_logger().info("计算结果：%d" % result.sum)
        return result
    
    def accept_callback(self, goal_handle):
        if goal_handle.request.num > 0:
            self.get_logger().info("收到目标值：%d" % goal_handle.request.num)
            return goal_handle.accept()
        self.get_logger().error("目标值不合法！")
        return goal_handle.reject()
    
    def cancle_callback(self, goal_handle):
        self.get_logger().info("目标取消中")
        goal_handle.cancled()
        return goal_handle.cancled()

def main():
    # 2.初始化ros2客户端
    rclpy.init()
    # 4.调用spin函数，并传入节点对象
    rclpy.spin(ProgressActionServer())
    # 5.资源释放
    rclpy.shutdown()

if __name__ == '__main__':
    main()
