"""
需求：启动 turtlesim_node 节点，编写程序，发布乌龟(turtle1)相对于窗体(world)的位姿
流程：
    1.导包
    2.初始化ros2客户端
    3.自定义节点类
        3-1.创建一个动态广播器
        3-2.创建一个乌龟位姿订阅方
        3-3.回调函数中，获取乌龟位姿，并生成相对关系，然后发布
    4.调用spin函数，并传入节点对象
    5.资源释放 
"""

# 1.导包
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from turtlesim.msg import Pose
from geometry_msgs.msg import TransformStamped
import tf_transformations

# 3.自定义节点类
class TFDynamicBroadcasterPy(Node):
    def __init__(self):
        super().__init__("tf_dynamic_broadcaster_py_node_py")
        # 3-1.创建一个动态广播器
        self.broadcaster = TransformBroadcaster(self)
        # 3-2.创建一个乌龟位姿订阅方
        self.sub = self.create_subscription(
            Pose, 
            "/turtle1/pose", 
            self.do_pose, 
            10)
    # 3-3.回调函数中，获取乌龟位姿，并生成相对关系，然后发布
    def do_pose(self, pose):
        # 组成 transform
        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = "world"
        ts.child_frame_id = "turtle1"
        # 偏移量
        ts.transform.translation.x = pose.x
        ts.transform.translation.y = pose.y
        ts.transform.translation.z = 0.0
        # 四元数
        qtn = tf_transformations.quaternion_from_euler(0.0, 0.0, pose.theta)
        ts.transform.rotation.x = qtn[0]
        ts.transform.rotation.y = qtn[1]
        ts.transform.rotation.z = qtn[2]
        ts.transform.rotation.w = qtn[3]
        # 发布 transform
        self.broadcaster.sendTransform(ts)

def main():
    # 2.初始化ros2客户端
    rclpy.init()
    # 4.调用spin函数，并传入节点对象
    rclpy.spin(TFDynamicBroadcasterPy())
    # 5.资源释放
    rclpy.shutdown()

if __name__ == '__main__':
    main()
