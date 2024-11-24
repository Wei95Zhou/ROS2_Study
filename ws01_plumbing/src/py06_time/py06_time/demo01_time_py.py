# 1.导包
import rclpy
from rclpy.node import Node
import threading
from rclpy.time import Time
from rclpy.duration import Duration

# 3.自定义节点类
class MyNode(Node):
    def __init__(self):
        super().__init__("time_node_py")
        # self.demo_rate()
        # self.demo_time()
        # self.demo_duration()
        self.demo_opt()

    def demo_opt(self):
        t1 = Time(seconds=20)
        t2 = Time(seconds=15)
        d1 = Duration(seconds=7)
        d2 = Duration(seconds=13)
        self.get_logger().info("t1 >= t2? %d" % (t1 >= t2))
        t3 = t1 - t2
        t4 = t1 + d1
        t5 = t1 - d1
        self.get_logger().info("t3 = %d" % t3.nanoseconds)
        self.get_logger().info("t4 = %d" % t4.nanoseconds)
        self.get_logger().info("t5 = %d" % t5.nanoseconds)
        self.get_logger().info("d1 >= d2? %d" % (d1 >= d2))

    def demo_duration(self):
        # 1.创建 Duration 对象
        d1 = Duration(seconds=10, nanoseconds=800000000)
        # 2.调用函数
        self.get_logger().info("ns = %d" % d1.nanoseconds)

    def demo_time(self):
        # 1.创建 Time 对象
        t1 = Time(seconds=5, nanoseconds=500000000)
        right_now = self.get_clock().now()
        # 2.调用 time 函数
        self.get_logger().info("s = %.2f, ns = %d" % (t1.seconds_nanoseconds()[0], t1.seconds_nanoseconds()[1]))
        self.get_logger().info("s = %.2f, ns = %d" % (right_now.seconds_nanoseconds()[0], right_now.seconds_nanoseconds()[1]))
        self.get_logger().info("ns = %d" % t1.nanoseconds)
        self.get_logger().info("ns = %d" % right_now.nanoseconds)

    def demo_rate(self):
        # 1.创建 Rate 对象
        self.rate = self.create_rate(1.0)
        # 2.调用 sleep 函数 ---- 导致程序阻塞
        # 解决方案1：使用 time 休眠
        # while rclpy.ok():
        #     self.get_logger().info("+++++++++++++++")
        #     self.rate.sleep()

        # 解决方案2：创建子线程实现运行频率控制
        thread = threading.Thread(target=self.do_some)
        thread.start()

    def do_some(self):
        while rclpy.ok():
            self.get_logger().info("+++++++++++++++")
            self.rate.sleep()

def main():
    # 2.初始化ros2客户端
    rclpy.init()
    # 4.调用spin函数，并传入节点对象
    rclpy.spin(MyNode())
    # 5.资源释放
    rclpy.shutdown()

if __name__ == '__main__':
    main()
