from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # 1.启动两个 turtlesim_node，其中一个要设置命名空间
    t1 = Node(package="turtlesim", executable="turtlesim_node")
    t2 = Node(package="turtlesim", executable="turtlesim_node", namespace="t2")

    # 2.控制第二个乌龟调头
    """  
    cmd=[""], 要执行的命令
    output="both", 日志即输出到日志文件，也输出到终端
    shell=True, cmd中的命令要当成终端指令来执行
    """
    rotate = ExecuteProcess(
        cmd=["ros2 action send_goal /t2/turtle1/rotate_absolute turtlesim/action/RotateAbsolute \"{'theta': 3.14}\""],
        output="both", 
        shell=True
    )

    # 3.调用自定义的节点，该节点调用顺序有要求，要在调头完毕才能执行
    exer01 = Node(package="cpp07_exercise", executable="exer01_pub_sub")
    # 怎么控制节点的执行顺序？需要注册事件完成。
    # 创建事件注册对象，在对象重声明针对哪个目标节点，在哪个事件触发时，执行哪种操作
    register_rotate_exit_event = RegisterEventHandler(
        # 创建一个新对象
        event_handler = OnProcessExit( # 触发的动作
            target_action=rotate, # 目标节点
            on_exit=exer01) # 触发后执行的事件
    )

    return LaunchDescription([t1, t2, rotate, register_rotate_exit_event])
