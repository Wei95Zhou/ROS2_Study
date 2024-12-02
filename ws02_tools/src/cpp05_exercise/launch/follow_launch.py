from launch import LaunchDescription
from launch_ros.actions import Node
# 封装终端指令相关类---------------
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable
# 参数声明与获取------------------
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
# 文件包含相关--------------------
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# 分组相关------------------------
# from launch_ros.actions import PushRosNamespace
# from launch.actions import GroupAction
# 事件相关------------------------
# from launch.event_handlers import OnProcessStart, OnProcessExit
# from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo
# 获取功能包下share目录路径--------
# from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 优化坐标系的设置
    # 优化思想：频繁使用的数据，设置为变量，调用时直接调用变量
    t2 = DeclareLaunchArgument(name="t2_name", default_value="t2")
    # 1.启动 turtlesim_node节点
    turtle = Node(package="turtlesim", executable="turtlesim_node")
    # 2.启动自定义的spwan节点
    spawn = Node(package="cpp05_exercise", executable="exer01_spawn", 
                 parameters=[{"turtle_name":LaunchConfiguration(variable_name="t2_name")}])
    
    # 3.分别广播两只乌龟相当于world的坐标变换
    broadcastr1 = Node(package="cpp05_exercise", executable="exer02_tf_broadcaster", name="broa1")
    broadcastr2 = Node(package="cpp05_exercise", executable="exer02_tf_broadcaster", name="broa2", 
                       parameters=[{"turtle": LaunchConfiguration(variable_name="t2_name")}])
    
    # 4.创建监听节点
    listener = Node(package="cpp05_exercise", executable="exer03_tf_listener", 
                    parameters=[{"father_frame": LaunchConfiguration(variable_name="t2_name"), "child_frame": "turtle1"}])
    return LaunchDescription([t2, turtle, spawn, broadcastr1, broadcastr2, listener])
