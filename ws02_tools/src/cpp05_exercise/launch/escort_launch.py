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

""" 
    需求：实现乌龟护航案例
        先实现一只乌龟（back）护航，要求back的最终位置处于turtle1的正后方2米处
    思路：
        1.发布目标点相对于turtle1的坐标变换：静态广播器即可实现
        2.监听back与目标点坐标系的相对位置关系：exer03_tf_listener可以实现
    流程：
        1.抽取参数：DeclareLaunchArgument(name="t2_name", default_value="t2")
        2.创建turtlesim_node节点，并生成新乌龟
        3.发布坐标变换
        4.监听坐标变换
"""
def generate_launch_description():
    # 1.抽取参数：DeclareLaunchArgument(name="t2_name", default_value="t2")
    escort_back = DeclareLaunchArgument(name="turtle_back", default_value="turtle_back")
    # 2.创建turtlesim_node节点，并生成新乌龟
    master = Node(package="turtlesim", executable="turtlesim_node")
    spwan_back = Node(package="cpp05_exercise", executable="exer01_spawn",
                 name="spawn_back", 
                 parameters=[{"x":2.0, "y":5.0, "turtle_name":LaunchConfiguration("turtle_back")}])
    # 3.发布坐标变换
    turtle1_world = Node(package="cpp05_exercise", executable="exer02_tf_broadcaster", name="turtle1_world")
    back_world = Node(package="cpp05_exercise", executable="exer02_tf_broadcaster", name="back_world",
                      parameters=[{"turtle":LaunchConfiguration("turtle_back")}])
    escort_goal_back = Node(package="tf2_ros", executable="static_transform_publisher", name="escort_goal_back",
                            arguments=["--frame-id", "turtle1", "--child-frame-id", "escort_goal_back", "--x", "-1.5"])
    # 4.监听坐标变换
    back_escort_goal_back = Node(package="cpp05_exercise", executable="exer03_tf_listener", 
                                 name="back_escort_goal_back",
                                 parameters=[{"father_frame":LaunchConfiguration("turtle_back"), "child_frame":"escort_goal_back"}])
    return LaunchDescription([escort_back, master, spwan_back, turtle1_world, back_world, escort_goal_back, back_escort_goal_back])
