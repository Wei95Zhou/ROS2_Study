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
from ament_index_python.packages import get_package_share_directory

from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

def generate_launch_description():
    """ 
        需求：加载urdf文件，并在rviz2中显示机器人模型
        核心：
            1.启动robot_state_publisher节点，该节点要以参数的方式加载urdf文件内容
            2.启动rviz2节点
        优化：
            1.添加 joint_state_publisher 节点（当机器人有非固定关节时，必须包含该节点）
            2.设置 rviz2 的默认配置文件
            3.动态传入urdf文件，把urdf文件封装为参数
    """
    # 1.启动robot_state_publisher节点，该节点要以参数的方式加载urdf文件内容
    # 调用格式：ros2 launch cpp06_urdf display.launch.py model:=`ros2 pkg prefix --share cpp06_urdf`/urdf/urdf/xxxx.urdf
    # `ros2 pkg prefix --share cpp06_urdf`是一个指令，效果等价于get_package_share_directory("cpp06_urdf")
    model = DeclareLaunchArgument(name="model", default_value=get_package_share_directory("cpp06_urdf") + "/urdf/urdf/demo01_helloworld.urdf")
    # p_value = ParameterValue(Command(["xacro ", get_package_share_directory("cpp06_urdf") + "/urdf/urdf/demo01_helloworld.urdf"]))
    p_value = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]))
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description":p_value}])
    # 优化1，添加 joint_state_publisher 节点
    joint_state_pub = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher"
    )
    # 2.启动rviz2节点
    rviz2 = Node(
        package="rviz2", 
        executable="rviz2",
        arguments=["-d", get_package_share_directory("cpp06_urdf") + "/rviz/urdf.rviz"]
    )
    return LaunchDescription([model, robot_state_pub, joint_state_pub, rviz2])

""" 
    问题描述：通过 joint_state_publisher_gui 让关节运行到指定位置后，关节存在“抖动”，
            在初始位置和指定位置之间抖动
    解决：不再启动joint_state_publisher节点
    原因：
        1.joint_state_publisher 和 joint_state_publisher_gui 作用一致，都会发布非固定关节的运动信息
        2.robot_state_pub会订阅关节的运动信息，并生成坐标变换数据广播
        3.joint_state_publisher 或 joint_state_publisher_gui 有一个存在时，就会发布关节运动信息，进而就能生成坐标变换
          当两个都不启动时，坐标树就不能生成，rviz中机器人模型显示异常
          当两个都存在时，joint_state_publisher 一直发布初始关节位姿信息，而joint_state_publisher_gui 发布指定的关节位姿信息
          最终，两个消息都要订阅，产生抖动的效果。
"""
