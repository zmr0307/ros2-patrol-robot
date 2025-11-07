import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # 1. 找到我们的URDF/XACRO文件
    pkg_path = get_package_share_directory('robot_bringup_cpp')
    xacro_file = os.path.join(pkg_path, 'urdf', 'my_robot.urdf.xacro')

    # 2. 将xacro文件解析成标准的URDF字符串
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # 3. 定义【唯一】的 robot_state_publisher 节点
    #    我们将URDF作为参数直接传递给它
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description] # 【关键】把URDF作为参数喂给它
    )

    # 4. 定义 joint_state_publisher_gui 节点
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    # 5. 定义 Rviz2 节点

    node_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
    )

    # 6. 返回启动清单
    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz2
    ])