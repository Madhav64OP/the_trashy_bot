import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch import LaunchDescription

def generate_launch_description():
    desc_pkg_share = get_package_share_directory("trashy_description")
    control_pkg_share = get_package_share_directory("trashy_control")
    
    urdf_file = os.path.join(desc_pkg_share, "urdf", "trashy.urdf.xacro")
    
    robot_desc = xacro.process_file(
        urdf_file,
        mappings={'is_ignition': 'false'}
    ).toxml()
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': False
        }]
    )
    
    controller_params_file = os.path.join(control_pkg_share, 'config', 'trashy_controllers.yaml')
    
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {'robot_description': robot_desc},
            controller_params_file
        ],
        output="screen"
    )
    
    # Isko baad mein dekhte hai 
    # delayed_controller_manager = TimerAction(period=0.3, actions=[controller_manager])

    velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller"],
        output="screen"
    )
    
    delayed_velocity_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[velocity_controller],
        )
    )
    
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen"
    )
    
    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_broadcaster],
        )
    )
    
    teleop = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        output="screen",
        prefix="gnome-terminal --",
        remappings=[
            ('/cmd_vel', '/velocity_controller/cmd_vel_unstamped')
        ]
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        delayed_velocity_spawner,
        delayed_joint_broad_spawner,
        teleop
    ])