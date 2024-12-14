import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import TimerAction
import launch_ros
import os

import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart


from launch_ros.actions import Node
def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='vacbot').find('vacbot')
    default_model_path = os.path.join(pkg_share, 'urdf/vacbot.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'config/urdf_config.rviz')
    # world_path=os.path.join(pkg_share, 'world/my_world.sdf')
    
    robot_description = {'robot_description': Command(['xacro ', LaunchConfiguration('model')])}

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    controller_params_file = os.path.join(pkg_share,'config','ros2_controllers.yaml')

    controller_manager = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_params_file]
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])


    joint_broad_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    diff_drive_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

#    rviz_node = launch_ros.actions.Node(
#        package='rviz2',
#        executable='rviz2',
#        name='rviz2',
#        output='screen',
#        arguments=['-d', LaunchConfiguration('rvizconfig')],
#    )

    # spawn_entity = launch_ros.actions.Node(
    # 	package='gazebo_ros', 
    # 	executable='spawn_entity.py',
    #     arguments=['-entity', 'sam_bot', '-topic', 'robot_description'],
    #     output='screen'
    # )
    # robot_localization_node = launch_ros.actions.Node(
    #      package='robot_localization',
    #      executable='ekf_node',
    #      name='ekf_filter_node',
    #      output='screen',
    #      parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    # )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        # launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
        delayed_joint_broad_spawner,
        robot_state_publisher_node,
        diff_drive_spawner,
        delayed_controller_manager,
        # spawn_entity,
        # robot_localization_node,
        # rviz_node
    ])
