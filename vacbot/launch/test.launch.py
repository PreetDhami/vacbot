# Copyright 2020 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from os.path import join

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare(package='vacbot').find('vacbot')

    # Get URDF via xacro
    robot_description = {'robot_description': Command([PathJoinSubstitution([FindExecutable(name="xacro")]), " ", pkg_share, '/urdf/vacbot.urdf.xacro'])}
    robot_controllers = PathJoinSubstitution([pkg_share,'config','ros2_controllers.yaml'])
    rviz_config_file = PathJoinSubstitution([pkg_share, 'config', 'vacbot.rviz'])
    world_path = PathJoinSubstitution([pkg_share, 'world', 'my_world.sdf'])

    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_path,
        description='World to load'
    )


    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
    )
    
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont", "--param-file", robot_controllers],
    )

    # Delay start of joint_state_broadcaster after `robot_controller`
    # TODO(anyone): This is a workaround for flaky tests. Remove when fixed.
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource([
    #     PathJoinSubstitution([FindPackageShare(package='ros_gz_sim'),'launch', 'gz_sim.launch.py'])]),
    #     launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items())


    # bridge_params = join(pkg_share,'config','gz_bridge.yaml')
    # ros_gz_bridge = Node(
    #     package="ros_gz_bridge",
    #     executable="parameter_bridge",
    #     arguments=[
    #         '--ros-args',
    #         '-p',
    #         f'config_file:={bridge_params}',
    #     ]
    # )


    # create = Node(
    # 	package='ros_gz_sim', 
    # 	executable='create',
    #     arguments=['-name', 'vacbot', '-topic', 'robot_description', '-z', '0.1'],
    #     output='screen'
    # )



    return LaunchDescription([
        control_node,
        robot_state_pub_node,
        robot_controller_spawner,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
        rviz_node,
        # world_arg,
        # gazebo,
        # ros_gz_bridge,
        # create
    ])