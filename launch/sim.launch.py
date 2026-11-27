"""Roomba en Gazebo Harmonic con ros2_control (JointGroupVelocityController),
cinemática inversa propia y odometría propia.

  ros2 launch roomba_odom sim.launch.py                       # odometría exacta a 50 Hz
  ros2 launch roomba_odom sim.launch.py integration:=euler odom_rate:=5.0
  ros2 launch roomba_odom sim.launch.py guardian:=true        # permisos + emergencia + voz
  ros2 launch roomba_odom sim.launch.py gui:=false            # Gazebo sin ventana

Cadena:
  /cmd_vel -> kinematics_node -> /wheel_velocity_controller/commands -> Gazebo
  Gazebo -> /joint_states -> odometry_node -> /odom + TF odom->base_link
  Gazebo (OdometryPublisher) -> ros_gz_bridge -> /ground_truth

Con guardian:=true, las órdenes manuales van a /cmd_vel_in y el guardián
decide qué llega a /cmd_vel.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('roomba_odom')
    lc = LaunchConfiguration
    args = {
        'gui': 'true',
        'world': os.path.join(share, 'worlds', 'vacio.sdf'),
        'wheel_radius': '0.036',
        'wheel_separation': '0.235',
        'integration': 'exact',
        'odom_rate': '50.0',
        'odometria': 'true',
        'guardian': 'false',
        'rviz': 'false',
    }
    decl = [DeclareLaunchArgument(k, default_value=v) for k, v in args.items()]
    sim_time = {'use_sim_time': True}
    dims = {
        'wheel_radius': ParameterValue(lc('wheel_radius'), value_type=float),
        'wheel_separation': ParameterValue(lc('wheel_separation'), value_type=float),
    }

    robot_description = ParameterValue(Command([
        'xacro ', os.path.join(share, 'urdf', 'roomba.urdf.xacro'),
        ' wheel_radius:=', lc('wheel_radius'),
        ' wheel_separation:=', lc('wheel_separation'),
        ' use_gazebo:=true']), value_type=str)

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': PythonExpression(
                ["'-r -v 2 ' + ('' if '", lc('gui'), "' == 'true' else '-s ') + '", lc('world'), "'"]),
            'on_exit_shutdown': 'true',
        }.items())

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': robot_description}, sim_time], output='screen')

    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-topic', 'robot_description', '-name', 'roomba', '-z', '0.04'])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/model/roomba/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
        remappings=[('/model/roomba/odometry', '/ground_truth')],
        parameters=[sim_time])

    spawners = Node(package='controller_manager', executable='spawner', output='screen',
                    arguments=['joint_state_broadcaster', 'wheel_velocity_controller',
                               '--controller-manager', '/controller_manager'],
                    parameters=[sim_time])
    tras_spawn = RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[spawners]))

    kinematics = Node(package='roomba_odom', executable='kinematics_node', output='screen',
                      parameters=[dims, sim_time])

    odometry = Node(package='roomba_odom', executable='odometry_node', output='screen',
                    condition=IfCondition(lc('odometria')),
                    parameters=[dims, sim_time, {
                        'integration': lc('integration'),
                        'rate': ParameterValue(lc('odom_rate'), value_type=float)}])

    con_guardian = IfCondition(lc('guardian'))
    gestor = Node(package='roomba_odom', executable='gestor', output='screen',
                  condition=con_guardian, parameters=[sim_time])
    guardian = Node(package='roomba_odom', executable='guardian', output='screen',
                    condition=con_guardian, parameters=[sim_time])
    voz = Node(package='roomba_odom', executable='voz_node', output='screen',
               condition=con_guardian, parameters=[sim_time])

    # El robot nace en el origen: odom y el marco de la verdad coinciden en t = 0.
    # Así RViz puede dibujar /ground_truth (frame gt_odom) junto a /odom.
    gt_tf = Node(package='tf2_ros', executable='static_transform_publisher',
                 arguments=['--frame-id', 'odom', '--child-frame-id', 'gt_odom'],
                 parameters=[sim_time])

    rviz = Node(package='rviz2', executable='rviz2', output='log',
                arguments=['-d', os.path.join(share, 'rviz', 'roomba.rviz')],
                condition=IfCondition(lc('rviz')), parameters=[sim_time])

    return LaunchDescription(decl + [gz, rsp, spawn, bridge, tras_spawn, kinematics, odometry,
                                     gestor, guardian, voz, gt_tf, rviz])
