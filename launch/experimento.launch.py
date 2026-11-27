"""Experimento de odometría: una trayectoria, seis odometrías a la vez.

  ros2 launch roomba_odom experimento.launch.py trayectoria:=recta
  ros2 launch roomba_odom experimento.launch.py trayectoria:=circulo
  ros2 launch roomba_odom experimento.launch.py trayectoria:=ocho

Lanza la simulación (sin GUI por defecto) y seis odometry_node que leen los
mismos /joint_states: {euler, rk2, exact} x {50 Hz, 5 Hz}, en /odom/<método>_<Hz>.
Al terminar, el nodo experimento añade los errores finales al CSV 'salida'
y se cierra todo.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

METODOS = ('euler', 'rk2', 'exact')
FRECUENCIAS = (50, 5)


def generate_launch_description():
    share = get_package_share_directory('roomba_odom')
    lc = LaunchConfiguration
    args = {
        'trayectoria': 'circulo',
        'gui': 'false',
        'salida': '~/odom_resultados.csv',
        'v': '0.25',
        'radio': '1.0',
        'wheel_radius': '0.036',
        'wheel_separation': '0.235',
    }
    decl = [DeclareLaunchArgument(k, default_value=v) for k, v in args.items()]
    sim_time = {'use_sim_time': True}
    dims = {
        'wheel_radius': ParameterValue(lc('wheel_radius'), value_type=float),
        'wheel_separation': ParameterValue(lc('wheel_separation'), value_type=float),
    }

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'sim.launch.py')),
        launch_arguments={'gui': lc('gui'), 'odometria': 'false', 'guardian': 'false',
                          'wheel_radius': lc('wheel_radius'),
                          'wheel_separation': lc('wheel_separation')}.items())

    odoms, topics = [], []
    for m in METODOS:
        for hz in FRECUENCIAS:
            topic = f'/odom/{m}_{hz}'
            topics.append(topic)
            odoms.append(Node(
                package='roomba_odom', executable='odometry_node', name=f'odom_{m}_{hz}',
                output='screen',
                parameters=[dims, sim_time, {
                    'integration': m, 'rate': float(hz), 'odom_topic': topic,
                    # solo una publica la TF, para poder ver el robot en RViz
                    'publish_tf': (m == 'exact' and hz == 50)}]))

    exp = Node(package='roomba_odom', executable='experimento', output='screen',
               parameters=[sim_time, {
                   'trayectoria': lc('trayectoria'),
                   'odom_topics': topics,
                   'salida': lc('salida'),
                   'v': ParameterValue(lc('v'), value_type=float),
                   'radio': ParameterValue(lc('radio'), value_type=float)}])
    fin = RegisterEventHandler(OnProcessExit(target_action=exp,
                                             on_exit=[EmitEvent(event=Shutdown())]))

    return LaunchDescription(decl + [sim] + odoms + [exp, fin])
