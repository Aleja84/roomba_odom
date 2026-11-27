"""Experimento: recorre una trayectoria en lazo abierto y mide el error final
de cada odometría frente a /ground_truth.

Suscribe:
  ground_truth          nav_msgs/Odometry (OdometryPublisher de Gazebo)
  <odom_topics>         nav_msgs/Odometry, una por método y frecuencia
Publica:
  cmd_vel               geometry_msgs/TwistStamped a 50 Hz

Cada pose se expresa respecto a la primera que se recibió al empezar, así que
da igual dónde arranque cada estimación. Al acabar escribe una fila por
odometría en el CSV 'salida' y termina.
"""
import csv
import math
import os

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from roomba_odom.integracion import wrap
from roomba_odom.trayectorias import consigna, duracion, perfil


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def pose2d(msg):
    p = msg.pose.pose
    return p.position.x, p.position.y, yaw(p.orientation)


def relativa(p0, p):
    """Pose p expresada en el marco de p0 (inv(p0) ∘ p)."""
    dx, dy = p[0] - p0[0], p[1] - p0[1]
    c, s = math.cos(p0[2]), math.sin(p0[2])
    return c * dx + s * dy, -s * dx + c * dy, wrap(p[2] - p0[2])


class Experimento(Node):
    def __init__(self):
        super().__init__('experimento')
        p = self.declare_parameter
        self.nombre = p('trayectoria', 'circulo').value
        self.tramos = perfil(self.nombre, p('v', 0.25).value, p('radio', 1.0).value)
        self.topics = list(p('odom_topics', ['odom']).value)
        self.salida = os.path.expanduser(p('salida', '~/odom_resultados.csv').value)
        self.espera = p('espera', 3.0).value           # s quieto antes y después

        self.ultima = {}                                # topic -> pose2d
        self.create_subscription(Odometry, 'ground_truth',
                                 lambda m: self.on_pose('ground_truth', m), 20)
        for t in self.topics:
            self.create_subscription(Odometry, t, lambda m, t=t: self.on_pose(t, m), 20)
        self.pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)

        self.estado = 'esperando'
        self.t_estado = None
        self.inicio = {}
        self.terminado = False
        self.create_timer(0.02, self.step)
        self.get_logger().info(
            f'Trayectoria {self.nombre} ({duracion(self.tramos):.1f} s), '
            f'odometrías: {", ".join(self.topics)}')

    def on_pose(self, topic, msg):
        self.ultima[topic] = pose2d(msg)

    def ahora(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def mandar(self, v, w):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.twist.linear.x = v
        m.twist.angular.z = w
        self.pub.publish(m)

    def step(self):
        t = self.ahora()
        if self.estado == 'esperando':
            faltan = [k for k in ['ground_truth'] + self.topics if k not in self.ultima]
            if faltan:
                self.get_logger().info(f'Esperando a {faltan}', throttle_duration_sec=2.0)
                return
            if self.t_estado is None:
                self.t_estado = t                       # todo llega: dejar que se asiente
            elif t - self.t_estado > self.espera:
                self.inicio = dict(self.ultima)
                self.estado, self.t_estado = 'moviendo', t
                self.get_logger().info('¡En marcha!')
        elif self.estado == 'moviendo':
            v, w = consigna(self.tramos, t - self.t_estado)
            self.mandar(v, w)
            if t - self.t_estado > duracion(self.tramos):
                self.estado, self.t_estado = 'parando', t
        elif self.estado == 'parando':
            self.mandar(0.0, 0.0)
            if t - self.t_estado > self.espera:
                self.resultados()
                self.estado = 'fin'

    def resultados(self):
        gt = relativa(self.inicio['ground_truth'], self.ultima['ground_truth'])
        filas = []
        self.get_logger().info(
            f'Real: x = {gt[0]:+.4f} m, y = {gt[1]:+.4f} m, '
            f'theta = {math.degrees(gt[2]):+.2f} deg')
        self.get_logger().info(f'{"odometría":<22} {"e_pos [mm]":>11} {"e_theta [deg]":>14}')
        for topic in self.topics:
            o = relativa(self.inicio[topic], self.ultima[topic])
            e_pos = math.hypot(o[0] - gt[0], o[1] - gt[1])
            e_th = wrap(o[2] - gt[2])
            self.get_logger().info(f'{topic:<22} {e_pos * 1000:>11.2f} {math.degrees(e_th):>14.3f}')
            filas.append([self.nombre, topic, f'{o[0]:.5f}', f'{o[1]:.5f}', f'{o[2]:.5f}',
                          f'{gt[0]:.5f}', f'{gt[1]:.5f}', f'{gt[2]:.5f}',
                          f'{e_pos:.6f}', f'{e_th:.6f}'])
        nuevo = not os.path.exists(self.salida)
        with open(self.salida, 'a', newline='') as f:
            w = csv.writer(f)
            if nuevo:
                w.writerow(['trayectoria', 'odometria', 'x', 'y', 'theta',
                            'x_real', 'y_real', 'theta_real', 'e_pos_m', 'e_theta_rad'])
            w.writerows(filas)
        self.get_logger().info(f'Resultados añadidos a {self.salida}')
        self.terminado = True


def main():
    rclpy.init()
    node = Experimento()
    try:
        while rclpy.ok() and not node.terminado:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
