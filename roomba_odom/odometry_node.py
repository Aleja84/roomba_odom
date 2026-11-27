"""Odometría a partir de los encoders (/joint_states).

Suscribe:
  joint_states   sensor_msgs/JointState (posición de las ruedas, rad)
Publica:
  odom           nav_msgs/Odometry  (topic configurable con odom_topic)
  TF             odom -> base_link  (si publish_tf)

Parámetros:
  integration    euler | rk2 | exact
  rate           Hz a los que se integra. Cada periodo se toma el último
                 JointState recibido y se integra el incremento de rueda
                 desde el anterior. 0 = integrar cada mensaje que llegue.
  wheel_radius, wheel_separation, left_joint, right_joint, odom_frame, base_frame
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from roomba_odom.integracion import METODOS, incrementos, paso


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        p = self.declare_parameter
        self.metodo = p('integration', 'exact').value
        if self.metodo not in METODOS:
            raise ValueError(f'integration debe ser uno de {METODOS}, no {self.metodo!r}')
        self.rate = p('rate', 50.0).value
        self.r = p('wheel_radius', 0.036).value
        self.L = p('wheel_separation', 0.235).value
        self.j_left = p('left_joint', 'left_wheel_joint').value
        self.j_right = p('right_joint', 'right_wheel_joint').value
        self.odom_frame = p('odom_frame', 'odom').value
        self.base_frame = p('base_frame', 'base_link').value
        self.publish_tf = p('publish_tf', True).value
        odom_topic = p('odom_topic', 'odom').value

        self.x = self.y = self.phi = 0.0
        self.ultimo = None        # (t, th_L, th_R) del último JointState recibido
        self.previo = None        # (t, th_L, th_R) del último que se integró

        self.pub = self.create_publisher(Odometry, odom_topic, 10)
        self.tf = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(JointState, 'joint_states', self.on_js, 50)
        if self.rate > 0.0:
            self.create_timer(1.0 / self.rate, self.integrar)
        self.get_logger().info(
            f'Integración {self.metodo} a {self.rate:g} Hz, r = {self.r} m, L = {self.L} m '
            f'-> {odom_topic}{" + TF" if self.publish_tf else ""}')

    def on_js(self, msg):
        try:
            iL = msg.name.index(self.j_left)
            iR = msg.name.index(self.j_right)
        except ValueError:
            return
        if len(msg.position) <= max(iL, iR):
            return
        t = Time.from_msg(msg.header.stamp)
        if t.nanoseconds == 0:
            t = self.get_clock().now()
        self.ultimo = (t, msg.position[iL], msg.position[iR])
        if self.rate <= 0.0:
            self.integrar()

    def integrar(self):
        if self.ultimo is None:
            return
        if self.previo is None:
            self.previo = self.ultimo            # primera lectura: solo referencia
            return
        t, thL, thR = self.ultimo
        t0, thL0, thR0 = self.previo
        dt = (t - t0).nanoseconds * 1e-9
        if dt <= 0.0:
            return                               # no ha llegado nada nuevo
        ds, dphi = incrementos(thR - thR0, thL - thL0, self.r, self.L)
        self.x, self.y, self.phi = paso(self.x, self.y, self.phi, ds, dphi, self.metodo)
        self.previo = self.ultimo
        self.publicar(t, ds / dt, dphi / dt)

    def publicar(self, t, v, w):
        qz, qw = math.sin(self.phi / 2.0), math.cos(self.phi / 2.0)
        odom = Odometry()
        odom.header.stamp = t.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.pub.publish(odom)
        if self.tf is not None:
            tf = TransformStamped()
            tf.header = odom.header
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf.sendTransform(tf)


def main():
    rclpy.init()
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:          # al cerrar, algún callback puede fallar con el contexto ya invalido
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
