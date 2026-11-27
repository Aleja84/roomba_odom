"""Cinemática inversa: /cmd_vel -> velocidades de rueda para JointGroupVelocityController.

Suscribe:
  cmd_vel   geometry_msgs/TwistStamped (Twist si stamped:=false)
Publica:
  wheel_velocity_controller/commands   std_msgs/Float64MultiArray [w_L, w_R] en rad/s
      El orden es el de 'joints' en config/controllers.yaml:
      [left_wheel_joint, right_wheel_joint].

Si no llegan órdenes durante cmd_timeout s, manda ceros (el
JointGroupVelocityController mantiene la última consigna indefinidamente).
"""
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from roomba_odom.integracion import inversa


class KinematicsNode(Node):
    def __init__(self):
        super().__init__('kinematics_node')
        p = self.declare_parameter
        self.r = p('wheel_radius', 0.036).value
        self.L = p('wheel_separation', 0.235).value
        self.w_max = p('max_wheel_velocity', 0.0).value    # rad/s; 0 = sin límite
        self.timeout = p('cmd_timeout', 0.5).value
        stamped = p('stamped', True).value
        topic_out = p('command_topic', 'wheel_velocity_controller/commands').value

        self.pub = self.create_publisher(Float64MultiArray, topic_out, 10)
        if stamped:
            self.create_subscription(TwistStamped, 'cmd_vel', lambda m: self.on_cmd(m.twist), 10)
        else:
            self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.t_cmd = None
        self.parado = True
        self.create_timer(0.05, self.watchdog)
        self.get_logger().info(f'r = {self.r} m, L = {self.L} m -> {topic_out}')

    def on_cmd(self, t):
        w_R, w_L = inversa(t.linear.x, t.angular.z, self.r, self.L)
        if self.w_max > 0.0:
            # escalar las dos ruedas por igual conserva la curvatura v/w
            s = max(abs(w_R), abs(w_L)) / self.w_max
            if s > 1.0:
                w_R, w_L = w_R / s, w_L / s
        self.enviar(w_L, w_R)
        self.t_cmd = self.get_clock().now()
        self.parado = False

    def watchdog(self):
        if self.parado or self.t_cmd is None:
            return
        if (self.get_clock().now() - self.t_cmd).nanoseconds * 1e-9 > self.timeout:
            self.enviar(0.0, 0.0)
            self.parado = True

    def enviar(self, w_L, w_R):
        self.pub.publish(Float64MultiArray(data=[float(w_L), float(w_R)]))


def main():
    rclpy.init()
    node = KinematicsNode()
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
