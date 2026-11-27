"""Órdenes de «voz» por texto, en lazo cerrado con la odometría.

Suscribe:
  voz              std_msgs/String: adelante | atras | izquierda | derecha | parar
  odom             nav_msgs/Odometry (la de odometry_node)
  emergency_stop   std_msgs/Bool: cancela la orden en curso
Publica:
  cmd_vel_voz      geometry_msgs/TwistStamped a 20 Hz mientras ejecuta una orden

  adelante / atras    1 m (distancia proyectada sobre el rumbo inicial)
  izquierda/derecha   una vuelta completa, +2π / -2π. El giro se ACUMULA
                      sumando incrementos de yaw: el yaw de /odom está en
                      (-π, π] y nunca «llega» a 2π.

Si el guardián bloquea (sin permiso), la orden sigue pendiente y se completa
cuando vuelva el permiso, porque se mide con la odometría y no con el tiempo.

Prueba:
  ros2 topic pub --once /voz std_msgs/String "data: adelante"
"""
import math
import unicodedata

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String

from roomba_odom.experimento import pose2d
from roomba_odom.guardian import QOS_LATCHED
from roomba_odom.integracion import wrap

ORDENES = {
    'adelante': ('lineal', +1.0),
    'atras': ('lineal', -1.0),
    'izquierda': ('giro', +2.0 * math.pi),
    'derecha': ('giro', -2.0 * math.pi),
}


def normalizar(texto):
    t = unicodedata.normalize('NFKD', texto.strip().lower())
    return ''.join(c for c in t if not unicodedata.combining(c))      # atrás -> atras


class VozNode(Node):
    def __init__(self):
        super().__init__('voz_node')
        p = self.declare_parameter
        self.distancia = p('distancia', 1.0).value
        self.v_max = p('v_max', 0.25).value
        self.w_max = p('w_max', 1.0).value
        self.k_v = p('k_v', 1.0).value
        self.k_w = p('k_w', 1.5).value
        self.tol_d = p('tol_distancia', 0.005).value
        self.tol_a = math.radians(p('tol_angulo_deg', 1.0).value)

        self.pose = None
        self.orden = None              # dict con tipo, objetivo, inicio, acumulado
        self.create_subscription(String, 'voz', self.on_voz, 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 20)
        self.create_subscription(Bool, 'emergency_stop', self.on_estop, QOS_LATCHED)
        self.pub = self.create_publisher(TwistStamped, 'cmd_vel_voz', 10)
        self.create_timer(0.05, self.step)
        self.get_logger().info(f'Órdenes: {", ".join(ORDENES)}, parar')

    def on_odom(self, msg):
        nueva = pose2d(msg)
        if self.orden is not None and self.orden['tipo'] == 'giro' and self.pose is not None:
            self.orden['giro'] += wrap(nueva[2] - self.pose[2])       # acumular
        self.pose = nueva

    def on_estop(self, msg):
        if msg.data and self.orden is not None:
            self.get_logger().warn('Emergencia: orden cancelada')
            self.terminar()

    def on_voz(self, msg):
        palabra = normalizar(msg.data)
        if palabra == 'parar':
            self.terminar()
            return
        if palabra not in ORDENES:
            self.get_logger().warn(f'Orden no reconocida: "{msg.data}"')
            return
        if self.pose is None:
            self.get_logger().warn('Todavía no hay odometría')
            return
        tipo, objetivo = ORDENES[palabra]
        if tipo == 'lineal':
            objetivo *= self.distancia
        self.orden = {'nombre': palabra, 'tipo': tipo, 'objetivo': objetivo,
                      'inicio': self.pose, 'giro': 0.0}
        self.get_logger().info(f'Orden: {palabra}')

    def step(self):
        if self.orden is None or self.pose is None:
            return
        o = self.orden
        if o['tipo'] == 'lineal':
            x0, y0, p0 = o['inicio']
            hecho = (self.pose[0] - x0) * math.cos(p0) + (self.pose[1] - y0) * math.sin(p0)
            falta = o['objetivo'] - hecho
            if abs(falta) < self.tol_d:
                self.terminar(f'{hecho:+.3f} m')
                return
            v = max(-self.v_max, min(self.v_max, self.k_v * falta))
            if abs(v) < 0.03:
                v = math.copysign(0.03, falta)          # no quedarse a medias
            self.mandar(v, 0.0)
        else:
            falta = o['objetivo'] - o['giro']
            if abs(falta) < self.tol_a:
                self.terminar(f'{math.degrees(o["giro"]):+.1f} deg')
                return
            w = max(-self.w_max, min(self.w_max, self.k_w * falta))
            if abs(w) < 0.15:
                w = math.copysign(0.15, falta)
            self.mandar(0.0, w)

    def mandar(self, v, w):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.twist.linear.x = v
        m.twist.angular.z = w
        self.pub.publish(m)

    def terminar(self, resultado=None):
        if self.orden is not None:
            fin = f' ({resultado})' if resultado else ' (cancelada)'
            self.get_logger().info(f'Orden {self.orden["nombre"]} terminada{fin}')
        self.orden = None
        self.mandar(0.0, 0.0)


def main():
    rclpy.init()
    node = VozNode()
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
