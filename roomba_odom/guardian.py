"""Guardián: única puerta entre las órdenes de movimiento y las ruedas.

  cmd_vel_in   (teleop, experimento...)  ─┐
  cmd_vel_voz  (voz_node, tiene prioridad) ┼─► guardián ─► cmd_vel ─► kinematics_node
  emergency_stop  std_msgs/Bool           ─┤
  gestor/permiso  std_srvs/Trigger        ─┘

Reglas:
  - Cada periodo_permiso s (10) pide permiso al gestor. El permiso vale hasta
    la siguiente petición (+ margen). Sin respuesta, o con respuesta «no», no
    hay permiso.
  - emergency_stop = true bloquea todo hasta que se publique false. El topic
    es «latched» (TRANSIENT_LOCAL): quien se conecte tarde recibe el último valor.
  - Bloqueado -> publica velocidad cero a 20 Hz (no basta con callarse: así
    se para aunque alguien publique directamente en cmd_vel).

Publica también guardian/estado (std_msgs/String) para ver por qué no se mueve.
"""
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

QOS_LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)


class Guardian(Node):
    def __init__(self):
        super().__init__('guardian')
        p = self.declare_parameter
        self.periodo = p('periodo_permiso', 10.0).value
        self.margen = p('margen_permiso', 1.0).value
        self.cmd_timeout = p('cmd_timeout', 0.3).value

        self.estop = False
        self.t_permiso = None          # instante del último permiso concedido
        self.pidiendo = False
        self.cmd = {'in': None, 'voz': None}     # (t, TwistStamped)
        self.estado_txt = ''

        self.create_subscription(TwistStamped, 'cmd_vel_in', lambda m: self.on_cmd('in', m), 10)
        self.create_subscription(TwistStamped, 'cmd_vel_voz', lambda m: self.on_cmd('voz', m), 10)
        self.create_subscription(Bool, 'emergency_stop', self.on_estop, QOS_LATCHED)
        self.pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.pub_estado = self.create_publisher(String, 'guardian/estado', QOS_LATCHED)
        self.cli = self.create_client(Trigger, 'gestor/permiso')

        self.create_timer(self.periodo, self.pedir_permiso)
        self.create_timer(0.05, self.step)
        self.pedir_permiso()           # la primera, nada más arrancar
        self.get_logger().info(f'Guardián: permiso cada {self.periodo:g} s')

    def ahora(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # --- entradas
    def on_cmd(self, fuente, msg):
        self.cmd[fuente] = (self.ahora(), msg)

    def on_estop(self, msg):
        if msg.data != self.estop:
            self.get_logger().warn('¡FRENO DE EMERGENCIA!' if msg.data else 'Freno liberado')
        self.estop = msg.data

    # --- permiso
    def pedir_permiso(self):
        if self.pidiendo:
            return                     # la anterior aún no ha contestado
        if not self.cli.service_is_ready():
            self.get_logger().warn('Gestor no disponible: sin permiso', throttle_duration_sec=5.0)
            return
        self.pidiendo = True
        self.cli.call_async(Trigger.Request()).add_done_callback(self.on_permiso)

    def on_permiso(self, fut):
        self.pidiendo = False
        try:
            ok = fut.result().success
        except Exception as e:                 # noqa: BLE001
            self.get_logger().error(f'Error pidiendo permiso: {e}')
            ok = False
        if ok:
            self.t_permiso = self.ahora()
        else:
            self.t_permiso = None
            self.get_logger().warn('Permiso denegado por el gestor')

    def tiene_permiso(self):
        return (self.t_permiso is not None and
                self.ahora() - self.t_permiso < self.periodo + self.margen)

    # --- salida
    def step(self):
        t = self.ahora()
        if self.estop:
            self.estado('EMERGENCIA')
            self.cero()
            return
        if not self.tiene_permiso():
            self.estado('SIN PERMISO')
            self.cero()
            return
        for fuente in ('voz', 'in'):           # la voz manda si está activa
            c = self.cmd[fuente]
            if c is not None and t - c[0] < self.cmd_timeout:
                self.estado(f'OK ({fuente})')
                c[1].header.stamp = self.get_clock().now().to_msg()
                self.pub.publish(c[1])
                return
        self.estado('OK (sin órdenes)')        # kinematics_node para solo por timeout

    def cero(self):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        self.pub.publish(m)

    def estado(self, txt):
        if txt != self.estado_txt:
            self.estado_txt = txt
            self.pub_estado.publish(String(data=txt))
            self.get_logger().info(f'Estado: {txt}')


def main():
    rclpy.init()
    node = Guardian()
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
