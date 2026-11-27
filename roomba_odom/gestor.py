"""Gestor de permisos de movimiento.

Servicio:
  gestor/permiso   std_srvs/Trigger -> success = ¿puede moverse?

Parámetro (se puede cambiar en marcha):
  conceder   true | false
    ros2 param set /gestor conceder false   # denegar en la próxima petición
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Trigger


class Gestor(Node):
    def __init__(self):
        super().__init__('gestor')
        self.declare_parameter('conceder', True)
        self.create_service(Trigger, 'gestor/permiso', self.on_permiso)
        self.get_logger().info('Gestor listo en gestor/permiso')

    def on_permiso(self, _req, res):
        res.success = bool(self.get_parameter('conceder').value)
        res.message = 'permiso concedido' if res.success else 'permiso denegado'
        self.get_logger().info(res.message)
        return res


def main():
    rclpy.init()
    node = Gestor()
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
