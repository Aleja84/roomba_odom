"""Botón de frenado de emergencia (ventana con tkinter).

Publica emergency_stop (std_msgs/Bool, latched). El botón rojo frena y
queda enclavado; el verde lo libera.

Sin ventana (o sin python3-tk), lo mismo desde la terminal:
  ros2 topic pub --once -q /emergency_stop std_msgs/Bool "data: true" \
      --qos-durability transient_local --qos-reliability reliable
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from roomba_odom.guardian import QOS_LATCHED


class Boton(Node):
    def __init__(self):
        super().__init__('boton_emergencia')
        self.pub = self.create_publisher(Bool, 'emergency_stop', QOS_LATCHED)
        self.publicar(False)

    def publicar(self, valor):
        self.pub.publish(Bool(data=valor))
        self.get_logger().warn('EMERGENCIA activada' if valor else 'Emergencia liberada')


def main():
    rclpy.init()
    node = Boton()
    try:
        import tkinter as tk
    except ImportError:
        node.get_logger().error('Falta tkinter (sudo apt install python3-tk). '
                                'Usa ros2 topic pub, ver la ayuda del módulo.')
        node.destroy_node()
        rclpy.try_shutdown()
        return

    root = tk.Tk()
    root.title('Roomba: emergencia')
    estado = tk.StringVar(value='Libre')

    def frenar():
        node.publicar(True)
        estado.set('FRENADO')
        lbl.config(fg='red')

    def liberar():
        node.publicar(False)
        estado.set('Libre')
        lbl.config(fg='green')

    tk.Button(root, text='STOP', command=frenar, bg='red', fg='white',
              activebackground='darkred', font=('Helvetica', 32, 'bold'),
              width=8, height=2).pack(padx=20, pady=(20, 10))
    tk.Button(root, text='Liberar', command=liberar, font=('Helvetica', 14)).pack(pady=5)
    lbl = tk.Label(root, textvariable=estado, fg='green', font=('Helvetica', 16, 'bold'))
    lbl.pack(pady=(5, 20))
    root.bind('<space>', lambda _e: frenar())   # barra espaciadora = STOP

    def girar():
        rclpy.spin_once(node, timeout_sec=0.0)
        if rclpy.ok():
            root.after(50, girar)

    root.protocol('WM_DELETE_WINDOW', root.quit)
    root.after(50, girar)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
