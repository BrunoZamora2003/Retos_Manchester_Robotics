import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np

class ProcessNode(Node):
    def __init__(self):
        super().__init__('process')
        self.signal_sub = self.create_subscription(Float32, '/signal', self.signal_callback, 10)
        self.time_sub = self.create_subscription(Float32, '/time', self.time_callback, 10)
        self.proc_signal_pub = self.create_publisher(Float32, '/proc_signal', 10)

        # Parámetros de modificación de la señal
        self.phase_shift = np.pi / 20  # Desfase en radianes
        self.amplitud = 0.69  # Factor de escalado de la amplitud 
        self.offset = 0.45  # Desplazamiento vertical
        self.smooth_filtro = 0.1 # Factor de suavizado para reducir cambios bruscos

        # Variables para almacenar el estado de la señal
        self.processed_signal = None  # Última señal procesada
        self.last_time = None  # Última marca de tiempo recibida
        self.time_threshold = 0.1  # Intervalo de actualización en segundos (10 Hz)

    def signal_callback(self, msg):
        """ Recibe la señal de entrada, la modifica y la almacena sin publicarla aún. """
        theta = np.arcsin(np.clip(msg.data, -1, 1))  # Convierte la señal a un ángulo válido
        theta_m = theta + self.phase_shift  # Aplica el desfase
        processed_signal = self.amplitud * np.sin(theta_m) + self.offset  # Modifica la amplitud y agrega offset

        # Aplica un filtro para suavizar la señal en caso de cambios bruscos
        # El filtro funciona evitando picos bruscos en la senal que se procesa,
        # en cualquier cambio repentino en la senal, se amortigua con el valor pasado.
        if self.processed_signal is not None:
            processed_signal = self.smooth_filtro * processed_signal + (1 - self.smooth_filtro) * self.processed_signal
        
        self.processed_signal = processed_signal  # Guarda la señal procesada

    def time_callback(self, msg):
        """ Publica la señal procesada cada 0.1s (10 Hz), sincronizado con los datos de tiempo. """
        current_time = msg.data

        # Verifica si ha pasado suficiente tiempo desde la última publicación
        if self.last_time is None or (current_time - self.last_time) >= self.time_threshold:
            self.last_time = current_time  # Actualiza la última marca de tiempo

            if self.processed_signal is not None:
                proc_signal_msg = Float32()
                proc_signal_msg.data = self.processed_signal
                self.proc_signal_pub.publish(proc_signal_msg)  # Publica la señal procesada

                self.get_logger().info(f'Tiempo: {current_time:.2f}s - Señal procesada: {self.processed_signal:.3f}')


def main(args=None):
    rclpy.init(args=args)
    process_node = ProcessNode()
    rclpy.spin(process_node)
    process_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
