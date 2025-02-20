"""En la terminal ejecutar los siguientes comandos: 

    colcon build --packages-select challenge1
    source install/setup.bash
    ros2 launch challenge1 challenge_launch.py


    equipo capucha:
    -MARIAM LANDA BAUTISTA
    -BRUNO MANUEL ZAMORA GARCIA
    -ELIAS GUERRA PENSADO
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np

class SignalGenerator(Node):
    def __init__(self):
        super().__init__('signal_generator')
        self.signal_pub = self.create_publisher(Float32, '/signal', 10)
        self.time_pub = self.create_publisher(Float32, '/time', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.time = 0.0

    def timer_callback(self):
        signal = np.sin(self.time)
        time_msg = Float32()
        time_msg.data = self.time
        signal_msg = Float32()
        signal_msg.data = signal

        self.time_pub.publish(time_msg)
        self.signal_pub.publish(signal_msg)
        self.get_logger().info(f'Tiempo: {self.time}, Senal: {signal}')

        self.time += 0.1

def main(args=None):
    rclpy.init(args=args)
    signal_generator = SignalGenerator()
    rclpy.spin(signal_generator)
    signal_generator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()