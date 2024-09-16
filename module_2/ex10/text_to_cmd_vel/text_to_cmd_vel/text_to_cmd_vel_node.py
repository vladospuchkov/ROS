import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time

class TextToCmdVel(Node):
    def __init__(self):
        super().__init__('text_to_cmd_vel')
        self.subscription = self.create_subscription(
            String,
            'cmd_text',
            self.listener_callback,
            10
        )
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info("TextToCmdVel Node has been started")

    def listener_callback(self, msg):
        command = msg.data.lower()
        twist = Twist()

        # Определение команды движения
        if command == 'move_forward':
            twist.linear.x = 1.0
        elif command == 'move_backward':
            twist.linear.x = -1.0
        elif command == 'turn_left':
            twist.angular.z = 1.5
        elif command == 'turn_right':
            twist.angular.z = -1.5
        else:
            self.get_logger().warn(f"Unknown command: {command}")
            return

        # Публикация команды движения
        self.publisher_.publish(twist)
        self.get_logger().info(f"Published command: {command}")

        # Пауза перед остановкой
        time.sleep(1)  # Ожидаем 1 секунду

        # Останавливаем черепашку
        stop_twist = Twist()  # Все поля равны нулю (остановка)
        self.publisher_.publish(stop_twist)
        self.get_logger().info("Published stop command")

def main(args=None):
    rclpy.init(args=args)
    node = TextToCmdVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

