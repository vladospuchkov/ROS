import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class SnowmanTrajectory(Node):
    def __init__(self):
        super().__init__('snowman_trajectory')
        self.publisher_ = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Параметры траектории
        self.phase = 'circle'  # текущая фаза движения (circle или transition)
        self.circle_number = 1  # текущий круг (1, 2 или 3)
        self.circle_radius = 1.5  # радиус первого круга
        self.linear_speed = 0.5  # линейная скорость
        self.angular_speed = self.linear_speed / self.circle_radius  # угловая скорость для первого круга
        self.transition_distance = 0.5  # расстояние между кругами
        self.angle_travelled = 0.0  # угол, пройденный роботом по текущему кругу (радианы)
        self.transition_distance_travelled = 0.0  # расстояние при линейном перемещении

    def timer_callback(self):
        twist_msg = Twist()

        if self.phase == 'circle':
            twist_msg.linear.x = self.linear_speed
            twist_msg.angular.z = self.angular_speed

            self.angle_travelled += self.angular_speed * self.timer_period

            # Проверка завершения круга
            if self.angle_travelled >= 2 * math.pi:
                self.angle_travelled = 0.0
                self.phase = 'transition'
                self.circle_number += 1

                if self.circle_number == 2:
                    self.circle_radius = 1.0  # радиус второго круга
                elif self.circle_number == 3:
                    self.circle_radius = 0.5  # радиус третьего круга

                self.angular_speed = self.linear_speed / self.circle_radius

        elif self.phase == 'transition':
            twist_msg.linear.x = self.linear_speed
            twist_msg.angular.z = 0.0

            self.transition_distance_travelled += self.linear_speed * self.timer_period

            # Проверка завершения линейного перехода
            if self.transition_distance_travelled >= self.transition_distance:
                self.transition_distance_travelled = 0.0
                self.phase = 'circle'

                # Если это был третий круг, завершить движение
                if self.circle_number > 3:
                    self.get_logger().info('Snowman trajectory completed.')
                    self.stop_robot()
                    rclpy.shutdown()

        self.publisher_.publish(twist_msg)

    def stop_robot(self):
        """Останавливает робот, публикуя нулевые скорости."""
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.publisher_.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SnowmanTrajectory()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

