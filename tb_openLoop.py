import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import matplotlib.pyplot as plt

class TurtleBotControl(Node):
    def __init__(self):
        super().__init__('turtlebot_control')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.vel_msg = Twist()

    def move_constant_velocity(self, v, t):
        self.vel_msg.linear.x = v
        self.publisher_.publish(self.vel_msg)

        times = []
        positions = []

        for elapsed_time in range(t + 1):
            position = v * elapsed_time
            acceleration = 0.0
            times.append(elapsed_time)
            positions.append(position)
            print(f'Time: {elapsed_time}s, Velocity: {v} m/s, Position: {position} m, Acceleration: {acceleration} m/s²')
            time.sleep(1)

        self.vel_msg.linear.x = 0.0
        self.publisher_.publish(self.vel_msg)

        return times, positions

    def move_with_acceleration(self, a, v_max, distance):
        t_accel = v_max / a
        times = []
        positions = []

        for elapsed_time in range(int(t_accel) + 1):
            self.vel_msg.linear.x = min(a * elapsed_time, v_max)
            self.publisher_.publish(self.vel_msg)

            position = 0.5 * a * (elapsed_time ** 2)
            acceleration = a
            times.append(elapsed_time)
            positions.append(position)
            print(f'Time: {elapsed_time}s, Velocity: {self.vel_msg.linear.x} m/s, Position: {position} m, Acceleration: {acceleration} m/s²')
            time.sleep(1)

        duration = distance / v_max
        for elapsed_time in range(int(duration)):
            self.publisher_.publish(self.vel_msg)

            position = (0.5 * a * (t_accel ** 2)) + (v_max * elapsed_time)
            acceleration = 0.0
            times.append(elapsed_time + t_accel)
            positions.append(position)
            print(f'Time: {elapsed_time + t_accel}s, Velocity: {self.vel_msg.linear.x} m/s, Position: {position} m, Acceleration: {acceleration} m/s²')
            time.sleep(1)

        for elapsed_time in range(int(t_accel) + 1):
            self.vel_msg.linear.x = max(v_max - a * elapsed_time, 0.0)
            self.publisher_.publish(self.vel_msg)

            position = (0.5 * a * (t_accel ** 2)) + (v_max * duration) - (0.5 * a * (elapsed_time ** 2))
            acceleration = -a
            times.append(elapsed_time + t_accel + duration)
            positions.append(position)
            print(f'Time: {elapsed_time + t_accel + duration}s, Velocity: {self.vel_msg.linear.x} m/s, Position: {position} m, Acceleration: {acceleration} m/s²')
            time.sleep(1)

        self.vel_msg.linear.x = 0.0
        self.publisher_.publish(self.vel_msg)

        return times, positions

    def plot_results(self, times1, positions1, times2, positions2):
        plt.figure(figsize=(10, 5))
        plt.plot(times1, positions1, label='Constant Velocity', color='blue', marker='o')
        plt.plot(times2, positions2, label='With Acceleration', color='red', marker='o')
        plt.title('Position vs Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Position (m)')
        plt.legend()
        plt.grid()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    turtlebot_control = TurtleBotControl()

    try:
        times1, positions1 = turtlebot_control.move_constant_velocity(0.2, 10)
        times2, positions2 = turtlebot_control.move_with_acceleration(0.1, 0.5, 2)
        turtlebot_control.plot_results(times1, positions1, times2, positions2)

    except Exception as e:
        turtlebot_control.get_logger().error(f'An error occurred: {e}')
    finally:
        turtlebot_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

