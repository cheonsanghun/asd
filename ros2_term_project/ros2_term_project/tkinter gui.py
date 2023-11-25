import rclpy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import tkinter as tk
from tkinter import ttk

class CarControlNode:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('car_control_node')
        self.car_id = None
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)

        # Tkinter GUI
        self.root = tk.Tk()
        self.root.title("Car Control GUI")

        self.linear_scale = ttk.Scale(self.root, from_=0.0, to=10.0, orient='horizontal', length=200,
                                      label="Linear Velocity (m/s)", command=self.update_linear)
        self.linear_scale.set(6.0)
        self.linear_scale.pack(pady=10)

        self.angular_scale = ttk.Scale(self.root, from_=-3.0, to=3.0, orient='horizontal', length=200,
                                       label="Angular Velocity (rad/s)", command=self.update_angular)
        self.angular_scale.set(0.0)
        self.angular_scale.pack(pady=10)

        self.apply_button = ttk.Button(self.root, text="Apply", command=self.apply_values)
        self.apply_button.pack(pady=10)

    def update_linear(self, value):
        self.linear_value = float(value)

    def update_angular(self, value):
        self.angular_value = float(value)

    def apply_values(self):
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_value
        twist_msg.angular.z = self.angular_value
        self.cmd_vel_pub.publish(twist_msg)

    def run(self):
        self.root.mainloop()
        self.node.destroy_node()
        rclpy.shutdown()

def main():
    car_control_node = CarControlNode()
    car_control_node.run()

if __name__ == '__main__':
    main()
