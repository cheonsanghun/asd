import rclpy
from geometry_msgs.msg import Twist
import sys

def control_vehicle(vehicle_name):
    rclpy.init()
    node = rclpy.create_node('vehicle_controller')

    twist_pub = node.create_publisher(Twist, f'/{vehicle_name}/cmd_vel', 10)

    # 사용자 입력을 대신하여 원하는 값을 직접 입력
    linear_speed = 10.0
    angular_speed = 5.0

    while rclpy.ok():
        twist_msg = Twist()
        twist_msg.linear.x = linear_speed
        twist_msg.angular.z = angular_speed

        twist_pub.publish(twist_msg)
        rclpy.spin_once(node)
        print(f"Published Twist message: {twist_msg}")

    rclpy.shutdown()

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 control_spawned_vehicle.py <vehicle_name>")
        sys.exit(1)

    vehicle_name = sys.argv[1]
    control_vehicle(vehicle_name)

if __name__ == '__main__':
    main()
