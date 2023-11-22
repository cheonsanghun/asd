import rclpy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class CarControlNode:
    def __init__(self):
        rclpy.init()  # rclpy를 초기화
        self.node = rclpy.create_node('car_control_node')
        self.car_id = None  # 현재 주행 중인 차량 ID
        self.subscription = self.node.create_subscription(
            String,
            "/start_car",
            self.start_car_callback,
            10
        )
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)

    def start_car_callback(self, msg):
        self.car_id = msg.data
        self.node.get_logger().info(f"Starting car: {self.car_id}")
        self.spawn_car()  # 차량을 Gazebo에 스폰

    def spawn_car(self):
        # Use os.system to run the spawn_entity.py command
        if self.car_id == "PR001":
            os.system("ros2 run gazebo_ros spawn_entity.py -database prius_hybrid -entity PR001 -x 93 -y -11.7 -Y -1.57")
        elif self.car_id == "PR002":
            os.system("ros2 run gazebo_ros spawn_entity.py -database prius_hybrid -entity PR002 -x 93 -y -15.9 -Y -1.57")

    def control_car(self):
        rate = self.node.create_rate(10)  # 10Hz
        while rclpy.ok():
            if self.car_id:
                # Your car control logic goes here
                # Use self.car_id to identify the car being controlled
                # Use self.cmd_vel_pub to publish Twist messages for controlling car's velocity

                # Example: Publish a Twist message with linear x velocity of 6 m/s
                twist_msg = Twist()
                twist_msg.linear.x = 6.0
                self.cmd_vel_pub.publish(twist_msg)

            rate.sleep()

    def run(self):
        self.control_car()
        self.node.destroy_node()
        rclpy.shutdown()

def main():
    car_control_node = CarControlNode()
    car_control_node.run()

if __name__ == '__main__':
    main()