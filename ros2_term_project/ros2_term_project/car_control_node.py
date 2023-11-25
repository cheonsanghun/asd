import rclpy
from std_msgs.msg import String

def timer_callback(timer, msg, publisher):
    # timer 매개변수를 사용하지 않으므로 _로 대체
    publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('car_control_node')
    publisher = node.create_publisher(String, '/start_car', 10)

    msg = String()
    car_name = input('차량 이름을 입력하세요 (PR001 또는 PR002): ')
    msg.data = car_name

    timer_period = 1  # 초
    timer = node.create_timer(timer_period, lambda _: timer_callback(_, msg, publisher))

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()