import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Int64MultiArray

import time


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('move_servos_publisher')
        self.publisher_ = self.create_publisher(Int64MultiArray, 'multi_servo_cmd_sub', 10)
        timer_period = 3  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # pick up stuff
        # data_sets = [[2000,12000,20000,3000,-1,3000,500,500,500,500,500,500],
        #             [16000,-1,-1,-1,-1,-1,500,500,500,500,500,500],
        #             [16000,12000,12000,12000,12000,12000,500,500,500,500,500,500],
        #             [12000,12000,12000,12000,12000,12000,500,500,500,500,500,500]]

        # msg = Int64MultiArray()

        # msg.data = data_sets[self.i]
        # self.publisher_.publish(msg)

        # self.i += 1
        # if self.i == 4:
        #     self.i = 0

        
        # random movement
        data_sets = [[7000,13000,14000,10000,16000,8000,2000,2000,2000,2000,2000,2000],
                    [12000,12000,12000,12000,12000,12000,2000,2000,2000,2000,2000,2000]]
        msg = Int64MultiArray()
        msg.data = data_sets[self.i]
        self.publisher_.publish(msg)

        self.i += 1
        if self.i == 2:
            self.i = 0

        
        # self.get_logger().info('Publishing: "%s"' % str(msg.data))
        # self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()