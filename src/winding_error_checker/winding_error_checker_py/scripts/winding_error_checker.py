#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from digital_twin_msgs.msg import Current
import threading
import math

SIZE_A = 5000

class WindingErrorChecker(Node):

    def __init__(self):
        super().__init__('windings_checker')

        self.currents_listener_ = self.create_subscription(
            Current,
            'tb_lm/input_current',
            self.current_callback,
            10)

        self.currents_listener_  # prevent unused variable warning
        self.warnings_publisher_ = self.create_publisher(String, 'diagnostics/warnings/windings', 10)
        
        self.i = 0
        self.currents_= [[] for i in range(3)]
        self.currents_buf = [[] for i in range(3)]
        self.can_calculate = False
        phase_check_thread = threading.Thread(target = self.phase_checker, daemon = True)

    def publish_warning(self):
        msg = String()
        msg.data = 'Winding warning'
        self.warnings_publisher_.publish(msg)

    def current_callback(self, msg):
        if(self.i < SIZE_A):
            self.i = 0
            self.currents_buf_ = self.currents_
            self.can_calculate = True
            self.currents_ = [[] for i in range(3)]


        self.currents_.insert(1, msg.current1)
        self.currents_.insert(2, msg.current2)
        self.currents_.insert(3, msg.current3)
        self.i += 1

    def phase_checker(self):
        square = [0, 0, 0]
        mean = [0, 0, 0]
        root = [0, 0, 0]
        error_coeff = [0, 0, 0]

        while True:
            if self.can_calculate:
                print('can calculate')
                for sq in range(SIZE_A):
                    square[0] += pow(self.currents_buf_[0][sq], self.currents_buf_[0][sq])
                    square[1] += pow(self.currents_buf_[1][sq], self.currents_buf_[1][sq])
                    square[2] += pow(self.currents_buf_[2][sq], self.currents_buf_[2][sq])

                for m in range(3):
                    mean[m] = square[m] / SIZE_A

                for r in range(3):
                    root[r] = math.sqrt(mean[r])

                for e in range(3):
                    error_coeff[e] = root[e] / root[0]

                for err in error_coeff:
                    if abs(error_coeff[0] - err) >= 0.15:
                        self.publish_warning()

            self.can_calculate = False

def main(args=None):
    rclpy.init(args=args)

    windings_controller = WindingErrorChecker()

    rclpy.spin(windings_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    windings_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
