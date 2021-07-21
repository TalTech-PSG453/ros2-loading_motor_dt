#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from digital_twin_msgs.msg import Current
import threading
from math import pow, sqrt
import matlab.engine
import matlab
import sys

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
        self.warnings_publisher_ = self.create_publisher(String, 'diagnostics/windings/warnings', 10)
        #self.analysis_publisher_ = self.create_publisher(String, 'diagnostics/windings/park_clarke', 10)
        
        self.i = 0
        self.matlab_active = False
        self.can_calculate = False
        self.currents_= [[] for i in range(3)]
        self.currents_buf = [[] for i in range(3)]

        phase_check_thread = threading.Thread(target = self.phase_checker, daemon = True)
        self.matlab_thread = threading.Thread(target = self.matlab_analysis, daemon=True)
        phase_check_thread.start()
        self.get_logger().info('Init happened')

    def publish_warning(self):
        msg = String()
        msg.data = 'Winding warning'
        self.warnings_publisher_.publish(msg)
    
    #def publish_clarke_park(self):
    #    msg = String()
    #    msg.data = 'TEST MESSAGE'
    #    self.warnings_publisher_.publish(msg)

    def current_callback(self, msg):
        if(self.i >= SIZE_A):
            self.i = 0
            self.currents_buf_ = self.currents_
            self.can_calculate = True
            self.currents_ = [[] for i in range(3)]

        self.currents_[0].append(msg.current1)
        self.currents_[1].append(msg.current2)
        self.currents_[2].append(msg.current3)
        self.i += 1

    def matlab_analysis(self):
        b = self.currents_buf_
        # fix the above mess...
        b1 = matlab.double(b[1])
        b2 = matlab.double(b[2])
        b3 = matlab.double(b[0])
        print(type(b1))
        eng = matlab.engine.start_matlab()
        eng.cd(r'/home/sejego', nargout=0)      #substitute this for variable
        x, y = eng.parkclarke(b1,b2,b3, nargout = 2)
        print(type(x))
        print(type(y))
        print('-----')
        print(sys.getsizeof(x))
        print(sys.getsizeof(y))
        eng.quit()

    def get_rms(self, buffer):
        square = [0, 0, 0]
        mean = [0, 0, 0]
        root = [0, 0, 0]

        for sq in range(SIZE_A):
            square[0] += pow(buffer[0][sq], 2)
            square[1] += pow(buffer[1][sq], 2)
            square[2] += pow(buffer[2][sq], 2)

        for m in range(3):
            mean[m] = square[m] / SIZE_A
        for r in range(3):
            root[r] = sqrt(mean[r])
        return root

    def phase_checker(self):
        problematic = False
        error_coeff = [0, 0, 0]
        while True:
            if self.can_calculate:
                rms = self.get_rms(self.currents_buf_)
                for e in range(3):
                    error_coeff[e] = rms[e] / rms[0]

                for err in error_coeff:
                    if abs(error_coeff[0] - err) >= 0.15:
                        problematic = True
                        break

                if problematic:
                    self.get_logger().warn('Potential winding problem')
                    self.publish_warning()
                    problematic = False
                    if not self.matlab_active:
                        self.matlab_thread.start()
                        self.matlab_active = True

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
