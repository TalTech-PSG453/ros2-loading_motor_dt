#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from digital_twin_msgs.msg import Current
from digital_twin_msgs.msg import AxesCurrents
import threading
from math import pow, sqrt
import matlab.engine
import matlab
import sys
import os

SIZE_A = 5000

class WindingErrorChecker(Node):

    def __init__(self):
        super().__init__('windings_checker')

        self.currents_listener_ = self.create_subscription(
            Current,
            'input_current',
            self.current_callback,
            10)

        self.currents_listener_  # prevent unused variable warning
        self.warnings_publisher_ = self.create_publisher(String, 'diagnostics/warnings', 10)
        self.axes_currents_publisher_ = self.create_publisher(AxesCurrents, 'diagnostics/windings/axes_currents', 10)

        self.declare_parameter('matlab_path')
        param = self.get_parameter('matlab_path')
        matlab_script_path = param.get_parameter_value().string_value

        self.i = 0
        self.matlab_active = False
        self.can_calculate = False
        self.currents_= [[] for i in range(3)]

        phase_check_thread = threading.Thread(target = self.phase_checker, daemon = True)
        self.matlab_thread = threading.Thread(target = self.matlab_analysis, args=(matlab_script_path,), daemon=True)
        phase_check_thread.start()

        # Axes currents
        self.ids = []
        self.iqs = []

        self.get_logger().info('Init happened')
        self.get_logger().info('Matlab script path set to: "%s"' % matlab_script_path)

    def publish_warning(self):
        msg = String()
        msg.data = 'Winding warning'
        self.warnings_publisher_.publish(msg)
    
    def publish_axes_currents(self):
        msg = AxesCurrents()
        msg.q_axis_current = self.iqs
        msg.d_axis_current = self.ids
        self.axes_currents_publisher_.publish(msg)

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

    def matlab_analysis(self, script_path):
        self.get_logger().info('Starting Park & Clarke analysis in Matlab')
        b = self.currents_buf_
        ml_input = []
        for i in range(3):
            ml_input.append(matlab.double(b[i]))

        try:
            eng = matlab.engine.start_matlab()
        except:
            e = sys.exc_info()[0]
            self.get_logger().error("Failed to open MatLab Engine. Error: %s", e)
            exit()

        eng.cd(script_path, nargout=0)

        try:
            x, y = eng.parkclarke(ml_input[0], ml_input[1], ml_input[2], nargout = 2)
        except:
            e = sys.exc_info()[0]
            self.get_logger().error("Failed to run Park & Clarke function. Error: %s", e)
            eng.quit()
            exit()

        for element in x[0]:
            self.ids.append(element)

        for element in y[0]:
            self.iqs.append(element)

        self.publish_axes_currents()

        eng.quit()
        self.get_logger().info('Matlab analysis finished')

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
