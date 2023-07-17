import sys

from pathlib import Path

CURRENT_POSITION = Path(__file__).parent
sys.path.append(f"{CURRENT_POSITION}/../../")

from lib.models.cart2d import Cart2D
from lib.models.robot import RoboticSystem
from lib.controllers.standard import PIDSat
from lib.gui.gui_2d import CartWindow
from lib.models.virtual_robot import SpeedProfileGenerator2D, SpeedProfileGenerator
from lib.data.plot import DataPlotter

from PyQt5.QtWidgets import QApplication


class Cart2DRobot(RoboticSystem):

    def __init__(self):
        super().__init__(1e-3)  # delta_t = 1e-3
        # Mass = 1kg
        # radius = 15cm
        # friction = 0.8
        self.cart = Cart2D(1, 0.15, 0.8, 0.8)
        self.linear_speed_controller = PIDSat(10, 3.5, 0, 5)  # 5 newton
        self.angular_speed_controller = PIDSat(6, 10, 0, 4)  # 4 newton * metro

        self.target = (0.8, 0.5)
        self.linear_speed_profile_controller = SpeedProfileGenerator2D(self.target, 0.2, 0.5, 0.5)
        self.angular_speed_profile_controller = SpeedProfileGenerator(0, 2, 4, 4)

        self.plotter = DataPlotter()

    def run(self):
        v_target = self.linear_speed_profile_controller.evaluate(self.delta_t, self.get_pose())

        self.angular_speed_profile_controller.set_target(self.linear_speed_profile_controller.target_heading)
        w_target = self.angular_speed_profile_controller.evaluate(self.delta_t, self.get_pose()[2])

        Force = self.linear_speed_controller.evaluate(self.delta_t, v_target, self.cart.v)
        Torque = self.angular_speed_controller.evaluate(self.delta_t, w_target, self.cart.w)
        self.cart.evaluate(self.delta_t, Force, Torque)

        (x, y, _) = self.get_pose()
        self.plotter.add('t', self.t)
        self.plotter.add('x', x)
        self.plotter.add('y', y)
        self.plotter.add('x_target', self.target[0])
        self.plotter.add('y_target', self.target[1])
        self.plotter.add('v_target', v_target)
        self.plotter.add('w_target', w_target)
        self.plotter.add('v', self.cart.v)
        self.plotter.add('w', self.cart.w)

        if self.t > 5:
            self.plotter.plot(['t', 'time'],
                              [['v_target', 'V Target'], ['v', 'V']])
            self.plotter.plot(['t', 'time'],
                              [['w_target', 'W Target'], ['w', 'W']])
            self.plotter.show()
            return False

        return True

    def get_pose(self):
        return self.cart.get_pose()

    def get_speed(self):
        return self.cart.v, self.cart.w


if __name__ == '__main__':
    cart_robot = Cart2DRobot()
    app = QApplication(sys.argv)
    ex = CartWindow(cart_robot)
    sys.exit(app.exec_())
