import sys

from pathlib import Path

CURRENT_POSITION = Path(__file__).parent
sys.path.append(f"{CURRENT_POSITION}/../../")

from lib.models.cart2d import Cart2D
from lib.models.robot import RoboticSystem
from lib.controllers.standard import PIDSat
from lib.controllers.control2d import Polar2DController, Path2D
from lib.gui.gui_2d import CartWindow
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
        self.polar_controller = Polar2DController(2.5, 2, 2.0, 2)
        self.path_controller = Path2D(0.1, 0.5, 0.5, 0.01)  # tolerance 1cm
        self.path_controller.set_path([(0.5, 0.2),
                                       (0.5, 0.4),
                                       (0.2, 0.2)])
        (x, y, _) = self.get_pose()
        self.path_controller.start((x, y))
        self.plotter = DataPlotter()

    def run(self):
        pose = self.get_pose()
        target = self.path_controller.evaluate(self.delta_t, pose)
        self.plotter.add('x', pose[0])
        self.plotter.add('y', pose[1])
        self.plotter.add('x_t', self.path_controller.x_current)
        self.plotter.add('y_t', self.path_controller.y_current)
        if target is None:
            self.plotter.plot(['x', 'x'], [['y', 'y']])
            self.plotter.plot(['x_t', 'x_t'], [['y_t', 'y_t']])
            self.plotter.show()
            return False
        (x_target, y_target) = target
        (v_target, w_target) = self.polar_controller.evaluate(self.delta_t, x_target, y_target, self.get_pose())
        Force = self.linear_speed_controller.evaluate(self.delta_t, v_target, self.cart.v)
        Torque = self.angular_speed_controller.evaluate(self.delta_t, w_target, self.cart.w)
        self.cart.evaluate(self.delta_t, Force, Torque)
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
