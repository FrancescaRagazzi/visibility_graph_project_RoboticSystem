import sys

from pathlib import Path

CURRENT_POSITION = Path(__file__).parent
sys.path.append(f"{CURRENT_POSITION}/../../")

from lib.models.cart2d import Cart2D
from lib.models.robot import RoboticSystem
from lib.controllers.standard import PIDSat
from lib.controllers.control2d import Polar2DController
from lib.gui.gui_2d import CartWindow

from PyQt5.QtWidgets import QApplication


class Cart2DRobot(RoboticSystem):

    def __init__(self):
        super().__init__(1e-3)  # delta_t = 1e-3
        # Mass = 1kg
        # radius = 15cm
        # friction = 0.8
        self.cart = Cart2D(1, 0.15, 0.8, 0.8)
        self.linear_speed_controller = PIDSat(10, 3.5, 0, 5)  # 5 newton #controllore velocità lineare
        self.angular_speed_controller = PIDSat(6, 10, 0, 4)  # 4 newton * metro # controllore velocità angolare
        self.polar_controller = Polar2DController(0.5, 2, 2.0, 2)  # v = 2 m/s, w = 2 rad/s #polar controller che controlla l'errore di distanza e orientamento

    def run(self):
        (x_target, y_target) = (0.5, 0.5) #metri in avanti 0 e 20 cm verso l'alto
        (v_target, w_target) = self.polar_controller.evaluate(self.delta_t, x_target, y_target, self.get_pose()) #polar controller ritorna v target e w target da passare ai controllori PID instaziati per v e w

        Force = self.linear_speed_controller.evaluate(self.delta_t, v_target, self.cart.v) #cancoliamo la forza e la torque
        Torque = self.angular_speed_controller.evaluate(self.delta_t, w_target, self.cart.w)

        self.cart.evaluate(self.delta_t, Force, Torque) #passiamo la forza e la torque al robot
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
