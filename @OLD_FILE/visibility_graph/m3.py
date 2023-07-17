import sys
import math

from pathlib import Path

CURRENT_POSITION = Path(__file__).parent
sys.path.append(f"{CURRENT_POSITION}/../")

from lib.models.cart2d import Cart2D
from lib.models.robot import RoboticSystem
from lib.controllers.standard import PIDSat
from lib.controllers.control2d import Polar2DController, Path2D
from lib.gui.my_gui import MyCartWindow
from lib.data.plot import DataPlotter
from lib.models.cart2d import TwoWheelsCart2DEncodersOdometry

from PyQt5.QtWidgets import QApplication


class Cart2DRobot(RoboticSystem):

    def __init__(self):

        super().__init__(1e-3) # delta_t = 1e-3
        # Mass = 20kg
        # radius = 15cm
        # friction = 0.8
        # Traction Wheels, radius = 2.5cm, wheelbase = 20cm
        # Sensing Wheels, radius = 2cm, wheelbase = 24cm
        # Encoder resolution = 4000 ticks/revolution
        self.cart = TwoWheelsCart2DEncodersOdometry(20, 0.15, 0.8, 0.8,
                                                    0.025, 0.025, 0.2,
                                                    0.02, 0.02, 0.24, 2*math.pi/4000.0)

        # 5 Nm of max torque, antiwindup
        
        self.left_controller = PIDSat(8.0, 3.0, 0.0, 5, True)
        self.right_controller = PIDSat(8.0, 3.0, 0.0, 5, True)

        self.polar_controller = Polar2DController(2.5, 2, 2.0 , 2)
        
        self.path_controller = Path2D(0.2, 0.5, 0.5, 0.01)  # tolerance 1cm
        # self.path_controller.set_path([(0.5, 0.2),
        #                                (0.5, 0.4),
        #                                (0.3, 0.2)]) #elenco punti della traiettoria


        self.path_controller.set_path([(0.45,0.5-0.16),(0.54,0.5-0.09), (0.9, 0.5 - 0.05)])
      
        (x, y, _) = self.get_pose()
        self.path_controller.start((x, y)) # avviamo il path a partire dalla posizione corrente del robot
        
        self.plotter = DataPlotter()

    def run(self):
        pose = self.get_pose()
        target = self.path_controller.evaluate(self.delta_t, pose)

        self.plotter.add('x', pose[0])#effettivo pos 
        self.plotter.add('y', pose[1])
        self.plotter.add('x_t', self.path_controller.x_current)
        self.plotter.add('y_t', self.path_controller.y_current)
        if target is None: #target raggiunto
            self.plotter.plot(['x', 'x'], [['y', 'y']])
            self.plotter.plot(['x_t', 'x_t'], [['y_t', 'y_t']])
            self.plotter.show()
            return False

        
        (x_target, y_target) = target

        # polar control
        (v_target, w_target) = self.polar_controller.evaluate(self.delta_t, x_target, y_target, self.get_pose())
        vref_l = v_target - w_target * self.cart.encoder_wheelbase / 2.0
        vref_r = v_target + w_target * self.cart.encoder_wheelbase / 2.0
        (vl, vr) = self.cart.get_wheel_speed()

        # speed control (left, right)
        Tleft = self.left_controller.evaluate(self.delta_t, vref_l, vl)
        Tright = self.right_controller.evaluate(self.delta_t, vref_r, vr)

        # robot model
        self.cart.evaluate(self.delta_t, Tleft, Tright)

        return True

    def get_pose(self):
        return self.cart.get_pose()

    def get_speed(self):
        return self.cart.v, self.cart.w




if __name__ == '__main__':
    cart_robot = Cart2DRobot()
    app = QApplication(sys.argv)
    ex = MyCartWindow(cart_robot)
    sys.exit(app.exec_())
