
import sys
from pathlib import Path
import matplotlib.pyplot as plt


CURRENT_POSITION = Path(__file__).parent
sys.path.append(f"{CURRENT_POSITION}/../")

#import new classes 
from lib.models.environment import *
from lib.models.obstacle import *
from lib.models.visibility_graph import *
from lib.models.visibility_graph_robot2D_2wheels import *
from lib.models.obstacle import scale_polygon



env = Environment(1000, 600)

obstacles = [ # 
    Obstacle([(190, 260), (100, 350), (290, 440)]),
    Obstacle([(450, 160), (540, 90), (840, 340), (740, 540)]),
    Obstacle([(700,50), (770,100), (800,20)])
]


for obstacle in obstacles:
    env.add_obstacle(obstacle)

robot = VisibilityGraph(env, (100, 500), (900, 50))
robot.find_paths()

for point in robot.fastest_path:
    print(point)

scale_factor = 0.8
scaled_obstacles = []
for obstacle in obstacles:
    scaled_obstacles.append(obstacle.scale(scale_factor))


if __name__ == '__main__':
    app = QApplication(sys.argv)
    robot.visualize()
    cart_robot = Cart2DRobot(robot.fastest_path)
    ex = MyCartWindow(cart_robot)
    ex.set_obstacles(scaled_obstacles)

    sys.exit(app.exec_())




