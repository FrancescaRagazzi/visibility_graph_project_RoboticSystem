
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

# obstacles = [
#     Obstacle([(190, 260), (100, 350), (290, 440)]),
#     Obstacle([(450, 160), (540, 90), (840, 340), (740, 540)]),
#     Obstacle([(700,50), (770,100), (800,20)])
# ]


obstacles = [
    Obstacle([(400,100),(500,200), (200,200), (160,80)]),
    Obstacle([(800,200), (700,100),(800,100)]),
    Obstacle([(400,400), (600,200), (600,300)])
]



for obstacle in obstacles:
    env.add_obstacle(obstacle)


# visibility = VisibilityGraph(env, (100, 500), (900, 50))

visibility = VisibilityGraph(env, (600, 500), (270, 30))


visibility.find_paths()

for point in visibility.fastest_path:
    print(point)

scale_factor = 0.9
scaled_obstacles = []
for obstacle in obstacles:
    scaled_obstacles.append(obstacle.scale(scale_factor))


if __name__ == '__main__':
    app = QApplication(sys.argv)
    visibility.visualize()
    cart_robot = MyCart2D(visibility.fastest_path),
    ex = MyCartWindow(cart_robot)
    ex.set_obstacles(scaled_obstacles)

    sys.exit(app.exec_())