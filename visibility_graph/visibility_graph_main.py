
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

obstacles = [
    Obstacle([(100, 350),(290, 440),(190, 260) ]),
    Obstacle([(450, 160),(740, 540), (840, 340), (540, 90)]),
    Obstacle([(700,50), (770,100), (800,20)])
]

for obstacle in obstacles:
    env.add_obstacle(obstacle)

visibility = VisibilityGraph(env, (100, 500), (900, 50)) 


visibility.find_fastest_path()

for point in visibility.fastest_path:
    print(point)

scale_factor = 0.75
scaled_obstacles = []
for obstacle in obstacles:
    scaled_obstacles.append(obstacle.scale(scale_factor))


if __name__ == '__main__':
    app = QApplication(sys.argv)
    visibility.visualize()
    cart_robot = MyCart2D(visibility.fastest_path)
    ex = MyCartWindow(cart_robot)
    ex.set_obstacles(scaled_obstacles)

    sys.exit(app.exec_())












#second
# obstacles = [
#     Obstacle([(250, 250), (400, 250), (400, 400),(250,400)]),
#     Obstacle([(430, 110), (700, 150), (600, 350)]),
#     Obstacle([(600,550), (800,550), (700,350)])
# ]

# for obstacle in obstacles:
#     env.add_obstacle(obstacle)

# visibility = VisibilityGraph(env, (100, 500), (800, 50))

#third
# obstacles = [
#     Obstacle([(100, 100), (100, 600), (300, 600),(300,100)]),
#     Obstacle([(600,0), (600, 400), (800,400),(800,0)])
# ]

# for obstacle in obstacles:
#     env.add_obstacle(obstacle)

# visibility = VisibilityGraph(env, (10, 400), (950, 500))


