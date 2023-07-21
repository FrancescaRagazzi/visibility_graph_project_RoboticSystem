
import sys
from pathlib import Path
import math
import heapq
import matplotlib.pyplot as plt
from queue import PriorityQueue

CURRENT_POSITION = Path(__file__).parent
sys.path.append(f"{CURRENT_POSITION}/../")

#import new classes 
from lib.models.environment import Environment
from lib.models.obstacle import Obstacle
from robot2D_2wheels_visibility_graph import *

from lib.models.obstacle import scale_polygon


class VisibilityGraph:
    def __init__(self, environment, start, goal):
        self.environment = environment
        self.start = start
        self.goal = goal
        self.paths = []
        self.fastest_path = []

    def visibility_graph(self):
        points = [self.start, self.goal]
        for obstacle in self.environment.obstacles:
            points.extend(obstacle.vertices)

        edges = []
        for i in range(len(points)):
            for j in range(i + 1, len(points)):
                point1 = points[i]
                point2 = points[j]
                if self.can_connect(point1, point2):
                    edges.append((point1, point2))

        # Aggiungi gli archi tra i vertici dell'ostacolo stesso
        for obstacle in self.environment.obstacles:
            vertices = obstacle.vertices
            num_vertices = len(vertices)
            for i in range(num_vertices):
                point1 = vertices[i]
                point2 = vertices[(i + 1) % num_vertices]
                edges.append((point1, point2))

        return edges


    def is_inside_obstacle(self, point1):
        for obstacle in self.environment.obstacles:
            if obstacle.is_inside(point1):
                return True
        return False

    def can_connect(self, point1, point2):
        if not self.environment.is_valid_point(point1) or not self.environment.is_valid_point(point2):
            return False

        if self.environment.is_in_obstacle(point1) or self.environment.is_in_obstacle(point2):
            return False

        x1, y1 = point1
        x2, y2 = point2
        dx = x2 - x1
        dy = y2 - y1
        distance = math.sqrt(dx ** 2 + dy ** 2)

        step = 0.1
        num_steps = int(distance / step)

        for i in range(num_steps + 1):
            t = i / num_steps
            x = x1 + t * dx
            y = y1 + t * dy
            if self.environment.is_in_obstacle((x, y)):
                return False

        return True

    def find_paths(self):
        edges = self.visibility_graph()
        graph = {}
        for edge in edges:
            point1, point2 = edge
            if point1 not in graph:
                graph[point1] = []
            if point2 not in graph:
                graph[point2] = []
            graph[point1].append(point2)
            graph[point2].append(point1)

        stack = [(self.start, [self.start])]
        possible_paths = []

        while stack:
            current_node, path = stack.pop()

            if current_node == self.goal:
                possible_paths.append(path)
                continue

            for neighbor in graph[current_node]:
                if neighbor not in path and not self.environment.is_in_obstacle(neighbor):
                    stack.append((neighbor, path + [neighbor]))

        self.paths = possible_paths
        self.find_fastest_path(graph)

    def find_fastest_path(self, graph):
        distances = {point: math.inf for point in graph}
        distances[self.start] = 0
        previous = {point: None for point in graph}

        queue = PriorityQueue()
        queue.put((0, self.start))

        while not queue.empty():
            dist, current_node = queue.get()
            if current_node == self.goal:
                break
            if dist > distances[current_node]:
                continue
            for neighbor in graph[current_node]:
                new_dist = distances[current_node] + self.distance(current_node, neighbor)
                if new_dist < distances[neighbor]:
                    distances[neighbor] = new_dist
                    previous[neighbor] = current_node
                    queue.put((new_dist, neighbor))

        # Costruzione del percorso dal goal al punto di partenza
        path = []
        current_node = self.goal
        while current_node != self.start:
            path.append(current_node)
            current_node = previous[current_node]
        path.append(self.start)
        path.reverse()

        self.fastest_path = path

    
    def distance(self, point1, point2):
        x1, y1 = point1
        x2, y2 = point2
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


    def visualize(self):
        if not self.paths:
            print("Calcola prima i percorsi!")
            return

        plt.figure(figsize=(10, 6))
        for obstacle in self.environment.obstacles:
            vertices = obstacle.vertices + [obstacle.vertices[0]]
            xs, ys = zip(*vertices)
            plt.fill(xs, ys, "#023047")

        # Visualizzazione delle traiettorie possibili
        for path in self.paths:
            if self.start in path:
                start_idx = path.index(self.start)
                plt.plot(*zip(*path[start_idx:]), "#ffb703", linewidth=0.5, linestyle="dashed")

        # Visualizzazione del percorso più veloce
        plt.plot(*zip(*self.fastest_path), "#e63946", linewidth=3)

        plt.plot(self.start[0], self.start[1], "bo", label="Punto di partenza", color="#d62828")
        plt.plot(self.goal[0], self.goal[1], "ro", label="Punto di arrivo", color="#003049")

        plt.xlim(0, self.environment.width)
        plt.ylim(self.environment.height, 0)  # Inverti l'orientamento dell'asse y
        plt.gca().set_aspect("equal", adjustable="box")
        plt.legend()
        plt.show()


def get_direction_change_points(fastest_path):
    direction_change_points = []
    if len(fastest_path) < 3:
        return direction_change_points

    for i in range(1, len(fastest_path) - 1):
        x1, y1 = fastest_path[i - 1]
        x2, y2 = fastest_path[i]
        x3, y3 = fastest_path[i + 1]

        if (x2 - x1, y2 - y1) != (x3 - x2, y3 - y2):
            direction_change_points.append(fastest_path[i])

    direction_change_points.append(robot.goal)

    return direction_change_points


env = Environment(1000, 600)

obstacles = [
    Obstacle([(190, 260), (100, 350), (290, 440)]),
    Obstacle([(450, 160), (540, 90), (840, 340), (740, 540)]),
    Obstacle([(700,50), (770,100), (800,20)])
]


for obstacle in obstacles:
    env.add_obstacle(obstacle)

robot = VisibilityGraph(env, (100, 500), (900, 50))
robot.find_paths()


direction_change_points = get_direction_change_points(robot.fastest_path)

print("Punti di cambio direzione:")
for point in direction_change_points:
    print(point)



scale_factor = 0.8
scaled_obstacles = []
for obstacle in obstacles:
    scaled_obstacles.append(obstacle.scale(scale_factor))


if __name__ == '__main__':
    app = QApplication(sys.argv)

    # Visualizza il grafico del robot prima dell'avvio dell'app
    robot.visualize()

    # Crea e avvia l'app
    cart_robot = Cart2DRobot(direction_change_points)
    ex = MyCartWindow(cart_robot)
    ex.set_obstacles(scaled_obstacles)

    sys.exit(app.exec_())




