import sys

from pathlib import Path

CURRENT_POSITION = Path(__file__).parent
sys.path.append(f"{CURRENT_POSITION}/../")

from lib.models.cart2d import Cart2D
from lib.models.robot import RoboticSystem
from lib.controllers.standard import PIDSat
from lib.controllers.control2d import Polar2DController, StraightLine2DMotion
from lib.gui.gui_2d import CartWindow
from lib.data.plot import DataPlotter

from PyQt5.QtWidgets import QApplication

import math
import heapq
from lib.models.robot import RoboticSystem


class VisibilityPathPlanner:
    def __init__(self, environment, start, goal):
        self.environment = environment
        self.start = start
        self.goal = goal
        self.graph = self.build_visibility_graph()

    def build_visibility_graph(self):
        points = [self.start, self.goal]
        for obstacle in self.environment.obstacles:
            points.extend(obstacle.vertices)

        graph = {}
        for i in range(len(points)):
            for j in range(i + 1, len(points)):
                point1 = points[i]
                point2 = points[j]
                if self.can_connect(point1, point2):
                    if point1 not in graph:
                        graph[point1] = []
                    if point2 not in graph:
                        graph[point2] = []
                    graph[point1].append(point2)
                    graph[point2].append(point1)

        return graph

    def can_connect(self, point1, point2):
        if not self.environment.is_valid_point(point1) or not self.environment.is_valid_point(point2):
            return False

        if self.environment.is_in_obstacle(point1) or self.environment.is_in_obstacle(point2):
            return False

        x1, y1 = point1
        x2, y2 = point2
        dx = x2 - x1
        dy = y2 - y1
        distance = math.sqrt(dx**2 + dy**2)

        step = 0.1
        num_steps = int(distance / step)

        for i in range(num_steps + 1):
            t = i / num_steps
            x = x1 + t * dx
            y = y1 + t * dy
            if self.environment.is_in_obstacle((x, y)):
                return False

        return True

    def find_fastest_path(self):
        distances = {point: math.inf for point in self.graph}
        distances[self.start] = 0
        previous = {point: None for point in self.graph}

        queue = [(0, self.start)]
        while queue:
            dist, current_node = heapq.heappop(queue)
            if current_node == self.goal:
                break
            if dist > distances[current_node]:
                continue
            for neighbor in self.graph[current_node]:
                new_dist = distances[current_node] + self.distance(current_node, neighbor)
                if new_dist < distances[neighbor]:
                    distances[neighbor] = new_dist
                    previous[neighbor] = current_node
                    heapq.heappush(queue, (new_dist, neighbor))

        # Costruzione del percorso dal goal al punto di partenza
        path = []
        current_node = self.goal
        while current_node != self.start:
            path.append(current_node)
            current_node = previous[current_node]
        path.append(self.start)
        path.reverse()

        return path

    def distance(self, point1, point2):
        x1, y1 = point1
        x2, y2 = point2
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)


class Cart2DRobot(RoboticSystem):
    def __init__(self, path):
        super().__init__(1e-3)  # delta_t = 1e-3
        self.path = path
        self.current_index = 0

    def run(self):
        if self.current_index < len(self.path):
            target_x, target_y = self.path[self.current_index]
            current_x, current_y, current_theta = self.get_pose()
            dx = target_x - current_x
            dy = target_y - current_y
            distance = math.sqrt(dx**2 + dy**2)
            speed = 0.5  # Velocità costante del robot (puoi personalizzarla)
            if distance > speed * self.delta_t:
                direction = math.atan2(dy, dx)
                target_theta = direction
                diff_theta = target_theta - current_theta
                while diff_theta < -math.pi:
                    diff_theta += 2 * math.pi
                while diff_theta > math.pi:
                    diff_theta -= 2 * math.pi
                angular_speed = diff_theta / self.delta_t
                # Inserisci qui la logica per controllare il robot e guidarlo verso il target (target_x, target_y)
                # Ad esempio, potresti impostare la velocità delle ruote per raggiungere il target
                # Esempio: self.cart.set_wheel_speed(left_speed, right_speed)
            self.current_index += 1
        return True

    def get_pose(self):
        if self.current_index < len(self.path):
            return self.path[self.current_index] + (0,)  # Assume un'orientazione fissa di 0
        return None

    def get_speed(self):
        if self.current_index < len(self.path):
            target_x, target_y = self.path[self.current_index]
            current_x, current_y, _ = self.get_pose()
            dx = target_x - current_x
            dy = target_y - current_y
            distance = math.sqrt(dx**2 + dy**2)
            speed = distance / self.delta_t
            return speed, 0  # Velocità angolare fissa di 0
        return 0, 0


if __name__ == '__main__':
    # Creazione dell'ambiente e degli ostacoli
    env = Environment(10, 10)
    obstacle1 = Obstacle([(1, 2), (3, 2), (2, 4)])
    obstacle2 = Obstacle([(3, 4), (6, 2), (8, 3), (7, 5)])
    obstacle3 = Obstacle([(3, 6), (4, 8), (5, 6), (4, 5)])
    env.add_obstacle(obstacle1)
    env.add_obstacle(obstacle2)
    env.add_obstacle(obstacle3)

    # Definizione del robot
    robot = Robot(env, (1, 1), (9, 9))
    robot.find_paths()

    # Ottenimento della traiettoria più veloce e dei punti intermedi
    fastest_path = robot.fastest_path
    intermediate_points = fastest_path[1:-1]

    # Creazione del robot cartesiano 2D
    cart_robot = Cart2DRobot(intermediate_points)

    # Esecuzione della simulazione
    app = QApplication([])
    cart_window = CartWindow(cart_robot)
    sys.exit(app.exec_())
