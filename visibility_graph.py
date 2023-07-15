import matplotlib.pyplot as plt
import math
import heapq


class Environment:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.obstacles = []

    def add_obstacle(self, obstacle):
        self.obstacles.append(obstacle)
        return

    def is_valid_point(self, point):
        x, y = point
        if (self.width > x >= 0 and self.height > y >= 0 and not self.is_in_obstacle(point)):
            return True
        return False


    def is_in_obstacle(self, point):
        for obstacle in self.obstacles:
            if obstacle.is_inside(point):
                return True
        return False


class Obstacle:
    def __init__(self, vertices):
        self.vertices = vertices

    def is_inside(self, point):
        x, y = point
        num_vertices = len(self.vertices)
        inside = False
        p1x, p1y = self.vertices[0]
        for i in range(num_vertices + 1):
            p2x, p2y = self.vertices[i % num_vertices]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside


class Robot:
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

        return edges

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

        self.paths = self.get_possible_paths(graph)
        self.find_fastest_path(graph)

    def get_possible_paths(self, graph):
        stack = [(self.start, [self.start])]
        possible_paths = []
        while stack:
            current_node, path = stack.pop()
            if current_node == self.goal:
                possible_paths.append(path)
            else:
                for neighbor in graph[current_node]:
                    if neighbor not in path and not self.environment.is_in_obstacle(neighbor):
                        stack.append((neighbor, path + [neighbor]))

        return possible_paths

    def find_fastest_path(self, graph):
        distances = {point: math.inf for point in graph}
        distances[self.start] = 0
        previous = {point: None for point in graph}

        queue = [(0, self.start)]
        while queue:
            dist, current_node = heapq.heappop(queue)
            if current_node == self.goal:
                break
            if dist > distances[current_node]:
                continue
            for neighbor in graph[current_node]:
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

        self.fastest_path = path

    def distance(self, point1, point2):
        x1, y1 = point1
        x2, y2 = point2
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def visualize(self):
        if not self.paths:
            print("Calcola prima i percorsi!")
            return

        plt.figure()
        for obstacle in self.environment.obstacles:
            vertices = obstacle.vertices + [obstacle.vertices[0]]
            xs, ys = zip(*vertices)
            plt.fill(xs, ys, '#023047')

        # Visualizzazione delle traiettorie possibili
        for path in self.paths:
            plt.plot(*zip(*path), '#ffb703', linewidth=0.5, linestyle='dashed')

        # Visualizzazione del percorso più veloce
        plt.plot(*zip(*self.fastest_path), '#e63946', linewidth=3)

        plt.plot(self.start[0], self.start[1], 'bo', label='Punto di partenza', color = '#d62828')
        plt.plot(self.goal[0], self.goal[1], 'ro', label='Punto di arrivo', color='#003049')

        plt.xlim(0, self.environment.width)
        plt.ylim(0, self.environment.height)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.legend()
        plt.show()


# Esempio di utilizzo
env = Environment(10, 10)



# Aggiunta degli ostacoli nel grafico
obstacle1 = Obstacle([(1, 2), (3, 2), (2, 4)]) 
obstacle2 = Obstacle([(3, 4), (6, 2), (8,3), (7, 5)])  
obstacle3 = Obstacle([(3, 6), (4, 8), (5, 6), (4, 5)])  


env.add_obstacle(obstacle1)
env.add_obstacle(obstacle2)
env.add_obstacle(obstacle3)




robot = Robot(env, (1, 1), (9, 9))
robot.find_paths()
robot.visualize()
