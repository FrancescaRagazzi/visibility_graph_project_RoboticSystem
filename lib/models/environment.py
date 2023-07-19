class Environment:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.obstacles = []

    def add_obstacle(self, obstacle):
        self.obstacles.append(obstacle)

    def is_valid_point(self, point):
        x, y = point
        if 0 <= x < self.width and 0 <= y < self.height and not self.is_in_obstacle(point):
            return True
        return False

    def is_in_obstacle(self, point):
        for obstacle in self.obstacles:
            if obstacle.is_inside(point):
                return True
        return False


