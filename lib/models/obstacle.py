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

