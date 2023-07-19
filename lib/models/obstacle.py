class Obstacle:
    def __init__(self, vertices):
        self.vertices = vertices
    
    def get_vertices(self):
        return self.vertices
    
    def scale(self, scale_factor):
        scaled_vertices = scale_polygon(self.vertices, scale_factor)
        return Obstacle(scaled_vertices)

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

def scale_polygon(vertices, scale_factor):
    # Trova il centro del poligono originale
    center_x = sum(x for x, _ in vertices) / len(vertices)
    center_y = sum(y for _, y in vertices) / len(vertices)

    # Trasla il poligono in modo che il centro sia all'origine (0, 0)
    translated_vertices = [(x - center_x, y - center_y) for x, y in vertices]

    # Applica il fattore di scala ai vertici del poligono traslato
    scaled_vertices = [(x * scale_factor, y * scale_factor) for x, y in translated_vertices]

    # Trasla nuovamente il poligono scalato al centro originale
    scaled_vertices = [(x + center_x, y + center_y) for x, y in scaled_vertices]

    return scaled_vertices
