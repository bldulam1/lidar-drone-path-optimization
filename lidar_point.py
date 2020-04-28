class LidarPoint:
    def __init__(self, x, y):
        self.x, self.y = x, y
        self.neighbors = set()

        self.opening_pair = None

    def add_neighbor(self, neighbor):
        self.neighbors.add(neighbor)

    def add_opening_pair(self, point):
        self.opening_pair = point

    def __repr__(self):
        p = (int(self.x), int(self.y))
        return str(p)

    def __str__(self):
        return self.__repr__()

    def __eq__(self, other):
        return self.__repr__() == other.__repr__()

    def __hash__(self):
        return hash(self.__repr__())
