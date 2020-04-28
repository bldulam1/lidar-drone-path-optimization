import math

from lidar_point import LidarPoint


class LidarVector:
    def __init__(self, start: LidarPoint, end: LidarPoint):
        self.start, self.end = start, end
        self.num_points = 2

        dy = self.end.y - self.start.y
        dx = self.end.x - self.start.x
        self.distance = math.sqrt(dx * dx + dy * dy)
        self.direction = math.atan2(dy, dx)

    def __repr__(self):
        return "{} {:.4f}∠{:.2f}".format(self.start, self.distance, math.degrees(self.direction))

    def __str__(self):
        return self.__repr__()
