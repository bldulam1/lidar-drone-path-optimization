import math

import numpy as np

from lidar_point import LidarPoint


class LidarVector:
    def __init__(self, start: LidarPoint, end: LidarPoint):
        self.start, self.end = start, end
        self.num_points = 2

        dy = self.end.y - self.start.y
        dx = self.end.x - self.start.x
        self.distance = math.sqrt(dx * dx + dy * dy)
        self.direction = math.atan2(dy, dx)

    def get_abc(self) -> (float, float, float):
        """
        calculates the a,b,c parameters of the line ax + by = c
        :return: Tuple[float, float, float]
        """
        dy = self.end.y - self.start.y
        dx = self.end.x - self.start.x

        a, b = dy, -dx
        c = a * self.start.x + b * self.start.y
        return a, b, c

    def get_intersection_point_with(self, point: LidarPoint, angle: float) -> LidarPoint:
        """
        Calculates the intersection between two this line and the line formed by a point and angle
        :param point: LidarPoint
        :param angle: float in radians
        :return: LidarPoint
        """
        point2 = LidarPoint(x=point.x + math.cos(angle), y=point.y + math.sin(angle))
        vector = LidarVector(point, point2)

        a1, b1, c1 = vector.get_abc()
        a2, b2, c2 = self.get_abc()

        if a1 == 0 and a2 == 0:
            return None
        elif b1 == 0 and b2 == 0:
            return None
        else:
            a = np.array([[a1, b1], [a2, b2]])
            b = np.array([c1, c2])
            intersection = np.linalg.solve(a, b)

        return LidarPoint(x=intersection[0], y=intersection[1])

    def get_distance_from(self, point: LidarPoint) -> float:
        a, b, c = self.get_abc()
        return abs(a * point.x + b * point.y + c) / (a ** 2 + b ** 2) ** 0.5

    def __repr__(self):
        return "{} {:.4f}∠{:.2f}".format(self.start, self.distance, math.degrees(self.direction))

    def __str__(self):
        return self.__repr__()
