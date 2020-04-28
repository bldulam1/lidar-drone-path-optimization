import math
from enum import Enum

from lidar_point import LidarPoint
from lidar_vector import LidarVector


class WallType(Enum):
    VERTICAL = 'vertical'
    HORIZONTAL = 'horizontal'


class WallOpening:
    def __init__(self, wall1: LidarVector, wall2: LidarVector):
        if abs(math.degrees(wall1.direction) - 90) < 10:
            self.wall_type = WallType.VERTICAL
            if wall1.start.x < wall2.start.x:
                self.wall1, self.wall2 = wall1, wall2
            else:
                self.wall1, self.wall2 = wall2, wall1
        else:
            self.wall_type = WallType.HORIZONTAL
            if wall1.start.y < wall2.start.y:
                self.wall1, self.wall2 = wall1, wall2
            else:
                self.wall1, self.wall2 = wall2, wall1

    def get_mid_points(self, min_wall_distance=300):
        x_s = (self.wall1.start.x + self.wall2.start.x) / 2
        y_s = (self.wall1.start.y + self.wall2.start.y) / 2
        x_t = (self.wall1.end.x + self.wall2.end.x) / 2
        y_t = (self.wall1.end.y + self.wall2.end.y) / 2

        if self.wall_type == WallType.HORIZONTAL:
            x_s -= min_wall_distance
            x_t += min_wall_distance
        else:
            y_s -= min_wall_distance
            y_t += min_wall_distance

        return LidarVector(
            start=LidarPoint(x_s, y_s),
            end=LidarPoint(x_t, y_t)
        )

    def __repr__(self):
        if self.wall_type == WallType.VERTICAL:
            return str(self.wall1) + " V " + str(self.wall2)
        else:
            return str(self.wall1) + " H " + str(self.wall2)

    def __str__(self):
        return self.__repr__()
