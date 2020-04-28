from lidar_point import LidarPoint
from lidar_vector import LidarVector


class WallCorner:
    def __init__(self, cc: LidarPoint, cv: LidarPoint, ch: LidarPoint):
        self.corner_center = cc
        self.corner_vertical = cv
        self.corner_horizontal = ch

    def get_walls(self):
        if self.corner_center.y > self.corner_vertical.y:
            wall_vertical = LidarVector(self.corner_vertical, self.corner_center)
        else:
            wall_vertical = LidarVector(self.corner_center, self.corner_vertical)

        if self.corner_center.x > self.corner_horizontal.x:
            wall_horizontal = LidarVector(self.corner_horizontal, self.corner_center)
        else:
            wall_horizontal = LidarVector(self.corner_center, self.corner_horizontal)

        return wall_vertical, wall_horizontal

    def __repr__(self):
        return self.corner_center.__repr__()

    def __str__(self):
        return self.__repr__()
