import math

import pandas as pd

from lidar_point import LidarPoint
from lidar_vector import LidarVector


def get_distance_from_nearest_wall(wall_candidates: pd.DataFrame, pt: LidarPoint, theta: float) -> float:
    min_distance = None
    if math.cos(math.radians(theta)) > 0:
        wall_candidates = wall_candidates[(wall_candidates.xstart > pt.x) | (wall_candidates.xend > pt.x)]
    else:
        wall_candidates = wall_candidates[(wall_candidates.xstart <= pt.x) | (wall_candidates.xend <= pt.x)]

    if math.sin(math.radians(theta)) > 0:
        wall_candidates = wall_candidates[(wall_candidates.ystart > pt.y) | (wall_candidates.yend > pt.y)]
    else:
        wall_candidates = wall_candidates[(wall_candidates.ystart <= pt.y) | (wall_candidates.yend <= pt.y)]

    for _, wall in wall_candidates.iterrows():
        wall_start = LidarPoint(wall.xstart, wall.ystart)
        wall_end = LidarPoint(wall.xend, wall.yend)
        theta1 = math.degrees(LidarVector(pt, wall_start).direction)
        theta2 = math.degrees(LidarVector(pt, wall_end).direction)

        if theta1 < 0:
            theta1 += 360
        if theta2 < 0:
            theta2 += 360

        if theta1 > theta2:
            theta1, theta2 = theta2, theta1
            wall_start, wall_end = wall_end, wall_start

        if theta2 - theta1 > 180:
            theta1, theta2 = theta2 - 360, theta1
            wall_start, wall_end = wall_end, wall_start

        if theta1 <= theta <= theta2 or theta1 <= theta - 360 <= theta2:
            wall_v = LidarVector(wall_start, wall_end)
            poi = wall_v.get_intersection_point_with(pt, math.radians(theta))

            distance = LidarVector(pt, poi).distance
            if min_distance is None or distance < min_distance:
                min_distance = distance

    return min_distance
