import math
import os

import numpy as np
import pandas as pd

from drone_map import DroneMap
from lidar_point import LidarPoint
from lidar_vector import LidarVector


def generate_lidar_points(m_csv: str, lp_csv: str, fp_csv: str, num_points=500) -> pd.DataFrame:
    if not (os.path.exists(m_csv) and os.path.exists(fp_csv)):
        return pd.DataFrame(data=[])

    angles, distances = [], []
    walls = pd.read_csv(m_csv, names=['xstart', 'ystart', 'xend', 'yend'])

    pts = DroneMap(
        flight_path_csv=fp_csv,
        lidar_points_csv=lp_csv
    ).get_drone_positions()

    for i in range(len(pts)):
        pt = pts.iloc[i]
        angles.append(i)
        distances.append(num_points)
        for theta in np.linspace(start=0, stop=360, num=num_points, endpoint=False):
            min_distance = get_distance_from_nearest_wall(
                wall_candidates=walls,
                pt=LidarPoint(pt.x, pt.y),
                theta=theta
            )

            angles.append(round(360 - theta, 4))
            distances.append(min_distance)

    lp = pd.DataFrame(
        data=dict(
            angle=np.array(angles),
            distance=np.array(distances).astype(int)
        )
    )

    if os.path.exists(os.path.dirname(lp_csv)):
        lp.to_csv(lp_csv, index=False, header=False)

    return lp


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


if __name__ == '__main__':
    lp = generate_lidar_points(
        m_csv='./.cache/Mapping.csv',
        fp_csv='./.cache/FlightPath.csv',
        lp_csv='./.cache/lp.csv'
    )

    print(lp)
