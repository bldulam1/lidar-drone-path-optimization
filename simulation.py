import os

import numpy as np
import pandas as pd

from drone_map import DroneMap
from lidar_point import LidarPoint


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


if __name__ == '__main__':
    lp = generate_lidar_points(
        m_csv='./.cache/Mapping.csv',
        fp_csv='./.cache/FlightPath.csv',
        lp_csv='./.cache/lp.csv'
    )

    print(lp)
