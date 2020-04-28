import csv
import math

import pandas as pd


class DroneMap:
    def __init__(self, lidar_points_csv: str, flight_path_csv: str):
        self.lidar_points_csv = lidar_points_csv
        self.flight_path_csv = flight_path_csv

        self.drone_positions = None
        self.points = None

    def get_drone_positions(self) -> pd.DataFrame:
        if self.drone_positions is None:
            x, y = [], []
            with open(file=self.flight_path_csv) as f:
                i = 0
                for cf_row in csv.DictReader(f, fieldnames=['x', 'y']):
                    if i % 2:
                        x.append(float(cf_row['x']) * 1e3)
                        y.append(float(cf_row['y']) * 1e3)
                    i += 1

            self.drone_positions = pd.DataFrame(
                data=list(zip(x, y)),
                columns=['x', 'y']
            )
        return self.drone_positions

    def get_all_points(self, max_sweep=-1) -> (pd.DataFrame, pd.DataFrame):
        pos = self.get_drone_positions()
        if self.points is None:
            x, y = [], []

            with open(file=self.lidar_points_csv) as f:
                i = next_row_header = 0
                pos_x, pos_y = 0, 0
                for cf_row in csv.DictReader(f, fieldnames=['angle', 'distance']):
                    if i == next_row_header:
                        cf_sweep = int(float(cf_row['angle']))

                        if cf_sweep > max_sweep > 0:
                            break
                        pos_x, pos_y = pos.iloc[cf_sweep].x, pos.iloc[cf_sweep].y
                        next_row_header = i + int(cf_row['distance']) + 1
                    else:
                        theta, r = cf_row['angle'], cf_row['distance']
                        theta = math.radians(float(theta))
                        curr_x = float(r) * math.cos(theta) + pos_x
                        curr_y = float(r) * math.sin(theta) + pos_y
                        x.append(curr_x)
                        y.append(curr_y)
                    i += 1

            self.points = pd.DataFrame(
                data=list(zip(x, y)),
                columns=['x', 'y']
            )
            self.points = self.points.drop_duplicates(keep='last')
            self.points = self.points.sort_values(by=['x', 'y'])

        return self.points


if __name__ == '__main__':
    dm = DroneMap(
        lidar_points_csv='./.cache/LIDARPoints.csv',
        flight_path_csv='./.cache/FlightPath.csv'
    )

    # print(dm.get_drone_positions())
    print(dm.get_all_points())
