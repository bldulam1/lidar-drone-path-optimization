import csv
import math

import matplotlib.pyplot as plt
import pandas as pd


class DroneMap:
    def __init__(self, lidar_points_csv: str, flight_path_csv: str):
        self.lidar_points_csv = lidar_points_csv
        self.flight_path_csv = flight_path_csv

        self.fig, self.ax = plt.subplots()

        self.drone_positions = None
        self.points = None
        self.corners = None

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
                        theta = -math.radians(float(theta))
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

    def get_all_corners(self):
        if self.corners is None:
            df = self.get_all_points().copy()
            div = 100
            min_pts = 40
            max_radius = 1
            df['x1'], df['y1'] = df.x // div, df.y // div
            can_cor_x, can_cor_y = [], []

            for x in df.x1.unique():
                f_x = df[df.x1 == x]
                if len(f_x) < min_pts:
                    continue
                for y in f_x.y1.unique():
                    f_y = df[df.y1 == y]
                    if len(f_y) < min_pts:
                        continue
                    f_xy = f_y[f_y.x1 == x]

                    df_v = df[(df.x1 == x) & (abs(df.y1 - y) <= max_radius) & (df.y1 != y)]
                    df_h = df[(abs(df.x1 - x) <= max_radius) & (df.y1 == y) & (df.x1 != x)]
                    has_vertical, has_horizontal = len(df_v) > 0, len(df_h) > 0

                    # print(has_vertical, has_horizontal)
                    if has_vertical and has_horizontal:
                        if len(f_xy) > 4:
                            cor_x_mean = f_xy.x.mean()
                            cor_y_mean = f_xy.y.mean()

                            total_pts_len = len(f_xy)
                            right_pts = f_xy[f_xy.x >= cor_x_mean]
                            top_pts = f_xy[f_xy.y >= cor_y_mean]

                            is_top = len(top_pts) > total_pts_len - len(top_pts)
                            is_right = len(right_pts) > total_pts_len - len(right_pts)

                            if is_top:
                                cor_y_mean = top_pts.y.mean()
                            else:
                                cor_y_mean = f_xy[f_xy.y < cor_y_mean].y.mean()

                            if is_right:
                                cor_x_mean = right_pts.x.mean()
                            else:
                                cor_x_mean = f_xy[f_xy.x < cor_x_mean].x.mean()

                        else:
                            cor_x_mean, cor_y_mean = f_xy.x.mean(), f_xy.y.mean()
                        can_cor_x.append(int(cor_x_mean))
                        can_cor_y.append(int(cor_y_mean))

            self.corners = pd.DataFrame(
                data=list(zip(can_cor_x, can_cor_y)),
                columns=['x', 'y']
            ).astype(int)

        return self.corners

    def visualize_lidar_points(self, points=True, corners=True, drone_pos=True):
        if points:
            self.get_all_points().plot(kind='scatter', x='x', y='y', s=1, ax=self.ax)
        if drone_pos:
            self.get_drone_positions().plot(kind='line', x='x', y='y', ax=self.ax)
        if corners:
            self.get_all_corners().plot(kind='scatter', x='x', y='y', color='k', ax=self.ax)
        plt.show()


if __name__ == '__main__':
    dm = DroneMap(
        lidar_points_csv='./.cache/LIDARPoints.csv',
        flight_path_csv='./.cache/FlightPath.csv'
    )

    # print(dm.get_drone_positions())
    # print(dm.get_all_points())
    dm.visualize_lidar_points()
