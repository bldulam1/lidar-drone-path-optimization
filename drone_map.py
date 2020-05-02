import csv
import math
import os
from typing import List

import matplotlib.pyplot as plt
import pandas as pd

from dijkstra import Graph
from lidar_point import LidarPoint
from lidar_vector import LidarVector
from wall_corner import WallCorner
from wall_distance import get_distance_from_nearest_wall
from wall_opening import WallOpening


class DroneMap:
    def __init__(self, lidar_points_csv: str, flight_path_csv: str):
        self.lidar_points_csv = lidar_points_csv
        self.flight_path_csv = flight_path_csv

        self.fig, self.ax = plt.subplots()

        self.drone_positions = None
        self.points = None
        self.corners = None
        self.walls = None
        self.wall_openings = None

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
            x, y, sweep_id = [], [], []

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
                        sweep_id.append(cf_sweep)
                    i += 1

            self.points = pd.DataFrame(
                data=list(zip(x, y, sweep_id)),
                columns=['x', 'y', 'i']
            )

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

    def visualize_lidar_points(self, points=True, corners=True, drone_pos=True, by_scan_id=False, plot_interval=1):
        if by_scan_id:
            pts = self.get_all_points()
            pos = self.get_drone_positions()
            for i in range(len(pos)):
                plt.close(self.fig)
                self.fig, self.ax = plt.subplots()
                if points:
                    pts[pts.i == i].plot(kind='scatter', x='x', y='y', s=1, ax=self.ax)
                if drone_pos:
                    pos.iloc[i:i + 1].plot(kind='scatter', x='x', y='y', s=10, color='r', ax=self.ax)
                plt.show(block=False)
                plt.pause(plot_interval)

            plt.close(self.fig)
            self.fig, self.ax = plt.subplots()

        if points:
            self.get_all_points().plot(kind='scatter', x='x', y='y', s=1, ax=self.ax)
        if drone_pos:
            self.get_drone_positions().plot(kind='line', x='x', y='y', color='r', ax=self.ax)
        if corners:
            self.get_all_corners().plot(kind='scatter', x='x', y='y', color='k', ax=self.ax)
        plt.show()

    def get_walls(self, div=100):
        if self.walls is None:
            half_div = div / 2
            df_points = self.get_all_points().copy()
            walls = {}
            for _, corner in self.get_all_corners().iterrows():
                corner = LidarPoint(corner.x, corner.y)
                df = self.get_all_corners().copy()
                df['r'] = df.apply(lambda row: ((row.x - corner.x) ** 2 + (row.y - corner.y) ** 2) ** 0.5, axis=1)
                top_corner = df[(df.y > corner.y) & (abs(df.x - corner.x) < div)].sort_values(by='r')[:1]
                down_corner = df[(df.y < corner.y) & (abs(df.x - corner.x) < div)].sort_values(by='r')[:1]
                left_corner = df[(df.x < corner.x) & (abs(df.y - corner.y) < div)].sort_values(by='r')[:1]
                right_corner = df[(df.x > corner.x) & (abs(df.y - corner.y) < div)].sort_values(by='r')[:1]

                if len(top_corner) and len(down_corner):
                    top_corner = LidarPoint(top_corner.iloc[0].x, top_corner.iloc[0].y)
                    down_corner = LidarPoint(down_corner.iloc[0].x, down_corner.iloc[0].y)

                    top_pts = df_points[
                        (df_points.y < top_corner.y - half_div) &
                        (df_points.y > corner.y + half_div) &
                        (abs(df_points.x - corner.x) < div)
                        ]

                    down_pts = df_points[
                        (df_points.y < corner.y - half_div) &
                        (df_points.y > down_corner.y + half_div) &
                        (abs(df_points.x - corner.x) < div)
                        ]

                    if len(top_pts) > len(down_pts):
                        v_corner = LidarPoint(top_corner.x, top_corner.y)
                    else:
                        v_corner = LidarPoint(down_corner.x, down_corner.y)
                elif len(top_corner):
                    v_corner = LidarPoint(top_corner.iloc[0].x, top_corner.iloc[0].y)
                else:
                    v_corner = LidarPoint(down_corner.iloc[0].x, down_corner.iloc[0].y)

                if len(right_corner) and len(left_corner):
                    right_corner = LidarPoint(right_corner.iloc[0].x, right_corner.iloc[0].y)
                    left_corner = LidarPoint(left_corner.iloc[0].x, left_corner.iloc[0].y)

                    right_pts = df_points[
                        (df_points.x < right_corner.x - half_div) &
                        (df_points.x > corner.x + half_div) &
                        (abs(df_points.y - corner.y) < div)
                        ]

                    left_pts = df_points[
                        (df_points.x < corner.x - half_div) &
                        (df_points.x > left_corner.x + half_div) &
                        (abs(df_points.y - corner.y) < div)
                        ]

                    if len(right_pts) > len(left_pts):
                        h_corner = LidarPoint(right_corner.x, right_corner.y)
                    else:
                        h_corner = LidarPoint(left_corner.x, left_corner.y)
                elif not len(right_corner):
                    h_corner = LidarPoint(left_corner.iloc[0].x, left_corner.iloc[0].y)
                else:
                    h_corner = LidarPoint(right_corner.iloc[0].x, right_corner.iloc[0].y)

                wall_corner = WallCorner(cc=corner, cv=v_corner, ch=h_corner)
                w = wall_corner.get_walls()
                if w[0].__str__() not in walls:
                    walls[w[0].__str__()] = w[0]
                if w[1].__str__() not in walls:
                    walls[w[1].__str__()] = w[1]

            x_start, y_start = [], []
            x_end, y_end = [], []
            for wl in walls:
                x_start.append(walls[wl].start.x)
                y_start.append(walls[wl].start.y)
                x_end.append(walls[wl].end.x)
                y_end.append(walls[wl].end.y)

            self.walls = pd.DataFrame(
                data=list(zip(x_start, y_start, x_end, y_end)),
                columns=['xstart', 'ystart', 'xend', 'yend']
            ).sort_values(
                by=['xstart', 'ystart', 'xend', 'yend']
            ).drop_duplicates().reset_index(
                drop=True
            ).astype(int)

        return self.walls

    def generate_mapping_csv(self, csv_file="./.cache/Mapping.csv", verbose=False):
        output_dir = os.path.dirname(csv_file)
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
        if verbose:
            print("generating walls")
        self.get_walls().astype(int).to_csv(csv_file, index=False, header=False)
        if verbose:
            print("Written the csv file: {}, with the following contents".format(os.path.realpath(csv_file)))
            print(self.get_walls())

    def is_connected(self, p1: LidarPoint, p2: LidarPoint) -> bool:
        walls = self.get_walls()
        lv = LidarVector(p1, p2)
        wall_distance = get_distance_from_nearest_wall(
            wall_candidates=walls,
            pt=p1,
            theta=math.degrees(lv.direction)
        )
        return wall_distance and (wall_distance >= lv.distance)

    def get_wall_openings(self, min_wall_length=500) -> List[WallOpening]:
        if self.wall_openings is None:
            # Get short walls, walls with length < min_wall_length
            walls = self.get_walls().copy()
            walls['length'] = ((walls.xstart - walls.xend) ** 2 + (walls.ystart - walls.yend) ** 2) ** 0.5
            walls = walls[walls.length < min_wall_length]
            walls['angle'] = walls.apply(
                lambda row: math.degrees(math.atan2(row.yend - row.ystart, row.xend - row.xstart)),
                axis=1)
            self.wall_openings = []
            walls_seen = set()
            for _, wall1 in walls.iterrows():
                w1 = LidarVector(
                    start=LidarPoint(wall1.xstart, wall1.ystart),
                    end=LidarPoint(wall1.xend, wall1.yend)
                )

                if w1.__str__() in walls_seen:
                    continue
                else:
                    walls_seen.add(w1.__str__())
                temp_walls = walls.copy()
                temp_walls['distance'] = temp_walls.apply(
                    lambda row: ((row.xstart - wall1.xstart) ** 2 + (row.ystart - wall1.ystart) ** 2) ** 0.5, axis=1)
                temp_walls = temp_walls[temp_walls.distance > 0].sort_values(by='distance')
                wall2 = temp_walls.iloc[0]

                w2 = LidarVector(
                    start=LidarPoint(wall2.xstart, wall2.ystart),
                    end=LidarPoint(wall2.xend, wall2.yend)
                )
                if w2.__str__() in walls_seen:
                    continue
                else:
                    walls_seen.add(w2.__str__())
                self.wall_openings.append(WallOpening(wall1=w1, wall2=w2))

        return self.wall_openings

    def get_nodes(self, start: LidarPoint, end: LidarPoint) -> List[LidarPoint]:
        nodes = [start, end]

        wall_openings = self.get_wall_openings()
        for wo in wall_openings:
            wo_v = wo.get_mid_points()
            wo1, wo2 = wo_v.start, wo_v.end

            wo1.add_opening_pair(wo2)
            wo2.add_opening_pair(wo1)

            nodes += [wo1, wo2]

        for i in range(len(nodes)):
            for j in range(len(nodes)):
                if i == j:
                    continue
                if self.is_connected(nodes[i], nodes[j]):
                    nodes[i].add_neighbor(nodes[j])
                    nodes[j].add_neighbor(nodes[i])

        return nodes

    def get_optimum_flight_path(self, start: LidarPoint, end: LidarPoint, is_visit_all_rooms=False, plot=False,
                                fp_csv=None, verbose=False, min_wall_distance=300):
        connections = []
        points = self.get_all_points()
        if verbose:
            print("generating nodes")
        for node in self.get_nodes(start=start, end=end):
            for neighbor in node.neighbors:
                vector = LidarVector(node, neighbor)
                a, b, c = vector.get_abc()

                if vector.start.x < vector.end.x:
                    start_x, end_x = vector.start.x, vector.end.x
                else:
                    end_x, start_x = vector.start.x, vector.end.x

                if vector.start.y < vector.end.y:
                    start_y, end_y = vector.start.y, vector.end.y
                else:
                    end_y, start_y = vector.start.y, vector.end.y

                p = points[
                    (points.x >= start_x) & (points.x <= end_x) &
                    (points.y >= start_y) & (points.y <= end_y) &
                    (a * points.x + b * points.y > c)
                    ]

                if not len(p):
                    connections.append(
                        (node, neighbor, vector.distance)
                    )

        x_s, y_s = [], []

        if verbose:
            print("analyzing shortest path")
        positions = Graph(connections).dijkstra(start, end)
        for pos in positions:
            x_s.append(pos.x)
            y_s.append(pos.y)

        df = pd.DataFrame(data=list(zip(x_s, y_s)), columns=['x', 'y'])

        if fp_csv is not None:
            fp_csv_dir = os.path.dirname(fp_csv)
            if not os.path.exists(fp_csv_dir):
                os.makedirs(fp_csv_dir)

            with open(fp_csv, 'w', newline='') as csvfile:
                spamwriter = csv.writer(
                    csvfile,
                    delimiter=',',
                    quotechar='"',
                    quoting=csv.QUOTE_MINIMAL
                )
                for i in range(len(df)):
                    row = df.iloc[i]
                    spamwriter.writerow([i, 1])
                    spamwriter.writerow([row.x / 1e3, row.y / 1e3])

            if verbose:
                print("Written the csv file: {}".format(os.path.realpath(fp_csv)))

        if plot:
            df.plot(kind='line', x='x', y='y', color='r', ax=self.ax)
            if verbose:
                print("plotting results")
            self.visualize_lidar_points(drone_pos=False)

        return df


if __name__ == '__main__':
    dm = DroneMap(
        lidar_points_csv='./.cache/LIDARPoints.csv',
        flight_path_csv='./.cache/FlightPath.csv'
    )

    """Visualize Lidar points"""
    # dm.visualize_lidar_points(by_scan_id=False)
    # dm.generate_mapping_csv(csv_file="./.cache/Mapping.csv")

    """Test for connectivity between two points"""
    # lp1 = LidarPoint(5e3, 4e3)
    # lp2 = LidarPoint(15e3, 14e3)
    # assert not dm.is_connected(lp1, lp2), "{} {} should not be connected".format(lp1, lp2)
    # lp1 = LidarPoint(5e3, 4e3)
    # lp2 = LidarPoint(7.5e3, 8e3)
    # assert dm.is_connected(lp1, lp2), "{} {} should be connected".format(lp1, lp2)

    """Get Wall Openings"""
    # print(dm.get_wall_openings())

    """Get nodes of each room"""
    # nodes = dm.get_nodes(start=LidarPoint(7.5e3, 8e3), end=LidarPoint(17.5e3, 8e3))
    # print(nodes)

    """Get Optimum Flight Path"""
    dm.get_optimum_flight_path(
        start=LidarPoint(7.5e3, 8e3),
        end=LidarPoint(17.5e3, 12e3),
        fp_csv='./.cache/fp.csv',
    )
