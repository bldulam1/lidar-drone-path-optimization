import csv

import pandas as pd


class DroneMap:
    def __init__(self, lidar_points_csv: str, flight_path_csv: str):
        self.lidar_points_csv = lidar_points_csv
        self.flight_path_csv = flight_path_csv

        self.drone_positions = None

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


if __name__ == '__main__':
    dm = DroneMap(
        lidar_points_csv='./.cache/LIDARPoints.csv',
        flight_path_csv='./.cache/FlightPath.csv'
    )

    print(dm.get_drone_positions())
