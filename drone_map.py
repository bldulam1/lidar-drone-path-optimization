class DroneMap:
    def __init__(self, lidar_points_csv: str, flight_path_csv: str):
        self.lidar_points_csv = lidar_points_csv
        self.flight_path_csv = flight_path_csv
