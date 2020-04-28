# TODO: Simulation
# Input: Mapping.csv, FlightPath.csv
# Output: LidarPoints.csv

# TODO: Mapping
# Input: LidarPoints.csv, FlightPath.csv
# Output: Mapping.csv

from drone_map import DroneMap


def visualize(drone_map: DroneMap, by_scan_id: bool):
    drone_map.visualize_lidar_points(by_scan_id=by_scan_id)


if __name__ == '__main__':
    dm = DroneMap(
        lidar_points_csv='./.cache/LIDARPoints.csv',
        flight_path_csv='./.cache/FlightPath.csv'
    )

    """
    Challenge 1: Display
    """
    # visualize(drone_map=dm, by_scan_id=False)

    """
    Challenge 5: Mapping
    """
    dm.generate_mapping_csv(csv_file="./.cache/Mapping.csv")