from drone_map import DroneMap
from simulation import generate_lidar_points


def visualize(drone_map: DroneMap, by_scan_id=False):
    drone_map.visualize_lidar_points(by_scan_id=by_scan_id)


if __name__ == '__main__':
    """
    Challenge 1: Display
        Input:  LidarPoints.csv, FlightPath.csv
        Output: Plots
    """
    # dm = DroneMap(
    #     lidar_points_csv='./.cache/LIDARPoints.csv',
    #     flight_path_csv='./.cache/FlightPath.csv'
    # )
    # visualize(drone_map=dm, by_scan_id=False)

    """
    Challenge 5: Mapping
        Input:  LidarPoints.csv, FlightPath.csv
        Output: Mapping.csv
    """
    # dm.generate_mapping_csv(csv_file="./.cache/Mapping.csv")

    """
    Challenge 2: Simulation
        Input:  Mapping.csv, FlightPath.csv
        Output: LidarPoints.csv
    """
    # lp_csv = './.cache/lp.csv'
    # fp_csv = './.cache/FlightPath.csv'
    # # TODO memoize lidar point generation
    # lp = generate_lidar_points(
    #     m_csv='./.cache/Mapping.csv',
    #     fp_csv=fp_csv,
    #     lp_csv=lp_csv,
    #     num_points=534
    # )
    # # TODO Improve accuracy of identifying corners
    # visualize(
    #     drone_map=DroneMap(
    #         lidar_points_csv=lp_csv,
    #         flight_path_csv=fp_csv
    #     )
    # )

    """
    Challenege 4: Flight Reroute
        Input: LidarPoints.csv and FlightPath.csv
        Output: FlightPath.csv
    """
