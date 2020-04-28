import os

from drone_map import DroneMap
from lidar_point import LidarPoint
from simulation import generate_lidar_points


def solution_1():
    """
    Challenge 1: Display
        Input:  LidarPoints.csv, FlightPath.csv
        Output: Plots
    """
    dm = DroneMap(
        lidar_points_csv='./.cache/LIDARPoints.csv',
        flight_path_csv='./.cache/FlightPath.csv'
    )
    dm.visualize_lidar_points(by_scan_id=True)


def solution_2():
    """
    Challenge 2: Simulation
        Input:  Mapping.csv, FlightPath.csv
        Output: LidarPoints.csv
    """
    lp_csv = './.cache/lp.csv'
    fp_csv = './.cache/FlightPath.csv'
    # TODO memoize lidar point generation
    generate_lidar_points(
        m_csv='./.cache/Mapping.csv',
        fp_csv=fp_csv,
        lp_csv=lp_csv,
        num_points=534
    )
    # TODO Improve accuracy of identifying corners
    dm = DroneMap(
        lidar_points_csv=lp_csv,
        flight_path_csv=fp_csv
    )
    dm.visualize_lidar_points()


def solution_4():
    """
    Challenege 4: Flight Reroute
        Input: LidarPoints.csv and FlightPath.csv
        Output: FlightPath.csv
    """
    dm = DroneMap(
        lidar_points_csv='./.cache/LIDARPoints.csv',
        flight_path_csv='./.cache/FlightPath.csv'
    )
    dm.get_optimum_flight_path(
        start=LidarPoint(7.5e3, 8e3),
        end=LidarPoint(17.5e3, 12e3),
        fp_csv='./.cache/fp.csv',
        plot=True
    )


def solution_5():
    """
    Challenge 5: Mapping
        Input:  LidarPoints.csv, FlightPath.csv
        Output: Mapping.csv
    """
    dm = DroneMap(
        lidar_points_csv='./.cache/LIDARPoints.csv',
        flight_path_csv='./.cache/FlightPath.csv'
    )
    csv_file = "./.cache/Mapping.csv"
    dm.generate_mapping_csv(csv_file=csv_file)
    print("Generated {}".format(os.path.realpath(csv_file)))


if __name__ == '__main__':
    # solution_1()
    # solution_2()
    # # solution_3()
    # solution_4()
    solution_5()
