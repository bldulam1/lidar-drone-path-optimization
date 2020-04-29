import argparse
import os

from drone_map import DroneMap
from lidar_point import LidarPoint
from simulation import generate_lidar_points

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Test for the solutions of the lidar drone path optimization')
    parser.add_argument('--challenge',
                        action="store", dest="challenge", type=int,
                        help='index for the challenge')
    parser.add_argument('--lp_csv',
                        action="store", dest="lp_csv", type=str,
                        help='complete path of the LIDARPoints.csv')
    parser.add_argument('--fp_csv',
                        action="store", dest="fp_csv", type=str,
                        help='complete path of the FlightPoints.csv')
    parser.add_argument('--m_csv',
                        action="store", dest="m_csv", type=str,
                        help='complete path of the Mapping.csv')

    args = parser.parse_args()

    if args.challenge == 1:
        """
        Challenge 1: Display
            Input:  LidarPoints.csv, FlightPath.csv
            Output: Plots
        """
        print('Executing Challenge %d' % args.challenge)
        dm = DroneMap(lidar_points_csv='./.cache/LIDARPoints.csv', flight_path_csv='./.cache/FlightPath.csv')
        dm.visualize_lidar_points(by_scan_id=True)
    elif args.challenge == 2:
        """
        Challenge 2: Simulation
            Input:  Mapping.csv, FlightPath.csv
            Output: LidarPoints.csv
        """
        print('Executing Challenge %d' % args.challenge)
        lp_csv = './.cache/lp.csv'
        fp_csv = './.cache/FlightPath.csv'
        # TODO memoize lidar point generation
        generate_lidar_points(m_csv='./.cache/Mapping.csv', fp_csv=fp_csv, lp_csv=lp_csv, num_points=534, verbose=True)
        # TODO Improve accuracy of identifying corners
        dm = DroneMap(lidar_points_csv=lp_csv, flight_path_csv=fp_csv)
        dm.visualize_lidar_points()
    elif args.challenge == 3:
        print("Challenge %s has no solution yet" % args.challenge)
    elif args.challenge == 4:
        """
        Challenege 4: Flight Reroute
            Input: LidarPoints.csv and FlightPath.csv
            Output: FlightPath.csv
        """
        print('Executing Challenge %d' % args.challenge)
        dm = DroneMap(lidar_points_csv='./.cache/LIDARPoints.csv', flight_path_csv='./.cache/FlightPath.csv')
        dm.get_optimum_flight_path(
            start=LidarPoint(7.5e3, 8e3),
            end=LidarPoint(17.5e3, 12e3),
            fp_csv='./.cache/fp.csv',
            plot=True
        )
    elif args.challenge == 5:
        """
        Challenge 5: Mapping
            Input:  LidarPoints.csv, FlightPath.csv
            Output: Mapping.csv
        """
        print('Executing Challenge %d' % args.challenge)
        dm = DroneMap(lidar_points_csv='./.cache/LIDARPoints.csv', flight_path_csv='./.cache/FlightPath.csv')
        csv_file = "./.cache/Mapping.csv"
        dm.generate_mapping_csv(csv_file=csv_file)
        print("Generated {}, with the following contents:".format(os.path.realpath(csv_file)))
        print(dm.get_walls())

    else:
        raise argparse.ArgumentTypeError("Challenge index should be one of 1, 2, 3, 4, 5")
