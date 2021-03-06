import argparse

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
    parser.add_argument('--start_x',
                        action="store", dest="start_x", type=float,
                        help='x starting point')
    parser.add_argument('--start_y',
                        action="store", dest="start_y", type=float,
                        help='y starting point')
    parser.add_argument('--end_x',
                        action="store", dest="end_x", type=float,
                        help='x end point')
    parser.add_argument('--end_y',
                        action="store", dest="end_y", type=float,
                        help='y end point')
    parser.add_argument('-v',
                        action="store_true", dest='verbose', default=False,
                        help='verbose')

    args = parser.parse_args()

    if args.verbose and args.challenge <= 5:
        print('Executing Challenge %d' % args.challenge)

    if args.challenge == 1:
        """
        Challenge 1: Display
            Input:  LidarPoints.csv, FlightPath.csv
            Output: Plots
        """
        dm = DroneMap(lidar_points_csv=args.lp_csv, flight_path_csv=args.fp_csv)
        dm.visualize_lidar_points(by_scan_id=True)

    elif args.challenge == 2:
        """
        Challenge 2: Simulation
            Input:  Mapping.csv, FlightPath.csv
            Output: LidarPoints.csv
        """
        # TODO memoize lidar point generation
        # TODO figure 1 is an empty figure
        generate_lidar_points(m_csv='./.cache/Mapping.csv', fp_csv=args.fp_csv, lp_csv=args.lp_csv, num_points=534,
                              verbose=args.verbose)
        # TODO Improve accuracy of identifying corners
        dm = DroneMap(lidar_points_csv=args.lp_csv, flight_path_csv=args.fp_csv)
        dm.visualize_lidar_points()
    elif args.challenge == 3 or args.challenge == 4:
        """
        Challenege 4: Flight Reroute
            Input: LidarPoints.csv and FlightPath.csv
            Output: FlightPath.csv
        """

        if args.fp_csv is None:
            print("missing flight path csv")
        elif args.lp_csv is None:
            print("missing lidar points csv")
        elif args.start_x is None:
            print("missing starting point x coordinate")
        elif args.start_y is None:
            print("missing starting point y coordinate")
        elif args.end_x is None:
            print("missing end point x coordinate")
        elif args.end_y is None:
            print("missing end point y coordinate")
        else:
            dm = DroneMap(lidar_points_csv=args.lp_csv, flight_path_csv='./.cache/FlightPath.csv')
            dm.get_optimum_flight_path(
                end=LidarPoint(args.end_x, args.end_y),
                start=LidarPoint(args.start_x, args.start_y),
                fp_csv=args.fp_csv,
                plot=True,
                is_visit_all_rooms=args.challenge == 3,
                verbose=args.verbose,
            )

    elif args.challenge == 5:
        """
        Challenge 5: Mapping
            Input:  LidarPoints.csv, FlightPath.csv
            Output: Mapping.csv
        """
        dm = DroneMap(lidar_points_csv=args.lp_csv, flight_path_csv=args.fp_csv)
        dm.generate_mapping_csv(csv_file=args.m_csv, verbose=True)

    else:
        raise argparse.ArgumentTypeError("Challenge index should be one of the following: 1, 2, 3, 4, 5")
