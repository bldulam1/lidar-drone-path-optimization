# Drone Lidar Path Optimization

Given a lidar point cloud and a drone's positions, this application calculates the optimum path that a drone should traverse from one end to another.

## Getting Started

This application is written in Python3. Please follow the following steps to run this application.

### Prerequisites

Please make sure that you are using python3 to run this application

### Cloning this Repository

To clone this repository, please make sure that you have installed git.
Then execute the following:

```shell script
git clone https://github.com/bldulam1/lidar-drone-path-optimization
```
### Installing Dependencies

To install the package dependencies, please install the packages in the `requirements.txt`.

Please go to this project's directory then execute the following in the shell

```shell script
cd lidar-drone-path-optimization
pip install -r ./requirements.txt
```

### Predefined tests
To test the solution to the challenges, please use the following tests, by executing through the shell.

_Please make sure that you are in the directory of this repository when executing the following commands._

##### Display
```shell script
python main.py --lp_csv ./.cache/LIDARPoints.csv --fp_csv ./.cache/FlightPath.csv -v --challenge 1
```

##### Simulation
```shell script
python main.py --m_csv ./.cache/Mapping.csv --lp_csv ./.cache/lp.csv --fp_csv ./.cache/FlightPath.csv -v --challenge 2
```

##### Flight Optimization
```shell script
python main.py --m_csv ./.cache/Mapping.csv --lp_csv ./.cache/LIDARPoints.csv --fp_csv ./.cache/fp.csv -v --start_x 11e3 --start_y 4e3 --end_x 2.5e3 --end_y 12e3 --challenge 3
```


##### Flight Reroute
```shell script
python main.py --m_csv ./.cache/Mapping.csv --lp_csv ./.cache/LIDARPoints.csv --fp_csv ./.cache/fp.csv -v --start_x 11e3 --start_y 4e3 --end_x 2.5e3 --end_y 12e3 --challenge 4
```

##### Mapping
```shell script
python main.py --m_csv ./.cache/Mapping.csv --lp_csv ./.cache/LIDARPoints.csv --fp_csv ./.cache/FlightPath.csv -v --challenge 5
```


#### Description of Command Line Arguments
The following shows the description of the command line arguments of the application.

|     Flag    	|                             Description                             	| Default Value 	|              Example             	|
|:-----------:	|:-------------------------------------------------------------------:	|---------------	|:--------------------------------:	|
| --challenge 	|                    Specifies the challenge number                   	|               	| --challenge 3                    	|
| --lp_csv    	|                         lidar point csv file                        	|               	| --lp_csv ./.cache/lp.csv         	|
| --fp_csv    	|                         flight path csv file                        	|               	| --fp_csv ./.cache/FlightPath.csv 	|
| --m_csv     	|                           mapping csv file                          	|               	| --m_csv ./.cache/Mapping.csv     	|
| --start_x   	|  x-coordinate of the starting point, required in challenges 3 and 4 	|               	| --start_x 11e3                   	|
| --start_y   	| y-coordinate of the starting point,  required in challenges 3 and 4 	|               	| --start_y 4e3                    	|
| --end_x     	|    x-coordinate of the end point,  required in challenges 3 and 4   	|               	| --end_x 2.5e3                    	|
| --end_y     	|    y-coordinate of the end point,  required in challenges 3 and 4   	|               	| --end_y 12e3                     	|
| -v          	|          verbose,  displays each major step in the program          	|               	| -v                               	|


#### Explanation of the Solution
##### Reconstructing the Lidar Point Cloud
The lidar point cloud is reconstructed from a CSV file with the following format.

| Scan ID, _0_             	| number of points, _n_       	|
|------------------------	|---------------------------	|
| angle, _θ<sub>0</sub>_  	| distance, _r<sub>0</sub>_   	|
| angle, _θ<sub>1</sub>_  	| distance, _r<sub>1</sub>_   	|
| ...                       | ...                       	|
| angle, _θ<sub>n-1</sub>_ | distance, _r<sub>n-1</sub>_ 	|

| Scan ID, _1_             	| number of points, _m_       	|
|------------------------	|---------------------------	|
| angle, _θ<sub>0</sub>_  	| distance, _r<sub>0</sub>_   	|
| angle, _θ<sub>1</sub>_  	| distance, _r<sub>1</sub>_   	|
| ...                       | ...                       	|
| angle, _θ<sub>m-1</sub>_ | distance, _r<sub>m-1</sub>_ 	|

...

| Scan ID, _i_             	| number of points, p       	|
|------------------------	|---------------------------	|
| angle, θ<sub>0</sub>   	| distance, r<sub>0</sub>   	|
| angle, θ<sub>1</sub>   	| distance, r<sub>1</sub>   	|
| ...                    	| ...                       	|
| angle, θ<sub>p-1</sub> 	| distance, r<sub>p-1</sub> 	|

into a pandas DataFrame with x,y,i as the column headers.
The lidar point cloud which is in polar coordinates (_r_, _θ_) is converted into its cartesian form (_x_, _y_).
For this application the clockwise direction is considered the positive rotation.

The calculation of this conversion is shown [here](./drone_map.py) (in the `def get_all_points(self)` section)

##### Detecting Corners
**Assumption:**
The walls of the room are almost parallel with the x and y axes. In other words, the lidar point cloud is rotated such that its wall lines are almost perpendicular or horizontal.
The following are steps in detecting corners
1. Index the x and y series of the points DataFrame such that `x_index, y_index := x//factor, y//factor`
2. Filter x_indices such that length of `points[points.x == point.x_index]` is greater than a threshold value.
    The condition of the assumption is very important in this method.
3. Filter y_indices such that length of `points[points.y == point.y_index]` is greater than a threshold value
4. Match the filtered x and y indices, then check if there is a corner in an instance of the x_indices-y_indices pairing.

The calculation for this is shown [here](./drone_map.py) (in the `def get_all_corners(self)` section)


## Package Dependencies
* [pandas](https://pandas.pydata.org/docs/)
* [numpy](https://numpy.org/)
* [matplotlib](https://matplotlib.org/)


## Author

* **Brendon Dulam** - *Veoneer Japan* - [bldulam1](https://github.com/bldulam1) - [portfolio](https://bdulam.netlify.com)


## License

This project is licensed under the MIT License - see the [LICENSE.md](./LICENSE.md) file for details

## Acknowledgments

* Dijkstra Algorithm [video](https://www.youtube.com/watch?v=gdmfOwyQlcI&t=229s)
