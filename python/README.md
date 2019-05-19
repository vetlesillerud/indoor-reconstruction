## Notes:
These scripts work on:
* macOS Mojave v10.14.4
* GNU bash, version 3.2.57(1)-release (x86_64-apple-darwin18)
* Python 3.7.1
## Table of contents
1. [quaternions.py](#quaternions)
2. [config.json](#config)
2. [interpolate_and_write_full_scans.py](#interpolate_and_write_full_scans)
3. [multiway_registration.py](#multiway_registration)


## Information:
### *quaternions.py*:<a name="quaternions"></a>
This script contains the functions used to perform mathematical operations with quaternions. These functions are utilized **multiway_registration.py** and **interpolate_and_write_full_scans.py** but separated in order to avoid clutter.

### *config.json*:<a name="config"></a>
This file contains the key parameters that can be adjusted in order to manipulate the results of **multiway_registration.py** and **interpolate_and_write_full_scans.py**. 
This file is created avoid scrolling through all the lines of code to adjust specific parameters. 
In the parameters ELEMENT indicates a placeholder for either "datapacket", "scan", "fragment" or "odometry". This is because the parameter can be set for each of the indiviual elements. For ICP parameters ELEMENT excludes "datapacket".
In the parameters METHOD indicates a placeholder for either "point-to-point", "point-to-plane" or "color".
Parameters contained in this script:
* **name**: name of config file
* **dataset_name**: name of data set that should be utilized in the registration and interpolation scripts
* **fragment_name**: name of a specific fragment that should be e.g. visualized
* **ELEMENT_voxel_size**: This parameters decides the size of the voxels in the ELEMENT should be downsampled by voxel size
* **ELEMENT_outlier_removal**: This parameter determines the method used for outlier removal, either "statistical" or "radius"
* **ELEMENT_statistical**: Parameters for the statistical outlier removal method. Removes points that are further away from their neighbors in average. Contains additional parameters: **nb_neighbors** and **std_ratio**.  
* **ELEMENT_statistical** - **nb_neighbors**: decides the maximum number of neighbors around the target point. 
* **ELEMENT_statistical** - **std_ratio**: decides the standard deviation ratio.
* **ELEMENT_radius**: Parameter for the radius outlier removal method. Contains additional parameters: **nb_points** and **radius**. Removes points that have less than **nb_points** points in a given sphere of given **radius**.
* **ELEMENT_radius** - **nb_points**: number of points within the radius
* **ELEMENT_radius** - **radius**: radius of the sphere
* **local_icp_method_ELEMENT**: Specifies which ICP method that should be utilized, either "point-to-point", "point-to-point" or "color".
* **ELEMENT** - **icp** - **METHOD** - **max_correspondence_distance**: maximum correspondence point-pair distance
* **ELEMENT** - **icp** - **METHOD** - **max_iteration**: maximum number of iterations before iteration stops
* **ELEMENT** - **icp** - **METHOD** - **relative_fitness**: if the difference of fitness score is lower than this parameters iteration stops
* **ELEMENT** - **icp** - **METHOD** - **relative_rmse**: if the difference of inlier RMSE is lower than this parameter iteration stops
* **ELEMENT** - **icp** - **color** - **lambda_geometric**: how much the color is weighted in the ICP procedure. If 1, standard point-to-plane is utilized. If 0.5, color and normal vector are weighted equal.
* **pose_graph_optimization** - **max_correspondence_distance**: the max_correspondence distance utilized in the ICP procedure
* **pose_graph_optimization** - **edge_prune_threshold**: if loop closure are not associated with a value higher that this parameter, the loop closure are pruned.

### *interpolate_and_write_full_scans.py*:<a name="interpolate_and_write_full_scans"></a>
This script will read data packet PCD-files, e.g., "scan_0_1.pcd" and interpolate the quaternions in "quaternions_datapacket.csv". It will then transform the individual data packets with the interpolated quaternions and combine these data packets to a scan representing the scope used for acquisition.
The linear interpolation is done by comparing each quaternion with the previous and collecting these similar, subsequent quaternions.  These collected quaternions are interpolated from the first quaternion of a similar sequence, until the value of the quaternion that breaks this sequence.
````
usage: interpolate_and_write_full_scans.py [-h] [--visualization]
                                           [--directory DIRECTORY]

Reads datapacket pcds, interpolates quaternions and generates scans from
dataset in config file

optional arguments:
  -h, --help            show this help message and exit
  --visualization, -v   if generated clouds should be visualized
  --directory DIRECTORY, -d DIRECTORY
                        if only specified directory should be interpolated,
                        e.g. 'fragments/fragment0'
````
Example:

Interpolate all the data packet PCD files in the "dataset_name" directory.
````
$ python interpolate_and_write_full_scans.py 
````
Example:

Interpolate all the data packet PCD files in the specified subdirectory, i.e. "fragments/fragment0", of "dataset_name" directory.
````
$ python interpolate_and_write_full_scans.py -d fragments/fragment0
````

### *multiway_registration.py*:<a name="multiway_registration"></a>
This the main script of this thesis. It will do visualization and registration of the point clouds. This script will read PCD-files such as "scan0.pcd", and will utilizes the parameters of the config.json file.
````
usage: multiway_registration.py [-h] [--icp ICP]
                                [--visualization VISUALIZATION] [--verbose]
                                [--odometry] [--generate] [--blensor]
                                [--trajectory]

Multiway registration script for point cloud data. Supported point cloud
format: .pcd

optional arguments:
  -h, --help            show this help message and exit
  --icp ICP, -i ICP     if icp and optimization should be done on scans in
                        specified folder of dataset. Uses dataset specified in
                        config file. Use: -i odometry/odometryX
  --visualization VISUALIZATION, -v VISUALIZATION
                        if all scans in specified folder should be visualized
                        using only transform from IMU data.Uses dataset
                        specified in config file. Use: -v fragments/fragmentX
  --verbose, -vb        if open3d.VerbosityLevel.Debug should be activated
  --odometry, -o        if odometry should be estimated and fragments should
                        be registered
  --generate, -g        True if both odometry and fragments in should be
                        generated before registering fragments. False if
                        previously generated pose graphs and clouds should be
                        used to register fragments.
  --blensor, -b         True if scans generated in BlenSor should be used
  --trajectory, -t      True if registered cloud should be visualized with
                        trajectory
````
Example:

Register the point clouds contained in the "fragments/fragment0" subdirectory of "dataset_name" using the ICP method and parameters specified in config file.
````
$ python multiway_registration.py -i fragments/fragment0
````
Example:

Visualize the point clouds contained in the "fragments/fragment0" subdirectory of "dataset_name".
````
$ python multiway_registration.py -v fragments/fragment0
````
Example:

Register each fragment contained in the "dataset_name" directory and estimate the translation using the clouds contained in the "odometry" subdirectory. When the fragments and the odometry is estimated, the fragments will be registered and written to "multiway_registration.pcd" file in "fragments" subdirectory. If "-g" is alleviated, previously generated fragments and odometry will be read, so that only the fragments are registered.
````
$ python multiway_registration.py -o -g
````
Example:

Visualize the "multiway_registration.pcd" file in "fragments" subdirectory of "dataset_name" with trajectory and coordinate frames.
````
$ python multiway_registration.py -t
````
Example:

Visualize point clouds generated in BlenSor. Will read the clouds contained in the "/tmp/" directory of the computer in use if BlenSor is installed. "-i" and "-o" can also be utilized in stead of "-v" for registration or odometry estimation purposes. 
````
$ python multiway_registration.py -b -v
````
