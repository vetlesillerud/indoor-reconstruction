## Notes:
These scripts work on:
* macOS Mojave v10.14.4
* GNU bash, version 3.2.57(1)-release (x86_64-apple-darwin18)
* Xcode Version 10.1, GNU++14
* PCL Version 1.9.1
* OpenCV Version 4.0.1
* Boost Version 1.68.0
## Table of contents
1. [interpolation_data_collection](#interpolate)
* interpolation_vlp.cpp
* VelodyneCapture.h
2. [data_collection](#frame)
* vlp.cpp
2. [Tinkerforge_IMU2.0](#imu)
* brick_imu_v2.cpp
* brick_imu_v2.h
* ip_connection.cpp
* ip_connection.h
3. [binary_to_ascii_converter](#converter)
* binary_to_ascii.cpp

## Information:
### *interpolation_data_collection*:<a name="interpolate"></a>

This script is built upon [UnaNancyOwen's simple program](https://github.com/UnaNancyOwen/VelodyneCapture/tree/master/sample/simple) and uses his VelodyneCapture class. It retrieves data from the VLP-16 in forms of *data packets*, which is approximately *2.38&deg;* of a full *360&deg;* scan at *300 RPM*. One data packet contains data from *24* firing sequences of the 16 lasers, which results in a maximum of *384* points per packet. All data packets are written to a binary pcd file the moment it is retrieved, and an IMU measurement is addressed to a separate line in a CSV file. The point cloud is colored based on the intensity return, where the intensity value is converted to RGB float with a color mapping procedure. 

How to use this script with command line arguments:

| Syntax      | Description |
| ----------- | ----------- |
| -h      | Prints a help message|
| -d   | Directory name of where to save the pcds |
| -f   | Specify fragment number |
| -o   | or: specify odometry number |
| -start   | Specify field of view start degree [0-359]|
| -end   | Specify field of view end degree [0-359] |

Example usage: 

`./interpolation_vlp -d test_dir -f 0 -start 270 -end 90`

press **ctrl c** to stop the data collection. 


### *data_collection*:<a name="frame"></a>

This script is built upon [UnaNancyOwen's program](https://gist.github.com/UnaNancyOwen/9f9459d3c10f7a6325ebebabda9865f7#file-main-cpp). It retrieves data from the VLP-16 in the form of a *frame*, which means it captures one full *360&deg;* scan. This point cloud is written to a pcd file, and at the same moment, an IMU measurement is addressed to a separate line in a CSV file. This program also has a viewer that visualizes the scene while capturing data. 

How to use this script with command line arguments:

| Syntax      | Description |
| ----------- | ----------- |
| -h      | Prints a help message|
| -p <*.pcap>  | Velodyne capture from pcap file |
| -s   | Velodyne capture from sensor |
| -f   | Fast mode: Use space to start and stop writing pcds. Default: press space to write one scan as pcd.  |
| -d   | Directory name of where to save the pcds|


Example usage: 

`./vlp -s -f -d test_dir`

press **ctrl c** to stop the data collection. 

Here, the FOV needs to be changed in the Webinterface. 



### *Tinkerforge_IMU2.0*:<a name="imu"></a>
These scripts are from [Tinkerforge](https://github.com/Tinkerforge/imu-v2-brick) and are used in both the previous programs. The scripts allow to connect and retrieve data from the [Tinkerforge IMU 2.0](https://www.tinkerforge.com/en/doc/Hardware/Bricks/IMU_V2_Brick.html#imu-v2-brick-description). The header files are included in both the previous scripts. 

### *binary_to_ascii_converter*:<a name="converter"></a>
This script simply converts an input ASCII pcd to a binary pcd file. 

Example usage: 

`./converter some_ascii_pcd_file.pcd`



