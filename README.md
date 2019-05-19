# indoor-reconstruction


This reconstruction system is developed by us, Fredrik Kristoffer Johanssen and Vetle Smedbakken Sillerud, as part of our master thesis in Cybernetics; "Reconstruction of Indoor Environments Using LiDAR and IMU." Below is a reconstruction of both authors, Fredrik on the left, and Vetle on the right. 
<p align="center">
<img src="/images/Author_Fredrik.png"
     alt="Fredrik"
     width="300" /><img src="/images/author_vetle.png"
     alt="Vetle"
     width="250" /></p>


## Summary
The goal of this thesis is to develop a method for reconstructing accurate point cloud representations of indoor environments using the Velodyne LiDAR Puck 16 (VLP-16) and the Tinkerforge IMU 2.0. The following points can summarize the general goals of this thesis:
* Develop a data acquisition system for collecting LiDAR and IMU data. This system must handle synchronization of the data acquisition in order to prepare the scans for alignment. 
* Generate high-density point clouds by combining LiDAR and IMU data.

We conclude that at subset level, our reconstruction system can reconstruct high-density point clouds of indoor environments with a precision that is mostly limited to the inherent uncertainties of the VLP- 16. We also conclude that the registration of several subsets obtained from different positions is able to preserve both visual appearance and reflective intensity of objects in the scene. Our reconstruction system can thus be utilized to generate real data sets of high-density point clouds. 


## System setup
The system setup consist of; Tinkerforge IMU 2.0, 3D printed IMU mounting cap, VLP-16 LiDAR, Hama Star 61 153-3D tripod, VLP-16 Interface Box, Computer.
<p align="center">
<img src="/images/system_setup.png"
     alt="System Setup"
     width="500" /></p>


## Developed method
Our developed method consist of two parts: data acquisition and data processing. The data acquisition is written in C++ and can be found in the C++ folder. The data processing is written in python and can be found in the python folder. 



## Results 
Result from one stationary position:

<p align="center">
<img src="/images/robin_wall_image.jpeg"
     alt="Shelf with unique figures"
     width="400" /><img src="/images/robin_wall_cloud.png"
     alt="Point cloud representation of shelf"
     width="420" /></p>


Results from several positions with odometry between each:

<p align="center">
<img src="/images/hallway_overview.png"
     alt="Hallway"
     width="420" /></p>


<p align="center">
<img src="/images/hallway_1_image.jpeg"
     alt="Hallway viewpoint 1 - image"
     width="373" /><img src="/images/hallway_1_cloud.png"
     alt="Hallway viewpoint 1 - point cloud"
     width="447" /></p>
     
<p align="center">
<img src="/images/hallway_2_image.jpg"
     alt="Hallway viewpoint 2 - image"
     width="373" /><img src="/images/hallway_2_cloud.png"
     alt="Hallway viewpoint 2 - point cloud"
     width="447" /></p>
     
     
<p align="center">
<img src="/images/hallway_3_image.jpeg"
     alt="Hallway viewpoint 3 - image"
     width="373" /><img src="/images/hallway_3_cloud.png"
     alt="Hallway viewpoint 3 - point cloud"
     width="447" /></p>


