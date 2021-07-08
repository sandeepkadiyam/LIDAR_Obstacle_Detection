# LIDAR_Obstacle_Detection
**Lidar** Sensor is one of the crucial sensors in state of the art autonomous vehicles. A high resolution point cloud data of the scene surrounding the vehicle can be visualized using **Lidar**. In this project, various processing techniques have been performed on the point cloud data to detect the obstacles in the scene.

## Lidar
* Lidar sends out beams of light(lasers) and measures how long it takes for them to come back and thus calculates the distance of the object.
* Lasers will be sent out in many different angles and the range is determined by lidar's field of view.
* Once the laser bounces of materials, the laser intensity value is also recieved which can be used to evaluate the properties of the material.

## Point Cloud Data
* Lidar data is stored in a format called Point Cloud Data. 
* Each .pcd file consists a list of (X,Y,Z) or (X,Y,Z,I) coordinates. I in (X,Y,Z,I) refers to the laser Intensity value.

## Project Description
The project mainly consists of four parts.
* Obtaining Point Cloud Data from the **Lidar** scan.
* Point Cloud Segmentation.
* Point Cloud Clustering.
* Point Cloud Filtering.

## Lidar Scan
* In this project, the lidar is represented in a simulator. It does ray casting to sense its surroundings.
* The Point Cloud Data of the scene surrounding the vehicle after performing a lidar scan is visualized as below,
<img src="media/simulatedPcd.png" width="700" height="400" />



