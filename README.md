# Udacity-SensorFusionEngineer

Question: Things to understand about each sensor:

Purpose: What are the main strengths, weaknesses, and reasons to be? When to use it? and when not to?  
Raw Data: What is the data coming out of the sensor?  
Output: What are the main outputs people usually get when processing these sensors?  

Answer: A detailed explanation of Camera, Lidar, and Radar sensors in the context of autonomous vehicle perception systems.

## 1. Camera

1.1 Purpose:

    Strengths: Excellent at classification, High resolution, color information, texture recognition, 2D percieved structures (lane detection, traffic sign recognition, traffic lights), Can measure TTC. 
    Weaknesses: Poor performance in low light or adverse weather, limited depth perception.  
    Use when: Detailed visual information is needed, for classification tasks, and in good lighting conditions.  
    Avoid when: Precise distance measurements are required or in low visibility conditions.  

1.2 Raw Data:
2D images (usually color) at various resolutions and frame rates.

1.3 Output:
Object detection and classification (2D bounding boxes)
Semantic segmentation
Lane detection
Traffic sign recognition
Visual odometry

## 2. Lidar

2.1 Purpose:

    Strengths: Accurate distance measurements, 3D mapping, works in low light conditions.  
    Weaknesses: Performance degradation in adverse weather (rain, snow, fog), relatively expensive.  
    Use when: Precise 3D mapping is required, for obstacle detection and avoidance.  
    Avoid when: Cost is a major constraint or in extremely adverse weather conditions.  

2.2 Raw Data:
3D point clouds (x, y, z coordinates and intensity)

2.3 Output:
3D object detection (3D bounding boxes)
Ground plane estimation
Free space detection
3D mapping and localization
Point cloud segmentation

## 3 Radar

3.1 Purpose:

    Strengths: Works in all weather conditions, direct velocity measurements, long range.  
    Weaknesses: Lower resolution compared to Lidar, limited angular resolution.  
    Use when: Reliable detection in all weather conditions is needed, for velocity estimation.  
    Avoid when: High-resolution spatial information is required.  

3.2 Raw Data:
Range, azimuth, elevation, and radial velocity measurements

3.3 Output:
Object detection and tracking
Velocity estimation
Adaptive cruise control
Collision warning systems
Occupancy grid mapping
