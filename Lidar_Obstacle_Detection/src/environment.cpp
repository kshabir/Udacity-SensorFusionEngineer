#include <thread>
#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
#include "boundingBox.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

//#define PCL_METHOD
#define CUSTOMIZED_METHOD

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar"); // Green: Ego Car, Blue: Rest of Cars
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor
    Lidar* lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ>* pointProcessor;

    // Perform Segmentation (Ransac based Plane fitting)
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentClouds = pointProcessor->SegmentPlane(inputCloud, 100, 0.2);
    // Separate Obstacles and Road/plane clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacleCloud = segmentClouds.first;
    pcl::PointCloud<pcl::PointXYZ>::Ptr roadCloud = segmentClouds.second;


    // Rending both clouds
    renderPointCloud(viewer, roadCloud, "RoadCloud", Color(0, 1, 0));
    renderPointCloud(viewer, obstacleCloud, "ObstacleCloud", Color(1, 0, 0));
    
    // Perform Clustering: use obstacle cloud
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = pointProcessor->Clustering(obstacleCloud, 1.0, 3, 30);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> l1clusters = pointProcessor->manhattanClustering(obstacleCloud, 1.0, 3, 30);
    // Extract each cluster and render it after placing a BB

    int clusterId = 0;

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : clusters)
    {
        std::cout << "cluster size ";
        std::cout << cluster->points.size() << std::endl;
        renderPointCloud(viewer, cluster, "obstCloud" +std::to_string(clusterId), Color(1, 0, 0));
        Box box = pointProcessor->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        clusterId++;
    }
}

// For streaming PCD files
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    
    //Perform filtering (pre-processing)
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFiltered = pointProcessorI->FilterCloud(inputCloud, 0.18, Eigen::Vector4f(-10, -5, -2, 1), Eigen::Vector4f(30, 7, 4, 1));

    // Perform Segmentation (Ransac based Plane fitting): Default pcl method or customized method
    # if defined PCL_METHOD
        std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentClouds = pointProcessorI->SegmentPlane(cloudFiltered, 100, 0.2);
    #elif defined CUSTOMIZED_METHOD
        std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentClouds = pointProcessorI->SegmentPlaneRansac(cloudFiltered, 100, 0.2);
    #endif
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr obstacleCloud = segmentClouds.first;
    pcl::PointCloud<pcl::PointXYZI>::Ptr roadCloud = segmentClouds.second;

    // Rending both clouds
    renderPointCloud(viewer, roadCloud, "RoadCloud", Color(0, 1, 0));
    renderPointCloud(viewer, obstacleCloud, "ObstacleCloud", Color(1, 0, 0));

    // Perform Clustering using obstacle cloud
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = pointProcessorI->Clustering(obstacleCloud, 0.5, 10, 600);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,1), Color(1,1,0)};
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : clusters)
    {
        std::cout << "cluster size ";
        std::cout << cluster->points.size() << std::endl;
        renderPointCloud(viewer, cluster, "obstcleCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);

        Box box = pointProcessorI->BoundingBox(cluster);
        //BoundingBox customizedBox = pointProcessorI->customizedBoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }

}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<std::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    std::cout << "stream size: " << stream.size() << std::endl;
    auto streamIt = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;


    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // load pcd file
        inputCloudI = pointProcessorI->loadPcd((*streamIt).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);
        streamIt++;
        // Loop the stream
        if(streamIt == stream.end())
        streamIt = stream.begin();

        // some delay to see the result
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        viewer->spinOnce();
    } 
}
