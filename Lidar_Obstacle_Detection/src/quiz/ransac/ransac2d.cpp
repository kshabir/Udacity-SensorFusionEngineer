/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	while(maxIterations-- > 0)
	{
		// Randomly sample subset and fit line
		std::unordered_set<int> inliers;
		while(inliers.size() < 3)
		{
			inliers.insert(rand() % cloud->points.size());
		}

		pcl::PointXYZ point1, point2, point3;
		auto itr = inliers.begin();
		point1 = cloud->points[*itr];

		itr++;
		point2 = cloud->points[*itr];

		itr++;
		point3 = cloud->points[*itr];

		// Measure distance between every point and fitted plane

		float A, B, C, D;
		A = (point2.y - point1.y) * (point3.z - point1.z) - (point2.z - point1.z) * (point3.y - point1.y);
		B = (point2.z - point1.z) * (point3.x - point1.x) - (point2.x - point1.x) * (point3.z - point1.z);
		C = (point2.x - point1.x) * (point3.y - point1.y) - (point2.y - point1.y) * (point3.x - point1.x);
		D = - 1* (A*point1.x + B*point1.y + C*point1.z);

		// Measure distance between every point and fitted line
		for(int i = 0; i < cloud->points.size(); i++)
		{
			float x = cloud->points[i].x;
			float y = cloud->points[i].y;
			float z = cloud->points[i].z;
			float dist = fabs(A*x + B*y + C*z + D) / sqrt(A*A + B*B + C*C);
			if(dist < distanceTol)
			{
				inliers.insert(i);
			}
		}
		if(inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}
	}
	
	return inliersResult;

}

std::unordered_set<int> RansacLine(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	while(maxIterations-- > 0)
	{
		// Randomly sample subset and fit line
		std::unordered_set<int> inliers;
		while(inliers.size() < 2)
		{
			inliers.insert(rand() % cloud->points.size());
		}

		float x1, y1, z1, x2, y2, z2;
		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;

		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;

		float A, B, C;
		A = y1 - y2;
		B = x2 - x1;
		C = x1*y2 - x2*y1;

		// Measure distance between every point and fitted line
		for(int i = 0; i < cloud->points.size(); i++)
		{
			float x3 = cloud->points[i].x;
			float y3 = cloud->points[i].y;
			float dist = fabs(A*x3 + B*y3 + C) / sqrt(A*A + B*B);
			if(dist < distanceTol)
			{
				inliers.insert(i);
			}
		}
		if(inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}
	}
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	//std::unordered_set<int> inliers = RansacLine(cloud, 25, 0.33);
	std::unordered_set<int> inliers = RansacPlane(cloud, 25, 0.33);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
