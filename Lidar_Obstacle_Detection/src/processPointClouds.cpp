// PCL lib Functions for processing point clouds 
#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    typename pcl::PointCloud<PointT>::Ptr cloudFiltered{new pcl::PointCloud<PointT>};
    // Downsampling using voxel grid
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);


    // ROI based filtering
    pcl::CropBox<PointT> cropBoxFilter(true);
    cropBoxFilter.setMin(minPoint);
    cropBoxFilter.setMax(maxPoint);
    cropBoxFilter.setInputCloud(cloudFiltered);
    cropBoxFilter.filter(*cloudFiltered);

    // Optional: Rooftop point removal
    std::vector<int> indices;
    typename pcl::PointCloud<PointT>::Ptr optionalFiltering{new pcl::PointCloud<PointT>};

    // Step 1: Get Indices of region of interest (ROI)
    pcl::CropBox<PointT> cropBox(true);
    cropBox.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    cropBox.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    cropBox.setInputCloud(cloudFiltered);
    cropBox.filter(indices);

    // Type conversion
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    for(auto point : indices)
    {
        inliers->indices.push_back(point);
    }

    // Step 2: Remove the points outside the ROI
    pcl::ExtractIndices<PointT> extract;
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.setInputCloud(cloudFiltered);
    extract.filter(*optionalFiltering);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "downsampled original " << cloud->points.size() << " points to " << cloudFiltered->points.size() << std::endl;
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return optionalFiltering;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloud_obstacles(new pcl::PointCloud<PointT>());
    
    pcl::ExtractIndices<PointT> extract;

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_plane);

    extract.setNegative(true);
    extract.filter(*cloud_obstacles);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_obstacles, cloud_plane);
    return segResult;
}

template<typename PointT>
pcl::PointIndices::Ptr ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    pcl::PointIndices::Ptr maxInliers {new pcl::PointIndices()};
	srand(time(NULL));
	
	// TODO: Fill in this function
	while(maxIterations-- > 0)
	{
        pcl::PointIndices::Ptr inliers {new pcl::PointIndices()};

        // Randomly sample subset
        PointT point1 = cloud->points.at(rand() % (cloud->points.size()));
        PointT point2 = cloud->points.at(rand() % (cloud->points.size()));
        PointT point3 = cloud->points.at(rand() % (cloud->points.size()));

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
				inliers->indices.push_back(i);
			}
		}
		if(inliers->indices.size() > maxInliers->indices.size())
		{
			maxInliers = inliers;
		}
	}

    if(maxInliers->indices.size() == 0)
    {
        std::cout << "No inliers found" << std::endl;
    }
	
	return maxInliers;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices());

    /* pcl Internal Segmentation method */
    pcl::ModelCoefficients::Ptr coefficients{new pcl::ModelCoefficients()};
    pcl::SACSegmentation<PointT> seg;

    // Set the segmentation parameters
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneRansac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    // TODO:: Fill in this function to find inliers for the cloud.
    inliers = RansacPlane(cloud, maxIterations, distanceThreshold);
    std::cout << "Inliers size : " << inliers->indices.size() << std::endl;

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}

// template<typename PointT>
// BoundingBox<PointT> ProcessPointClouds<PointT>::mBoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);
// {
//     BoundingBox<PointT> box;
//     return box;
//     //return computeBoundingBox(cluster);
// }

// template<typename PointT>
// void ProcessPointClouds<PointT>::clusterHelper(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
//                    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster,
//                    std::vector<bool>& processed, 
//                    int index, 
//                    KdTree* tree, 
//                    float distanceTol)
// {
//     processed[index] = true;
//     cluster->push_back(cloud->points[index]);

//     PointT point = cloud->points[index];
//     std::vector<int> neighbors = tree->search({point.x, point.y, point.y}, distanceTol);

//     for (int i = 0; i < neighbors.size(); i++)
//     {
//         if (!processed[neighbors[i]])
//         {
//             clusterHelper(cloud, cluster, processed, i, tree, distanceTol);
//         }
//     }

// }


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Create a KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    typename pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    for (auto it = clusterIndices.begin(); it != clusterIndices.end(); ++it)
    {
        typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
        // Create a cluster with the extracted points belonging to the same object
        for (auto pointIt = it->indices.begin(); pointIt != it->indices.end(); ++pointIt)
        {
            cluster->push_back(cloud->points[*pointIt]);
        }
        cluster->width = cluster->size();
        cluster->height = 1;
        cluster->is_dense = true;
        // Add the cluster to the return cluster vector
        clusters.push_back(cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::manhattanClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // A Placeholder for L1-norm clustering

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    return clusters;
}

template<typename PointT>
ExtendedBox ProcessPointClouds<PointT>::computeProperties(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    ExtendedBox box;
    
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    box.center = centroid.head<3>();

    PointT min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    
    box.x_min = min_pt.x;
    box.y_min = min_pt.y;
    box.z_min = min_pt.z;
    box.x_max = max_pt.x;
    box.y_max = max_pt.y;
    box.z_max = max_pt.z;

    box.dimensions = max_pt.getVector3fMap() - min_pt.getVector3fMap();
    box.yaw = std::atan2(box.dimensions.y(), box.dimensions.x());

    static int next_id = 0;
    box.id = next_id++;
    box.confidence = 1.0f;
    box.name = "Object";

    return box;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<std::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<std::filesystem::path> paths(std::filesystem::directory_iterator{dataPath}, std::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}