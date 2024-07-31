#ifndef BOUNDING_BOX_H
#define BOUNDING_BOX_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <string>
#include <Eigen/Dense>

template<typename PointT>
class BoundingBox {
public:
    Eigen::Vector3f center;
    Eigen::Vector3f dimensions;
    float yaw;
    int id;
    float confidence;
    std::string name;
    static int next_id;

    //BoundingBox() {}

    static BoundingBox computeProperties(typename pcl::PointCloud<PointT>::Ptr cloud)
    {
        BoundingBox box;
        
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud, centroid);
        box.center = centroid.head<3>();

        PointT min_pt, max_pt;
        pcl::getMinMax3D(*cloud, min_pt, max_pt);
        box.dimensions = max_pt.getVector3fMap() - min_pt.getVector3fMap();
        box.yaw = std::atan2(box.dimensions.y(), box.dimensions.x());
        box.id = next_id++;
        box.name = "Car";
        box.confidence = 1.0f;
        
        return box;
    }

    static BoundingBox computeBoundingBox(typename pcl::PointCloud<PointT>::Ptr cloud)
    {
        return computeProperties(cloud);
    }
};

template<typename PointT>
int BoundingBox<PointT>::next_id = 0;

#endif // BOUNDING_BOX_H
