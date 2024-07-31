#ifndef BOUNDING_BOX_H
#define BOUNDING_BOX_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <Eigen/Dense>

template<typename PointT>
class BoundingBox {
private:
    typename pcl::PointCloud<PointT>::Ptr cloud;
    Eigen::Vector3f center;
    Eigen::Vector3f dimensions;
    float yaw;
    int id;
    float confidence;

    static int next_id;

    void computeProperties() {
        pcl::compute3DCentroid(*cloud, center);
        
        PointT min_pt, max_pt;
        pcl::getMinMax3D(*cloud, min_pt, max_pt);
        dimensions = max_pt.getVector3fMap() - min_pt.getVector3fMap();
        
        yaw = std::atan2(dimensions.y(), dimensions.x());
        
        id = next_id++;
        
        confidence = 1.0f;
    }

    typename pcl::PointCloud<PointT>::Ptr getCloud() const { return cloud; }
    Eigen::Vector3f getCenter() const { return center; }
    Eigen::Vector3f getDimensions() const { return dimensions; }
    float getYaw() const { return yaw; }
    int getId() const { return id; }
    float getConfidence() const { return confidence; }

public:
    BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster) : cloud(cluster) {
        computeProperties();
    }

    // Public methods can be added here as needed
};

template<typename PointT>
int BoundingBox<PointT>::next_id = 0;

#endif // BOUNDING_BOX_H

