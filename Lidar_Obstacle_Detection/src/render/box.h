#ifndef BOX_H
#define BOX_H
#include <Eigen/Geometry> 
#include <string>

struct Box
{
    float x_min, y_min, z_min;
    float x_max, y_max, z_max;
};

struct BoxQ
{
    Eigen::Vector3f bboxTransform;
    Eigen::Quaternionf bboxQuaternion;
    float cube_length, cube_width, cube_height;
};

struct ExtendedBox : public Box
{
    Eigen::Vector3f center;
    Eigen::Vector3f dimensions;
    float yaw;
    float range;
    int id;
    float confidence;
    std::string type;
};
#endif
