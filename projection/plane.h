#ifndef PLANE_H
#define PLANE_H

#include <Eigen/Dense>
#include "scene/scene.h"

class Plane
{
public:
    Plane(Eigen::Vector3f origin, Eigen::Vector3f normal);

    Eigen::Vector3f project(const Eigen::Vector3f& origin, const Eigen::Vector3f& point3D);
    Eigen::Vector3f backProject(const Scene& scene, const Eigen::Vector3f& origin, const Eigen::Vector3f& point2D);

private:
    Eigen::Vector3f origin;
    Eigen::Vector3f normal;
};

#endif // PLANE_H
