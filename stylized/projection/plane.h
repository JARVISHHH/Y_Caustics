#ifndef PLANE_H
#define PLANE_H

#include <Eigen/Dense>
#include "scene/scene.h"

class Plane
{
public:
    Plane();
    Plane(float rotateAngle, Eigen::Vector3f center3D, Eigen::Vector3f normal3D);

    Eigen::Vector2f projectPoint(const Eigen::Vector3f& pointOrigin, const Eigen::Vector3f& point3D);
    Eigen::Vector2f projectDirection(const Eigen::Vector3f& pointOrigin, const Eigen::Vector3f& pointDirection);
    Eigen::Vector3f backProjectPoint(const Scene& scene, const Eigen::Vector3f& pointOrigin, const Eigen::Vector2f& point2D);
    Eigen::Vector3f backProjectDirection(const Scene& scene, const Eigen::Vector3f& pointOrigin, const Eigen::Vector2f& point2D);

private:
    float rotateAngle;

    Eigen::Vector3f center3D;  // center of the plane in 3D space
    Eigen::Vector3f normal3D;  // normal of the plane in 3D space
    Eigen::Vector3f center2D = Eigen::Vector3f(0, 0, 0);
    Eigen::Vector3f normal2D = Eigen::Vector3f(0, 1, 0);

    Eigen::Matrix4f matrix23D;
    Eigen::Matrix4f matrix22D;
};

#endif // PLANE_H
