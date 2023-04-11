#include "plane.h"

Plane::Plane(Eigen::Vector3f center3D, Eigen::Vector3f normal3D)
    :center3D(center3D), normal3D(normal3D)
{
    auto theta = std::acos(normal2D.dot(normal3D));
    auto axis = normal2D.cross(normal3D).normalized();
    Eigen::Matrix3f rotate = Eigen::AngleAxisf(theta, axis).matrix();
    Eigen::Vector3f translate = center3D - center2D;
    matrix23D.setIdentity();
    matrix23D.block<3, 3>(0, 0) = rotate;
    matrix23D.block<3, 1>(0, 3) = translate;

    matrix22D = matrix23D.inverse();
}

Eigen::Vector2f Plane::project(const Eigen::Vector3f& pointOrigin, const Eigen::Vector3f& point3D) {
    // pointOrigin is the photon origin in 3D space
    // point3D is the photon destination in 3D space
    // return a 2D point in plane space

    // Get the intersection with the plane in 3D space
    auto d = point3D - pointOrigin;
    assert(d.dot(normal) != 0);
    auto t = (center3D - pointOrigin).dot(normal3D) / d.dot(normal3D);
    assert(t >= 0);

    auto point = pointOrigin + t * d;
//    std::cout << "point " << point[0] << " " << point[1] << " " << point[2] << std::endl;
    Eigen::Vector4f p = {point[0], point[1], point[2], 1};
    p = matrix22D * p;

    assert(std::abs(p[2]) < 0.00005);

    return {p[0], p[1]};
}

Eigen::Vector3f Plane::backProject(const Scene& scene, const Eigen::Vector3f& pointOrigin, const Eigen::Vector2f& point2D) {
    // scene is the scene
    // pointOrigin is the photon origin in 3D space
    // point2D is the 2D point in plane space
    IntersectionInfo i;
    // return the point beck to 3D space
    Eigen::Vector4f point = {point2D[0], point2D[1], 0, 1};
    point = matrix23D * point;
//    std::cout << "point3D " << point[0] << " " << point[1] << " " << point[2] << std::endl;
    Eigen::Vector3f point3D = {point[0], point[1], point[2]};
    // Get the new ray
    Ray ray(pointOrigin, point3D - pointOrigin);
    // Find the new intersection
    bool intersect = scene.getIntersection(ray, &i);
    assert(intersect);
    return i.hit;
}