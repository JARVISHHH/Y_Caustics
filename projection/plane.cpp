#include "plane.h"

Plane::Plane(Eigen::Vector3f origin, Eigen::Vector3f normal)
    :origin(origin), normal(normal)
{

}

Eigen::Vector3f Plane::project(const Eigen::Vector3f& pointOrigin, const Eigen::Vector3f& point3D) {
    auto d = point3D - pointOrigin;
    assert(d.dot(normal) != 0);
    auto t = -(origin - pointOrigin).dot(normal) / d.dot(normal);
    assert(t >= 0);

    return pointOrigin + t * d;
}

Eigen::Vector3f Plane::backProject(const Scene& scene, const Eigen::Vector3f& pointOrigin, const Eigen::Vector3f& point2D) {
    IntersectionInfo i;
    Ray ray(pointOrigin, point2D - pointOrigin);
    bool intersect = scene.getIntersection(ray, &i);
    assert(intersect);
    return i.hit;
}
