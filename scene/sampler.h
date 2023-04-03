#ifndef SAMPLER_H
#define SAMPLER_H
#include <BVH/IntersectionInfo.h>
#include <scene/shape/triangle.h>
#include <scene/scene.h>

#include <Eigen>

using namespace Eigen;

class LightSampler
{
public:
    Vector3f sampleDirect(const Ray &ray, const IntersectionInfo &i,  Triangle *light, const Scene& scene, Ray &scatteredLight) const;
};

#endif // SAMPLER_H
