#ifndef INTEGRATOR_H
#define INTEGRATOR_H
#include "material/material.h"
#include "scene/scene.h"
#include "photon.h"
#include "photonmapping.h"
#include "scene/sampler.h"
#include <util/random.h>

using namespace Eigen;

class Integrator
{
private:
    bool m_importanceSampling;
    bool m_useOrenNayerBRDF;
    void selectMaterial(const tinyobj::material_t& mat, std::shared_ptr<Material> &obj);
    PhotonMap pmap, pmap_caustic;
public:
    Integrator(bool importanceSampling, bool useOrenNayerBRDF): m_importanceSampling(importanceSampling), m_useOrenNayerBRDF(useOrenNayerBRDF) {};
    Vector3f debugPhotonMap(const Ray& r, const Scene& scene);
    Vector3f traceRayWithPhotonMapping(const Ray& r, const Scene& scene, int depth, bool countEmitted, float max_dist, int max_num, int min_num);
    Vector3f traceRayWithPathTracing(const Ray& r, const Scene& scene, int depth, bool countEmitted);
    void setPmap(const PhotonMap &newPmap);
    void setPmapCaustic(const PhotonMap &newPmap_caustic);
};

#endif // INTEGRATOR_H
