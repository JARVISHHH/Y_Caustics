#ifndef PHOTONMAPPING_H
#define PHOTONMAPPING_H

#include "photon.h"
#include <Eigen>
#include "BVH/Ray.h"
#include "scene/scene.h"
#include "material/material.h"
#include "material/dielectric.h"
#include "material/lambertian.h"
#include "material/glossyspecular.h"
#include "material/mirror.h"

class PhotonMapping
{
public:
    void tracePhoton(PhotonMap &pmap, const Ray& r, const Scene& scene, Eigen::Vector3f lightColor, int depth, int lightIdx);
    void selectMaterial(const tinyobj::material_t& mat, std::shared_ptr<Material> &obj);
    Vector3f getIrradiance(PhotonMap &pmap, const Ray &r, const Scene &scene, int depth);
    void generatePhotonMap(PhotonMap &pmap, const Scene &scene, bool isCaustic = false);
    void tracePhotonCaustic(PhotonMap &pmap, const Ray &r, const Scene &scene, Eigen::Vector3f lightColor, int depth, bool flag, int lightIdx);
};

#endif // PHOTONMAPPING_H
