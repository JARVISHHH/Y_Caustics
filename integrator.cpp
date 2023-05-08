#include "integrator.h"
#include "material/orennayer.h"


void Integrator::selectMaterial(const tinyobj::material_t& mat, std::shared_ptr<Material> &obj) {
    if (mat.specular[0] > 0.25 && mat.shininess > 20.0 && mat.shininess < 150.0) {
        obj = std::make_shared<GlossySpecular>(mat);
    } else if (mat.specular[0] > 0.25 && mat.ior > 1.2) {
        obj = std::make_shared<Dielectric>(mat);
    } else if (mat.specular[0] > 0.25 && mat.shininess > 180.0) {
        obj = std::make_shared<Mirror>(mat);
    } else if (m_useOrenNayerBRDF){
       obj = std::make_shared<OrenNayer>(mat, m_importanceSampling);
    } else {
        obj = std::make_shared<Lambertian>(mat, m_importanceSampling);
    }
}

void Integrator::setPmapCaustic(const PhotonMap &newPmap_caustic) {
    pmap_caustic = newPmap_caustic;
}

void Integrator::setPmap(const PhotonMap &newPmap) {
    pmap = newPmap;
}

Vector3f Integrator::debugPhotonMap(const Ray& r, const Scene& scene) {
    IntersectionInfo i;
    Ray ray(r);
    if(scene.getIntersection(ray, &i)) {
        Vector3f pos = {i.hit[0], i.hit[1], i.hit[2]};
//        return pmap.visualizePhotonMap(pos, 0.01, 10);
        const Triangle *t = static_cast<const Triangle *>(i.data);//Get the triangle in the mesh that was intersected
        const tinyobj::material_t& mat = t->getMaterial();//Get the material of the triangle from the mesh
        std::shared_ptr<Material> obj = std::make_shared<Lambertian>(mat);
        selectMaterial(mat, obj);

        Vector3f n = i.object->getNormal(i);
        n = n.dot(ray.d) < 0 ? n : -n;

        Vector3f col = pmap.getGaussianIrradiance(pos, n, 0.02, 20, 5);
        col += pmap_caustic.getGaussianIrradiance(pos, n, 0.003, 30, 10);
        return col;
    }
    return Vector3f(0.0, 0.0, 0.0);
}

Vector3f Integrator::traceRayWithPhotonMapping(const Ray& r, const Scene& scene, int depth, bool countEmitted, float max_dist, int max_num, int min_num) {
    IntersectionInfo i;
    Ray ray(r);
    if(scene.getIntersection(ray, &i)) {
        const Triangle *t = static_cast<const Triangle *>(i.data); // Get the triangle in the mesh that was intersected
        const tinyobj::material_t& mat = t->getMaterial(); // Get the material of the triangle from the mesh
        std::shared_ptr<Material> obj = std::make_shared<Lambertian>(mat);

        selectMaterial(mat, obj);  // Decide the material that the ray hit

        double factor = 1.0;

        // Get the triangle normal
        Vector3f n = i.object->getNormal(i);
        n = n.dot(ray.d) < 0 ? n : -n;

        Ray scatteredLight(ray);
        LightSampler sampler;
        Vector3f sampleColor(0, 0, 0);

        // Direct lighting
        for (auto light: scene.getEmissives()) {
            Vector3f lightColor = sampler.sampleDirect(ray, i, light, scene, scatteredLight);
            Vector3f objColor = obj->sampleBRDF(ray.d, n, scatteredLight.d) * obj->getDiffuseColor();
            sampleColor += 2.0 * piecewiseMul(lightColor, objColor);
        }
        // Self-emitting radiance
        if (countEmitted) {
            sampleColor += obj->getEmissiveColor();
        }

        // russian roulette
        if (depth >= 10) {
            //return sampleColor;
            double continue_probability = obj->m_maxRadiance;
            if (random_double(0.0, 1.0) >= continue_probability) {
                return sampleColor;
            }
            factor /= continue_probability;
        }

        if (obj->getType() == MAT_TYPE_LAMBERTIAN) {
            Vector3f pos = {i.hit[0], i.hit[1], i.hit[2]};
            Vector3f normal = {n[0], n[1], n[2]};
//            sampleColor += pmap_caustic.getGaussianIrradianceWithFixedNum(pos, normal, max_num, min_num);
            sampleColor += pmap_caustic.getGaussianIrradiance(pos, normal, max_dist, max_num, min_num);
//            sampleColor += pmap.getGaussianIrradiance(pos, normal, 0.01, 30, 3);
        } else {
            Ray nextRay(ray);
            obj->getScatteredRay(ray, i, nextRay);
            Vector3f nextRadiance = this->traceRayWithPhotonMapping(nextRay, scene, depth + 1, obj->m_isSpecular, max_dist, max_num, min_num);
            sampleColor += (obj->getColor() * factor, nextRadiance);
        }

        return sampleColor;

    } else {
        return Vector3f(0, 0, 0);
    }
}

Vector3f Integrator::traceRayWithPathTracing(const Ray& r, const Scene& scene, int depth, bool countEmitted) {
    IntersectionInfo i;
    Ray ray(r);
    if(scene.getIntersection(ray, &i)) {
          //** Example code for accessing materials provided by a .mtl file **
        const Triangle *t = static_cast<const Triangle *>(i.data);//Get the triangle in the mesh that was intersected
        const tinyobj::material_t& mat = t->getMaterial();//Get the material of the triangle from the mesh
        std::shared_ptr<Material> obj = std::make_shared<Lambertian>(mat);

        selectMaterial(mat, obj);

        double factor = 1.0;

        Vector3f n = i.object->getNormal(i);
        n = n.dot(ray.d) < 0 ? n : -n;

        Ray scatteredLight(ray);
        LightSampler sampler;
        Vector3f sampleColor(0, 0, 0);

        for (auto light: scene.getEmissives()) {
            Vector3f lightColor = sampler.sampleDirect(ray, i, light, scene, scatteredLight);
            Vector3f objColor = obj->sampleBRDF(ray.d, n, scatteredLight.d) * obj->getDiffuseColor();
            sampleColor += piecewiseMul(lightColor, objColor);
        }

        if (countEmitted) {
            sampleColor += obj->getEmissiveColor();
        }

        if (depth >= 10) {
            // russian roulette
            double continue_probability = obj->m_maxRadiance;
            if (random_double(0.0, 1.0) >= continue_probability) {
                return sampleColor;
            }
            factor /= continue_probability;
        }

        Ray nextRay(ray);
        Vector3f res = obj->scatter(ray, i, nextRay) * factor;
        Vector3f nextRadiance = this->traceRayWithPathTracing(nextRay, scene, depth + 1, obj->m_isSpecular);

        return sampleColor + Vector3f(res[0] * nextRadiance[0], res[1] * nextRadiance[1], res[2] * nextRadiance[2]);

    } else {
        return Vector3f(0, 0, 0);
    }
}
