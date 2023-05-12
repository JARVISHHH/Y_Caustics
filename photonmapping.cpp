#include "photonmapping.h"
#include "util/coordinatesystem.h"
#include "util/random.h"

void PhotonMapping::generatePhotonMap(PhotonMap &pmap, const Scene &scene, bool isCaustic) {
    std::cout << "num_lights = "<<scene.getEmissives().size() << std::endl;
//    for (int i = 0; i < scene.getEmissives().size(); ++i) {
//        Eigen::Vector3<Eigen::Vector3f> vertices = scene.getEmissives()[i]->getVertices();
//        std::cout << "light " << i << std::endl;
//        std::cout << vertices(0) << std::endl;
//        std::cout << vertices(1) << std::endl;
//        std::cout << vertices(2) << std::endl;
//    }
    for (int i = 0; i < scene.getEmissives().size(); ++i) {
        auto light = scene.getEmissives()[i];
        size_t pmapSize = pmap.photons.size();
        while ((pmap.photons.size() - pmapSize) < (float)pmap.maxPhotonNum / scene.getEmissives().size()) {
//            if ((pmap.photons.size() - pmapSize) == 0){
//                std::cout << "pmap.photons.size() - pmapSize = " << (pmap.photons.size() - pmapSize) << std::endl;
//            }

            Vector3<Vector3f> vertices = light->getVertices();
            Vector3f lightPos = UniformSampleTriangle(vertices[0], vertices[1], vertices[2]);
            Vector3f n = light->getNormal(lightPos);
            Vector3f d = UniformSampleOnHemisphere();
            CoordinateSystem onb(n);
            d = onb.from(d).normalized();
            Vector3f color = Vector3f(light->getMaterial().emission[0], light->getMaterial().emission[1], light->getMaterial().emission[2]) / light->getMaterial().emission[0];
            double lightPdf = 1.0f / (light->getAera() * d.dot(n));
            Ray scatteredLight = Ray(lightPos + 0.01 * d, d);
            if (!isCaustic) tracePhoton(pmap, scatteredLight, scene, color / lightPdf, 0, i);
            else tracePhotonCaustic(pmap, scatteredLight, scene, color / lightPdf, 0, false, i);
        }
    }
}

//void PhotonMapping::generatePhotonMap(PhotonMap &pmap, const Scene &scene, bool isCaustic) {
//    int lightNum = scene.getEmissives().size();
//    std::vector<PhotonMap> photonMaps(lightNum, PhotonMap(pmap.maxPhotonNum / lightNum + 1));
////    #pragma omp parallel for
//    for(int i = 0; i < lightNum; i++) {
//        auto light = scene.getEmissives()[i];
//        Vector3<Vector3f> vertices = light->getVertices();
//        Vector3f color = Vector3f(light->getMaterial().emission[0], light->getMaterial().emission[1], light->getMaterial().emission[2]) / light->getMaterial().emission[0];
//        while(photonMaps[i].photons.size() < pmap.maxPhotonNum / lightNum) {
//            int tempSize = pmap.maxPhotonNum / lightNum - photonMaps[i].photons.size();
//            std::vector<PhotonMap> tempMaps(tempSize, PhotonMap(1));
//            #pragma omp parallel for
//            for(int j = 0; j < tempSize; j++) {
//                Vector3f lightPos = UniformSampleTriangle(vertices[0], vertices[1], vertices[2]);
//                Vector3f n = light->getNormal(lightPos);
//                Vector3f d = UniformSampleOnHemisphere();
//                CoordinateSystem onb(n);
//                d = onb.from(d).normalized();
//                double lightPdf = 1.0f / (light->getAera() * d.dot(n));
//                Ray scatteredLight = Ray(lightPos + 0.01 * d, d);
//                if (!isCaustic) tracePhoton(tempMaps[j], scatteredLight, scene, color / lightPdf, 0);
//                else tracePhotonCaustic(tempMaps[j], scatteredLight, scene, color / lightPdf, 0, false);
//            }
//            for(int j = 0; j < tempSize; j++) photonMaps[i].insert(tempMaps[j]);
//        }
//    }
//    for(int i = 0; i < lightNum; i++) pmap.insert(photonMaps[i]);
//}

void PhotonMapping::selectMaterial(const tinyobj::material_t& mat, std::shared_ptr<Material> &obj) {
    if (mat.specular[0] > 0.25 && mat.shininess > 20.0 && mat.shininess < 150.0) {
        obj = std::make_shared<GlossySpecular>(mat);
    } else if (mat.specular[0] > 0.25 && mat.ior > 1.2) {
        obj = std::make_shared<Dielectric>(mat);
    } else if (mat.specular[0] > 0.25 && mat.shininess > 180.0) {
        obj = std::make_shared<Mirror>(mat);
    } else {
        obj = std::make_shared<Lambertian>(mat);
    }

}

void PhotonMapping::tracePhoton(PhotonMap &pmap, const Ray &r, const Scene &scene, Eigen::Vector3f lightColor, int depth, int lightIdx) {
    if (depth > 5) return;

    IntersectionInfo i;
    Ray ray(r);
    if(scene.getIntersection(ray, &i)) {
        const Triangle *t = static_cast<const Triangle *>(i.data);//Get the triangle in the mesh that was intersected
        const tinyobj::material_t& mat = t->getMaterial();//Get the material of the triangle from the mesh
        std::shared_ptr<Material> obj = std::make_shared<Lambertian>(mat);

        selectMaterial(mat, obj);

        Vector3f newOrigin = i.hit;

        Vector3f factor = obj->getColor();

        float maxRefl = obj->m_maxRadiance;

        if (obj->getType() == MAT_TYPE_LAMBERTIAN) {
            if (random_double() > maxRefl) { // absorb the photon
                Photon p;
                p.lastHit = ray.o;
                p.power = Vector3f(factor[0] * lightColor[0] * (1.0f / (1 - maxRefl)),
                                   factor[1] * lightColor[1] * (1.0f / (1 - maxRefl)),
                                   factor[2] * lightColor[2] * (1.0f / (1 - maxRefl))) / pmap.maxPhotonNum;
                p.origin = Vector3f(newOrigin[0], newOrigin[1], newOrigin[2]);
                p.dir = Vector3f(ray.d[0], ray.d[1], ray.d[2]);
                p.lightIdx = lightIdx;
                pmap.store(p);
                return;
            }
        }

        Ray nextRay(ray);
        obj->getScatteredRay(ray, i, nextRay);
        tracePhoton(pmap, nextRay, scene, piecewiseMul(factor, lightColor) * (1.0f / maxRefl), depth + 1, lightIdx);
    }

}

void PhotonMapping::tracePhotonCaustic(PhotonMap &pmap, const Ray &r, const Scene &scene, Eigen::Vector3f lightColor, int depth, bool flag, int lightIdx) {
    if (depth > 5) return;

    IntersectionInfo i;
    Ray ray(r);
    if(scene.getIntersection(ray, &i)) {
        const Triangle *t = static_cast<const Triangle *>(i.data);//Get the triangle in the mesh that was intersected
        const tinyobj::material_t& mat = t->getMaterial();//Get the material of the triangle from the mesh
        std::shared_ptr<Material> obj = std::make_shared<Lambertian>(mat);

        selectMaterial(mat, obj);

        if (!flag && obj->getType() != MAT_TYPE_DIELECTRIC) return;

        Vector3f newOrigin = i.hit;

        Vector3f factor = obj->getColor();

        float maxRefl = obj->m_maxRadiance;

        if (obj->getType() == MAT_TYPE_LAMBERTIAN) {
            if (random_double() > maxRefl) { // absorb the photon
                Photon p;
                p.lastHit = ray.o;
                p.power = Vector3f(factor[0] * lightColor[0] * (1.0f / (1 - maxRefl)),
                                   factor[1] * lightColor[1] * (1.0f / (1 - maxRefl)),
                                   factor[2] * lightColor[2] * (1.0f / (1 - maxRefl))) / pmap.maxPhotonNum;
                p.origin = Vector3f(newOrigin[0], newOrigin[1], newOrigin[2]);
                p.dir = Vector3f(ray.d[0], ray.d[1], ray.d[2]);
                p.lightIdx = lightIdx;
                pmap.store(p);
                return;
            }
        }


        Ray nextRay(ray);
        obj->getScatteredRay(ray, i, nextRay);
        tracePhotonCaustic(pmap, nextRay, scene, piecewiseMul(factor, lightColor) * (1.0f / maxRefl), depth + 1, obj->getType() == MAT_TYPE_DIELECTRIC, lightIdx);
    }

}

Vector3f PhotonMapping::getIrradiance(PhotonMap &pmap, const Ray &r, const Scene &scene, int depth) {
    if (depth > 5) return Vector3f(0, 0, 0);

    IntersectionInfo i;
    Ray ray(r);
    if(scene.getIntersection(ray, &i)) {
        const Triangle *t = static_cast<const Triangle *>(i.data);//Get the triangle in the mesh that was intersected
        const tinyobj::material_t& mat = t->getMaterial();//Get the material of the triangle from the mesh
        std::shared_ptr<Material> obj = std::make_shared<Lambertian>(mat);

        selectMaterial(mat, obj);

        Vector3f newOrigin = i.hit;
        Vector3f n = i.object->getNormal(i);

        Vector3f factor = obj->getColor();

        if (obj->getType() == MAT_TYPE_LAMBERTIAN) {

            // direct visualization of the photon map

            Vector3f pos = {newOrigin[0], newOrigin[1], newOrigin[2]};
            Vector3f normal = {n[0], n[1], n[2]};
            Vector3f col = pmap.getFixedRadiusIrradiance(pos, normal, 0.1, 100, 5);
            return col;
        } else {
            Ray nextRay(ray);
            obj->getScatteredRay(ray, i, nextRay);
            return obj->getEmissiveColor() + piecewiseMul(factor, getIrradiance(pmap, nextRay, scene, depth + 1));
        }
    }
    return Vector3f(0.0, 0.0, 0.0);
}
