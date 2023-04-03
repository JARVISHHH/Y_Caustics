#include "pathtracer.h"
#include "material/dielectric.h"
#include "material/lambertian.h"
#include "material/glossyspecular.h"
#include "material/mirror.h"
#include "material/orennayer.h"
#include "scene/sampler.h"

#include <iostream>

#include <Eigen/Dense>

#include <util/CS123Common.h>
#include <util/random.h>

using namespace Eigen;

const double albedo = 0.75;
PathTracer::PathTracer(int width, int height, bool usePhotonMapping, int samplePerPixel, bool defocusBlurOn, bool useOrenNayerBRDF, bool importanceSampling)
    : m_width(width), m_height(height), m_usePhotonMapping(usePhotonMapping), m_defocusBlurOn(defocusBlurOn), m_useOrenNayerBRDF(useOrenNayerBRDF), m_importanceSampling(importanceSampling), rng(samplePerPixel, time(NULL))
{}

void PathTracer::traceScene(QRgb *imageData, const Scene& scene)
{
    if (m_usePhotonMapping) generatePhotons(scene);
    std::vector<Vector3f> intensityValues(m_width * m_height);
    Matrix4f invViewMat = (scene.getCamera().getScaleMatrix() * scene.getCamera().getViewMatrix()).inverse();
    for(int y = 0; y < m_height; ++y) {
        std::cerr << "\rScanlines remaining: " << m_height - y << ' ' << std::flush;
//        #pragma omp parallel for
        for(int x = 0; x < m_width; ++x) {
            int offset = x + (y * m_width);
            intensityValues[offset] = tracePixel(x, y, scene, invViewMat);
        }
    }

    toneMap(imageData, intensityValues);
}

void PathTracer::generatePhotons(const Scene& scene) {
    photonmapper.generatePhotonMap(pmap_r, scene);
    pmap_r.balance();
    std::cout<<"finish generating photon map, size: "<<pmap_r.photons.size()<<std::endl;
//    pmap_caustic.maxPhotonNum = 20000;
    photonmapper.generatePhotonMap(pmap_caustic, scene, true);
    pmap_caustic.balance();
    std::cout<<"finish generating caustic photon map, size: "<<pmap_caustic.photons.size()<<std::endl;

}

Vector3f PathTracer::tracePixel(int x, int y, const Scene& scene, const Matrix4f &invViewMatrix)
{
    Vector3f pixelColor(0.0, 0.0, 0.0);

    std::vector<Vector2f> pixelSamples = rng.GenerateStratifiedSamples();

    for (int s = 0; s < rng.m_samplesPerPixel; ++s) {
        Vector3f p(0, 0, 0);
        Vector3f d((2.f * (double(x) + pixelSamples[s][0]) / m_width) - 1, 1 - (2.f * (double(y) + pixelSamples[s][1]) / m_height), -1);
        d.normalize();
        Ray r(p, d);

        double lensRadius = 0.5f;
        double focalDistance = 3.5f;

        if (m_defocusBlurOn && lensRadius > 0.0f) {
            //picking a samplepoint on the lens and then changing it according to the lens properties
            Vector3f pLens = lensRadius * random_in_unit_disk();
            Vector3f pFocus = focalDistance * r.d + r.o;
            Vector3f aperaturePoint = r.o + (scene.getCamera().m_u * pLens[1]) + (scene.getCamera().m_s * pLens[0]);

            r.o = aperaturePoint;
            r.d = (pFocus - aperaturePoint).normalized();
        }

        r = r.transform(invViewMatrix);
        Vector3f color = m_usePhotonMapping ? traceRayWithPhotonMapping(r, scene, 0, true) * 2.5 : traceRayWithPathTracing(r, scene, 0, true) * 2.5;
        pixelColor += Vector3f(color[0] / (1 + color[0]), color[1] / (1 + color[1]), color[2] / (1 + color[2]));
    }
//    pixelColor = Vector3f(pixelColor[0] / (1 + pixelColor[0]), pixelColor[1] / (1 + pixelColor[1]), pixelColor[2] / (1 + pixelColor[2]));
    return pixelColor / (double)(rng.m_samplesPerPixel);
}

void PathTracer::selectMaterial(const tinyobj::material_t& mat, std::shared_ptr<Material> &obj) {

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

Vector3f PathTracer::traceRayWithPhotonMapping(const Ray& r, const Scene& scene, int depth, bool countEmitted) {
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
            //return sampleColor;
            // russian roulette
            double continue_probability = obj->m_maxRadiance;
            if (random_double(0.0, 1.0) >= continue_probability) {
                return sampleColor;
            }
            factor /= continue_probability;
        }

        if (obj->getType() == MAT_TYPE_LAMBERTIAN) {

            Vector3f pos = {i.hit[0], i.hit[1], i.hit[2]};
            Vector3f normal = {n[0], n[1], n[2]};
            Vector3f col = pmap_caustic.getIrradiance(pos, normal, 0.1, 100) * 100;
            sampleColor += col;

            int nsamps = 1;
            for (int ss = 0; ss < nsamps ; ss++) {
                Ray nextRay(ray);
                obj->getScatteredRay(ray, i, nextRay);
                sampleColor += photonmapper.getIrradiance(pmap_r, nextRay, scene, 0) * (1.0f / 1.0f);
            }
        } else {
            Ray nextRay(ray);
            obj->getScatteredRay(ray, i, nextRay);
            Vector3f nextRadiance = this->traceRayWithPhotonMapping(nextRay, scene, depth + 1, obj->m_isSpecular);
            sampleColor += (obj->getColor() * factor, nextRadiance);
        }

        return sampleColor;

    } else {
        return Vector3f(0, 0, 0);
    }
}

Vector3f PathTracer::traceRayWithPathTracing(const Ray& r, const Scene& scene, int depth, bool countEmitted) {
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



void PathTracer::toneMap(QRgb *imageData, std::vector<Vector3f> &intensityValues) {
    Vector3f maxColor(0, 0, 0);
    for(int y = 0; y < m_height; ++y) {
        for(int x = 0; x < m_width; ++x) {
            int offset = x + (y * m_width);
            maxColor[0] = std::max(maxColor[0], intensityValues[offset][0]);
            maxColor[1] = std::max(maxColor[1], intensityValues[offset][1]);
            maxColor[2] = std::max(maxColor[2], intensityValues[offset][2]);
            imageData[offset] = qRgb(255 * intensityValues[offset][0], 255 * intensityValues[offset][1], 255 * intensityValues[offset][2]);
        }
    }
    std::cout<<maxColor[0]<<" "<<maxColor[1]<<" "<<maxColor[2]<<std::endl;
}
