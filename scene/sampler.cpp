#include "sampler.h"
#include "util/random.h"

Vector3f LightSampler::sampleDirect(const Ray &ray, const IntersectionInfo &i, Triangle *light, const Scene& scene, Ray &scatteredLight) const {

    // Pick one light source and divide with probability of selecting light source
    Vector3<Vector3f> vertices = light->getVertices();

    Vector3f lightPos = UniformSampleTriangle(vertices[0], vertices[1], vertices[2]);
    Vector3f shadowDir = (lightPos - i.hit).normalized();
    scatteredLight = Ray(i.hit + 0.01 * shadowDir, shadowDir);

    Vector3f n = i.object->getNormal(i);
    n = n.dot(ray.d) < 0 ? n : -n;
    double cosTheta = scatteredLight.d.dot(n);

    double cosLightTheta = abs(-scatteredLight.d.dot(light->getNormal(lightPos)));
    if (cosLightTheta <= 0.001 || cosTheta <= 0.0) {
        return Vector3f(0, 0, 0);
    }

    IntersectionInfo shadowIntersection;

    if (!scene.getIntersection(scatteredLight, &shadowIntersection) || static_cast<const Triangle *>(shadowIntersection.data) != light) {
        return Vector3f(0, 0, 0);
    }

    double lightPdf = std::pow((lightPos - i.hit).norm(), 2) / (light->getAera() * cosLightTheta * cosTheta);

    return Vector3f(light->getMaterial().emission[0], light->getMaterial().emission[1], light->getMaterial().emission[2]) / lightPdf;
}
