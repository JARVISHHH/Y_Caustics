#include "lambertian.h"
#include "BVH/Object.h"
#include "util/random.h"
#include "material.h"
#include "util/coordinatesystem.h"

Vector3f Lambertian::scatter(const Ray &ray, const IntersectionInfo &i, Ray &scattered) {
    Vector3f n = i.object->getNormal(i);
    n = n.dot(ray.d) < 0 ? n : -n;
    getScatteredRay(ray, i, scattered);

    m_brdf = this->getDiffuseColor() * sampleBRDF();

    double pdf = samplePDF() * (m_importanceSampling ? abs(n.dot(scattered.d)) : 1.0);

    return m_brdf / pdf * abs(n.dot(scattered.d));
}

void Lambertian::getScatteredRay(const Ray &ray, const IntersectionInfo &i, Ray &scattered) {
    Vector3f n = i.object->getNormal(i);
    n = n.dot(ray.d) < 0 ? n : -n;
    Vector3f d = m_importanceSampling ? squareToHemisphereCosine() : UniformSampleOnHemisphere();

    CoordinateSystem onb(n);
    d = onb.from(d).normalized();

    scattered = Ray(i.hit, d);
}

double Lambertian::samplePDF() const {
    return m_importanceSampling ? (1.0 / M_PI) : (1 / (2 * M_PI));
}

double Lambertian::sampleBRDF(Eigen::Vector3f in, Eigen::Vector3f n, Eigen::Vector3f out) const {
    return 1.0 / M_PI;
}
