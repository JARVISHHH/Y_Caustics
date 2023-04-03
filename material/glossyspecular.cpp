#include "glossyspecular.h"
#include "BVH/Object.h"
#include "util/random.h"
using namespace Eigen;

Eigen::Vector3f GlossySpecular::scatter(const Ray &ray, const IntersectionInfo &i, Ray &scattered) {
    Vector3f n = i.object->getNormal(i);
    n = n.dot(ray.d) < 0 ? n : -n;

    getScatteredRay(ray, i, scattered);

    m_brdf = sampleBRDF(ray.d.normalized(), n, scattered.d) * this->getDiffuseColor();

    double pdf = samplePDF() * (m_importanceSampling ? abs(n.dot(scattered.d)) : 1.0);

    return m_brdf / pdf * n.dot(scattered.d);
}

void GlossySpecular::getScatteredRay(const Ray &ray, const IntersectionInfo &i, Ray &scattered)
{
    Vector3f n = i.object->getNormal(i);
    n = n.dot(ray.d) < 0 ? n : -n;
    Vector3f w = n;
    Vector3f u = ((std::abs(w.x()) > 0.1 ? Vector3f(0.0, 1.0, 0.0) : Vector3f(1.0, 0.0, 0.0)).cross(w)).normalized();
    Vector3f v = w.cross(u).normalized();
    Vector3f d = m_importanceSampling ? squareToHemisphereCosine() : UniformSampleOnHemisphere();
    d = (u * d[0] + v * d[1] + w * d[2]).normalized();
    scattered = Ray(ray.o + ray.d * i.t + d * 0.0001f, d);
}

double GlossySpecular::samplePDF() const {
    return m_importanceSampling ? (1.0 / M_PI) : (1 / (2 * M_PI));
}

double GlossySpecular::sampleBRDF(Vector3f in, Vector3f n, Vector3f out) const {
    Vector3f reflected = reflect(in, n);
    return (m_shininess + 2.0) / (2 * PI) * pow(reflected.dot(out), m_shininess);
}
