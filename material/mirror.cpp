#include "mirror.h"
#include "BVH/Object.h"

Vector3f Mirror::scatter(const Ray &ray, const IntersectionInfo &i, Ray &scattered) {

    getScatteredRay(ray, i, scattered);

    m_brdf = sampleBRDF() * this->getSpecularColor();

    double pdf = samplePDF();

    return m_brdf / pdf;
}

void Mirror::getScatteredRay(const Ray &ray, const IntersectionInfo &i, Ray &scattered) {
    Vector3f n = i.object->getNormal(i);
    n = n.dot(ray.d) < 0 ? n : -n;

    Vector3f d = reflect(ray.d, n);
    scattered = Ray(i.hit + d * 0.001f, d);
}

double Mirror::samplePDF() const {
    return 1.0;
}

double Mirror::sampleBRDF(Vector3f in, Vector3f n, Vector3f out) const {
    return 1.0;
}
