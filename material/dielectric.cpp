#include "dielectric.h"
#include "BVH/Object.h"
#include "util/random.h"

double Dielectric::reflectance(double cosine, double refIdx) {
    auto r0 = (1 - refIdx) / (1 + refIdx);
    r0 = r0 * r0;
    return r0 + (1 - r0) * pow((1 - cosine), 5);
}

Vector3f Dielectric::scatter(const Ray &ray, const IntersectionInfo &i, Ray &scattered) {
    getScatteredRay(ray, i, scattered);
    double pdf = samplePDF();
    return m_brdf / pdf;
}

void Dielectric::getScatteredRay(const Ray &ray, const IntersectionInfo &i, Ray &scattered) {
    Vector3f outwardN = i.object->getNormal(i);
    Vector3f n = outwardN.dot(ray.d) < 0 ? outwardN : -outwardN;
    double refractionRatio = outwardN.dot(ray.d) < 0 ? (1.0 / m_ir) : m_ir;

    double cosTheta = -ray.d.dot(n);
    double sinTheta = sqrt(1.0 - cosTheta * cosTheta);

    bool cannotRefract = refractionRatio * sinTheta > 1.0;
    Vector3f direction;

    if (cannotRefract || reflectance(cosTheta, refractionRatio) > random_double()) {
        direction = reflect(ray.d, n);
        m_brdf = this->getSpecularColor();
    } else {
        direction = refract(ray.d, n, refractionRatio).normalized();
        m_brdf = Vector3f(1.0, 1.0, 1.0);
    }

    scattered = Ray(i.hit + direction * 0.001f, direction);
}

double Dielectric::samplePDF() const {
    return 1.0;
}

double Dielectric::sampleBRDF(Vector3f in, Vector3f n, Vector3f out) const {
    return 1.0;
}
