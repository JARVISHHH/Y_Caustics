#include "orennayer.h"
#include "BVH/Object.h"
#include "util/coordinatesystem.h"
#include "util/random.h"


Vector3f OrenNayer::scatter(const Ray &ray, const IntersectionInfo &i, Ray &scattered) {

    Vector3f n = i.object->getNormal(i);
    n = n.dot(ray.d) < 0 ? n : -n;

    getScatteredRay(ray, i, scattered);

    m_brdf = sampleBRDF(ray.d.normalized(), n, scattered.d.normalized()) * this->getDiffuseColor();

    double pdf = samplePDF() * (m_importanceSampling ? abs(n.dot(scattered.d)) : 1.0);

    return m_brdf / pdf * abs(n.dot(scattered.d));
}

void OrenNayer::getScatteredRay(const Ray &ray, const IntersectionInfo &i, Ray &scattered)
{
    Vector3f n = i.object->getNormal(i);
    n = n.dot(ray.d) < 0 ? n : -n;

    Vector3f out = m_importanceSampling ? squareToHemisphereCosine() : UniformSampleOnHemisphere();
    CoordinateSystem onb(n);
    Vector3f d = onb.from(out).normalized();
    scattered = Ray(i.hit, d);
}

double OrenNayer::samplePDF() const {
    return m_importanceSampling ? (1.0 / M_PI) : (1.0 / (2 * M_PI));
}

double OrenNayer::sampleBRDF(Vector3f in, Vector3f n, Vector3f out) const {
    CoordinateSystem onb(n);
    Vector3f wi = onb.to(in);
    Vector3f wo = onb.to(out);

    float sinThetaI = SinTheta(wi);
    float sinThetaO = SinTheta(wo);

    //compute cosine term
    float maxCos = 0.0f;
    if(sinThetaI > 0.0001f && sinThetaO > 0.0001f) {
        float cosPhiI = std::acos(wi[0]);
        float cosPhiO = std::acos(wo[0]);
        float sinPhiI = std::sqrt(1.0f - cosPhiI * cosPhiI);
        float sinPhiO = std::sqrt(1.0f - cosPhiO * cosPhiO);
        float dCos = cosPhiI * cosPhiO + sinPhiI * sinPhiO;
        maxCos = std::max(0.0f, dCos);
    }

    //compute sine and tangent terms
    float sinAlpha, tanBeta;
    if(AbsCosTheta(wi) > AbsCosTheta(wo)) {
        sinAlpha = sinThetaO;
        tanBeta = sinThetaI / AbsCosTheta(wi);
    } else {
        sinAlpha = sinThetaI;
        tanBeta = sinThetaO / AbsCosTheta(wo);
    }

    return (m_a + m_b * maxCos * sinAlpha * tanBeta) / M_PI;
}
