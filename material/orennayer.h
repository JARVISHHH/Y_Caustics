#ifndef ORENNAYER_H
#define ORENNAYER_H

#include "material.h"

class OrenNayer : public Material
{
private:
    double m_a, m_b;
public:
    OrenNayer(const tinyobj::material_t& mat, bool importanceSampling = false): Material(mat, importanceSampling) {
        m_isSpecular = false;
        m_maxRadiance = std::min((double)std::max(this->getDiffuseColor()[0], std::max(this->getDiffuseColor()[1], this->getDiffuseColor()[2])), 0.9);
        double sigma = mat.shininess > 15.0 ? 0.1 : 0.001;
        double sigma2 = sigma * sigma;
        m_a = 1.0 - (sigma2 / (2.0 * (sigma2 + 0.33)));
        m_b = 0.45 * sigma2 / (sigma2 + 0.09);
    };
    Vector3f scatter(const Ray &ray, const IntersectionInfo &i, Ray &scattered);
    void getScatteredRay(const Ray &ray, const IntersectionInfo &i, Ray &scattered);
    double samplePDF() const;
    double sampleBRDF(Vector3f in, Vector3f n, Vector3f out) const;

    mat_type_t getType() const {return MAT_TYPE_ORENNAYER;};
};

#endif // ORENNAYER_H
