#ifndef DIELECTRIC_H
#define DIELECTRIC_H

#include "material.h"

class Dielectric : public Material
{
private:
    double m_ir;
    static double reflectance(double cosine, double refIdx);
public:
    Dielectric(const tinyobj::material_t& mat, bool importanceSampling = false): m_ir(mat.ior), Material(mat, importanceSampling) {
        m_isSpecular = true;
        m_maxRadiance = std::min((double)std::max(this->getSpecularColor()[0], std::max(this->getSpecularColor()[1], this->getSpecularColor()[2])), 0.9);
    };
    Vector3f getColor() const {
        return m_specularColor;
    }
    Vector3f scatter(const Ray &ray, const IntersectionInfo &i, Ray &scattered);
    void getScatteredRay(const Ray &ray, const IntersectionInfo &i, Ray &scattered);
    double samplePDF() const;
    double sampleBRDF(Vector3f in = Vector3f(0, 0, 0), Vector3f n = Vector3f(0, 0, 0), Vector3f out = Vector3f(0, 0, 0)) const;

    mat_type_t getType() const {return MAT_TYPE_DIELECTRIC;};
};

#endif // DIELECTRIC_H
