#ifndef MIRROR_H
#define MIRROR_H

#include "material.h"

class Mirror : public Material
{
public:
    Mirror(const tinyobj::material_t& mat, bool importanceSampling = false): Material(mat, importanceSampling) {
        m_isSpecular = true;
        m_maxRadiance = std::min((double)std::max(this->getSpecularColor()[0], std::max(this->getSpecularColor()[1], this->getSpecularColor()[2])), 0.9);
    };

    // Material interface
public:
    Vector3f scatter(const Ray &ray, const IntersectionInfo &i, Ray &scattered);
    void getScatteredRay(const Ray &ray, const IntersectionInfo &i, Ray &scattered);
    double samplePDF() const;
    double sampleBRDF(Vector3f in = Vector3f(0, 0, 0), Vector3f n = Vector3f(0, 0, 0), Vector3f out = Vector3f(0, 0, 0)) const;

    mat_type_t getType() const {return MAT_TYPE_MIRROR;};
};

#endif // MIRROR_H
