#ifndef GLOSSYSPECULAR_H
#define GLOSSYSPECULAR_H
#include "material/material.h"

class GlossySpecular : public Material {
private:
    double m_shininess;

public:
    GlossySpecular(const tinyobj::material_t& mat, bool importanceSampling = false): m_shininess(mat.shininess), Material(mat, importanceSampling) {
        m_isSpecular = false;
        m_maxRadiance = std::min((double)std::max(this->getDiffuseColor()[0], std::max(this->getDiffuseColor()[1], this->getDiffuseColor()[2])), 0.9);
    };
    Eigen::Vector3f scatter(const Ray &ray, const IntersectionInfo &i, Ray &scattered);
    void getScatteredRay(const Ray &ray, const IntersectionInfo &i, Ray &scattered);
    double samplePDF() const;
    double sampleBRDF(Vector3f in = Vector3f(0, 0, 0), Vector3f n = Vector3f(0, 0, 0), Vector3f out = Vector3f(0, 0, 0)) const;

    mat_type_t getType() const {return MAT_TYPE_GLOSSY;};

};

#endif // GLOSSYSPECULAR_H
