#ifndef MATERIAL_H
#define MATERIAL_H

#include <Eigen/Dense>
#include "BVH/IntersectionInfo.h"
#include "BVH/Ray.h"
#include "util/tiny_obj_loader.h"
using namespace Eigen;

typedef enum {
  MAT_TYPE_LAMBERTIAN,  // default
  MAT_TYPE_GLOSSY,
  MAT_TYPE_MIRROR,
  MAT_TYPE_DIELECTRIC,
  MAT_TYPE_ORENNAYER,
} mat_type_t;

class Material {
protected:
    Vector3f m_diffuseColor;
    Vector3f m_specularColor;
    Vector3f m_emissiveColor;


public:
    bool m_isSpecular;
    double m_maxRadiance;
    bool m_importanceSampling;
    Vector3f m_brdf;
    Material(const tinyobj::material_t& mat, bool importanceSampling) : m_importanceSampling(importanceSampling) {
        m_diffuseColor = Vector3f(mat.diffuse[0], mat.diffuse[1], mat.diffuse[2]);
        m_specularColor = Vector3f(mat.specular[0], mat.specular[1], mat.specular[2]);
        m_emissiveColor = Vector3f(mat.emission[0], mat.emission[1], mat.emission[2]);
    };

    virtual Eigen::Vector3f scatter(const Ray &ray, const IntersectionInfo &i, Ray &scattered) {
        return Eigen::Vector3f(0, 0, 0);
    }

    virtual void getScatteredRay(const Ray &ray, const IntersectionInfo &i, Ray &scattered) = 0;

    virtual Eigen::Vector3f getColor() const {
        return m_diffuseColor;
    }

    virtual Eigen::Vector3f getDiffuseColor() const {
        return m_diffuseColor;
    }

    virtual Eigen::Vector3f getSpecularColor() const {
        return m_specularColor;
    }

    virtual Eigen::Vector3f getEmissiveColor() const {
        return m_emissiveColor;
    }

    virtual double samplePDF() const = 0;

    virtual double sampleBRDF(Vector3f in = Vector3f(0, 0, 0), Vector3f n = Vector3f(0, 0, 0), Vector3f out = Vector3f(0, 0, 0)) const {
        return 0.0;
    }

    virtual mat_type_t getType() const = 0;

};

#endif // MATERIAL_H
