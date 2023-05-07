#ifndef PATHTRACER_H
#define PATHTRACER_H

#include <QImage>
#include <random>

#include "material/material.h"
#include "scene/scene.h"
#include "scene/randomgenerator.h"
#include "photon.h"
#include "photonmapping.h"
#include "stylized/projection/plane.h"
#include "stylized/stylizedcaustics.h"
#include "integrator.h"

struct ImageParameters {
    std::string caustic_img;
    Eigen::Vector2f img_size;
    float img_rotate;
    Eigen::Vector3f img_center;
    Eigen::Vector3f img_normal;
};

class PathTracer
{
public:
    PathTracer(Scene *scene, int width, int height, ImageParameters imageParameters, bool usePhotonMapping, int samplePerPixel, bool defocusBlurOn, bool useOrenNayerBRDF, bool importanceSampling);

    void traceScene(QRgb *imageData, const Scene &scene, float max_dist, int max_num, int min_num, float t = 0);

private:
    Scene *scene;
    int m_width, m_height;

    bool m_usePhotonMapping;
    bool m_defocusBlurOn;
    bool m_useOrenNayerBRDF;
    bool m_importanceSampling;

    PhotonMap pmap_r, pmap_g, pmap_b, pmap_caustic;
    PhotonMapping photonmapper;

    RandomGenerator rng;
    Integrator m_integrator;

    StylizedCaustics stylizedCaustics;
    Plane plane;

    void generatePhotons(const Scene& scene);
    void setIntegrator();

    void toneMap(QRgb *imageData, std::vector<Eigen::Vector3f> &intensityValues);

    Eigen::Vector3f tracePixel(int x, int y, const Scene &scene, const Eigen::Matrix4f &invViewMatrix, float max_dist, int max_num, int min_num);
};

#endif // PATHTRACER_H
