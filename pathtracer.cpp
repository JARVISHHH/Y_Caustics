#include "pathtracer.h"
#include "stylized/projection/plane.h"
#include "tonemapper.h"

#include <iostream>

#include <Eigen/Dense>

#include <util/CS123Common.h>
#include <util/random.h>

#include "tps.h"

using namespace Eigen;

bool doStylizedCaustics = true;
bool useGreedyMethod = true;

extern int maxPhotonsNum;

const double albedo = 0.75;
PathTracer::PathTracer(Scene *scene,
                       int width,
                       int height,
                       bool usePhotonMapping,
                       int samplePerPixel,
                       bool defocusBlurOn,
                       bool useOrenNayerBRDF,
                       bool importanceSampling) : scene(scene),
                                                  m_width(width),
                                                  m_height(height),
                                                  m_usePhotonMapping(usePhotonMapping),
                                                  m_defocusBlurOn(defocusBlurOn),
                                                  m_useOrenNayerBRDF(useOrenNayerBRDF),
                                                  m_importanceSampling(importanceSampling),
                                                  rng(samplePerPixel, time(NULL)),
                                                  m_integrator(importanceSampling, useOrenNayerBRDF)
{
    // First pass of photon mapping
    if (m_usePhotonMapping) {
        generatePhotons(*scene);
        std::cout << "light size: " << scene->getEmissives().size() << std::endl;
        if(doStylizedCaustics) {
            auto imageParameters = scene->getImageParameters();
            stylizedCausticsVector.resize(imageParameters.size());
            planeVector.resize(imageParameters.size());
            std::cout << "parameters size: " << imageParameters.size() << std::endl;
            for(int i = 0; i < imageParameters.size(); i++) {
                auto imageParameter = imageParameters[i];
                stylizedCausticsVector[i] = StylizedCaustics(imageParameter);
                auto& stylizedCaustics = stylizedCausticsVector[i];
                // Sample images
                auto imageSamples = stylizedCaustics.sample(imageParameter->caustic_img);
                // Set plane
                std::cout << "index end: " << imageParameter->lightIndexEnd << std::endl;
                planeVector[i] = Plane(imageParameter->img_rotate, imageParameter->img_center, imageParameter->img_normal.normalized());
                auto& plane = planeVector[i];
                std::cout << "created plane" << std::endl;
                // Projection
                auto& photons = pmap_caustic.photons;
                // (1) calculate average origin
                stylizedCaustics.calculateAverageOrigin(photons);
                std::cout << "calculated origin" << std::endl;
                // (2) calculate 2d coordinates
                stylizedCaustics.project(*scene, photons, plane);
                std::cout << "Finished projection" << std::endl;
                // Assign (Greedy algorithm)
                stylizedCaustics.assign(imageSamples);
                std::cout << "Finished assign" << std::endl;

                //For Yingtong
                //You have set A (n points), and you sample 300 points from A
                //Let the positions of the 300 points be std::vector<Eigen::Vector2f> A_sample;

                //You run the greedy algorithm and find the new 2D position of the 300 points,
                //Let the new positions of the 300 points be std::vector<Eigen::Vector2f> B_sample;

                std::vector<Eigen::Vector2f> A_sample = stylizedCaustics.getSubsetSourcesPos();
                std::vector<Eigen::Vector2f> B_sample = stylizedCaustics.getSubsetTargetsPos();

                std::vector<Eigen::Vector3d> A_sample_tps;
                std::vector<Eigen::Vector3d> B_sample_tps;

                for(int i=0; i<A_sample.size(); i++){
                    A_sample_tps.push_back(Eigen::Vector3d(A_sample[i][0], A_sample[i][1],0));
                    B_sample_tps.push_back(Eigen::Vector3d(B_sample[i][0], B_sample[i][1],0));
                }
                //Do TPS here
                //step 1: init tps
                ThinPlateSpline m_tps(A_sample_tps, B_sample_tps);

                //step 2: use tps to find the position of rest n-300 points
                //Let std::vector<Eigen::Vector2f> A_rest be the position of these n-300 points
                //Let std::vector<Eigen::Vector2f> B_rest be the target position of these n-300 points we want to solve

                std::vector<Eigen::Vector2f> A = stylizedCaustics.getSources();
                std::vector<Eigen::Vector2f> B(A.size());
                std::cout <<"A.size() " << A.size() << std::endl;
                for(int i = 0; i< A.size(); i++){
                    Eigen::Vector3d tempt(A[i][0], A[i][1], 0);
                    Eigen::Vector3d result = m_tps.interpolate(tempt);
                    B[i] = Eigen::Vector2f(result[0], result[1]);
                }
                stylizedCaustics.setTpsResults(B);
                std::cout << "Finished interpolation " << std::endl;

                //For Yutang
                // Note from Yingtong: I didn't declare B_rest above, stylizedCaustics.finalResults contains all n points. I can modify the code if it's not convenient.
                //Now you have std::vector<Eigen::Vector2f> B_sample, containing 300 points
                //and std::vector<Eigen::Vector2f> B_rest containing n-300 points
                //Do refinement here
                // Note from Yingtong: Guess refinement is better to be a member function of class StylizedCaustics, since all data is stored in StylizedCaustics.
                // or make the stylizedCaustics a parameter of the function
                // To show the final result, go to move() function in stylizedcaustics.cpp, uncomment the final results code, and comment the results after tps
                stylizedCaustics.refine(stylizedCaustics.tpsResults);
//                stylizedCaustics.refine_random(stylizedCaustics.tpsResults);
//                stylizedCaustics.refine_bidirectional(stylizedCaustics.tpsResults);
            }
        }
    }
}

void PathTracer::traceScene(QRgb *imageData, const Scene& scene,float max_dist, int max_num, int min_num, float t)
{
    if(doStylizedCaustics) {
        for(int i = 0; i < stylizedCausticsVector.size(); i++) {
            auto& plane = planeVector[i];
            auto& stylizedCaustics = stylizedCausticsVector[i];
            // Move
            auto currentPos = stylizedCaustics.move(t);
            // Back projection
            stylizedCaustics.backProject(scene, pmap_caustic, plane, currentPos);
        }
        std::cout << "finished move" << std::endl;
        pmap_caustic.update();
        setIntegrator();
    }

    std::vector<Vector3f> intensityValues(m_width * m_height);
    Matrix4f invViewMat = (scene.getCamera().getScaleMatrix() * scene.getCamera().getViewMatrix()).inverse();
    std::cout << "start trace" << std::endl;
    #pragma omp parallel for
    for(int y = 0; y < m_height; ++y) {
//        std::cerr << "\rScanlines remaining: " << m_height - y << ' ' << std::flush;
        #pragma omp parallel for
        for(int x = 0; x < m_width; ++x) {
            int offset = x + (y * m_width);
            intensityValues[offset] = tracePixel(x, y, scene, invViewMat, max_dist, max_num, min_num);
        }
    }

    toneMap(imageData, intensityValues);
}

void PathTracer::generatePhotons(const Scene& scene) {
    std::cout << "start generate" << std::endl;
    pmap_r.maxPhotonNum = 60000;
    photonmapper.generatePhotonMap(pmap_r, scene);
    std::cout << "finish first generate" << std::endl;
    pmap_r.balance();
    std::cout<<"finish generating photon map, size: "<<pmap_r.photons.size()<<std::endl;
    pmap_caustic.maxPhotonNum = maxPhotonsNum;
    photonmapper.generatePhotonMap(pmap_caustic, scene, true);
    pmap_caustic.balance();
    std::cout<<"finish generating caustic photon map, size: "<<pmap_caustic.photons.size()<<std::endl;
    setIntegrator();
}

void PathTracer::setIntegrator() {
    m_integrator.setPmap(pmap_r);
    m_integrator.setPmapCaustic(pmap_caustic);
}

Vector3f PathTracer::tracePixel(int x, int y, const Scene& scene, const Matrix4f &invViewMatrix, float max_dist, int max_num, int min_num)
{
    Vector3f pixelColor(0.0, 0.0, 0.0);

    std::vector<Vector2f> pixelSamples = rng.GenerateStratifiedSamples();

    for (int s = 0; s < pixelSamples.size(); ++s) {
        Vector3f p(0, 0, 0);
        Vector3f d((2.f * (double(x) + pixelSamples[s][0]) / m_width) - 1, 1 - (2.f * (double(y) + pixelSamples[s][1]) / m_height), -1);
        d.normalize();
        Ray r(p, d);

        double lensRadius = 0.5f;
        double focalDistance = 3.5f;

        if (m_defocusBlurOn && lensRadius > 0.0f) {
            //picking a samplepoint on the lens and then changing it according to the lens properties
            Vector3f pLens = lensRadius * random_in_unit_disk();
            Vector3f pFocus = focalDistance * r.d + r.o;
            Vector3f aperaturePoint = r.o + (scene.getCamera().m_u * pLens[1]) + (scene.getCamera().m_s * pLens[0]);

            r.o = aperaturePoint;
            r.d = (pFocus - aperaturePoint).normalized();
        }

        r = r.transform(invViewMatrix);
        Vector3f color = m_integrator.traceRayWithPhotonMapping(r, scene, 0, true, max_dist, max_num, min_num);// traceRayWithPathTracing(r, scene, 0, true) * 2.5;
        pixelColor += Vector3f(color[0] / (1 + color[0]), color[1] / (1 + color[1]), color[2] / (1 + color[2]));
    }
//    pixelColor = Vector3f(
//                pixelColor[0] / (1 + pixelColor[0]),
//                pixelColor[1] / (1 + pixelColor[1]),
//                pixelColor[2] / (1 + pixelColor[2]));
    return pixelColor / (double)(rng.m_samplesPerPixel);
}


void PathTracer::toneMap(QRgb *imageData, std::vector<Vector3f> &intensityValues) {
    Vector3f maxColor(0, 0, 0);
    for(int y = 0; y < m_height; ++y) {
        for(int x = 0; x < m_width; ++x) {
            int offset = x + (y * m_width);
//            maxColor[0] = std::max(maxColor[0], intensityValues[offset][0]);
//            maxColor[1] = std::max(maxColor[1], intensityValues[offset][1]);
//            maxColor[2] = std::max(maxColor[2], intensityValues[offset][2]);
//            imageData[offset] = qRgb(255 * intensityValues[offset][0], 255 * intensityValues[offset][1], 255 * intensityValues[offset][2]);
            Vector3f rgb_f = 255.f * ToneMapper::gammaCorrect(ToneMapper::reinhard_simple(intensityValues[offset]), 0.5);
//            Vector3f rgb_f = 255.f * gammaCorrect(aces_approx(intensityValues[offset]), 0.58);
            imageData[offset] = qRgb(rgb_f(0), rgb_f(1), rgb_f(2));
        }
    }
//    std::cout<<maxColor[0]<<" "<<maxColor[1]<<" "<<maxColor[2]<<std::endl;
}
