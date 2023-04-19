#ifndef STYLIZEDCAUSTICS_H
#define STYLIZEDCAUSTICS_H

#include "photon.h"
#include "stylized/projection/plane.h"
#include <Eigen/Dense>
#include <vector>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <queue>

class StylizedCaustics
{
public:
    StylizedCaustics();
    StylizedCaustics(float width, float height);

    std::vector<Eigen::Vector2f> sample(int width, int height, std::string path);
    void project(const Scene& scene, const std::vector<Photon> photons, Plane& plane);
    void assign(std::vector<Eigen::Vector2f>& images);
    void refine(std::vector<Eigen::Vector2f> I);
    std::vector<Eigen::Vector2f> move(float t = 0);
    void backProject(const Scene& scene, PhotonMap& pmap_caustic, Plane& plane, std::vector<Eigen::Vector2f>& currentPos);

    void calculateAverageOrigin(const std::vector<Photon>& photons);
    Eigen::Vector3f getAverageOrigin() {return averageOrigin;}

private:
    float energy(float a, float b);
    void initializeMatrices();
    void greedy();

    float width, height;

    int m;  // Total number of points
    int n;  // Number of points in subsets
    std::vector<Eigen::Vector2f> sources;  // Points of source caustics A
    std::vector<Eigen::Vector2f> targets;  // Points of target image B
    std::vector<int> subsetSourcesIndex;
    std::vector<int> subsetTargetsIndex;
    std::unordered_map<int, int> sourcesIndexInSubset;
    std::unordered_map<int, int> targetsIndexInSubset;
    std::vector<int> assignmentMap;  // ith point in A should map to assignmentMap[i]th point in B
    Eigen::Vector3f averageOrigin;

    Eigen::MatrixXf DA, DB, DAB;
    float beta = 0.5;

    std::vector<int> photonsMap;
};

#endif // STYLIZEDCAUSTICS_H
