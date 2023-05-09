#ifndef STYLIZEDCAUSTICS_H
#define STYLIZEDCAUSTICS_H

#include "photon.h"
#include "stylized/projection/plane.h"
#include <Eigen/Dense>
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
    StylizedCaustics(std::shared_ptr<ImageParameter> imageParameter);

    std::vector<Eigen::Vector2f> sample(std::string path);
    void project(const Scene& scene, const std::vector<Photon> photons, Plane& plane);
    void assign(std::vector<Eigen::Vector2f>& images);
    void refine(std::vector<Eigen::Vector2f> I);
    void refine_random(std::vector<Eigen::Vector2f> I);
    void refine_bidirectional(std::vector<Eigen::Vector2f> I);
    std::vector<Eigen::Vector2f> move(float t = 0);
    void backProject(const Scene& scene, PhotonMap& pmap_caustic, Plane& plane, std::vector<Eigen::Vector2f>& currentPos);

    void calculateAverageOrigin(const std::vector<Photon>& photons);
    Eigen::Vector3f getAverageOrigin() {return averageOrigin;}

    std::vector<Eigen::Vector2f> getSubsetSourcesPos();
    std::vector<Eigen::Vector2f> getSubsetTargetsPos();
    std::vector<Eigen::Vector2f> getSources() {return sources;}
    std::vector<Eigen::Vector2f> getTargets() {return targets;}

    std::vector<Eigen::Vector2f> tpsResults;
    void setTpsResults(std::vector<Eigen::Vector2f>& B) {
        tpsResults.resize(m);
        for(int i = 0; i < m; i++)
        {
            if(sourcesIndexInSubset.find(i) != sourcesIndexInSubset.end()) tpsResults[i] = targets[assignmentMap[i]];
            else tpsResults[i] = B[i];
        }
    }

private:
    float energy(float a, float b);
    void initializeMatrices();
    void greedy();

    float width, height;

    std::shared_ptr<ImageParameter> imageParameter;

    int m;  // Total number of points
    int n;  // Number of points in subsets

    std::vector<Eigen::Vector2f> sources;  // Points of source caustics A
    std::vector<Eigen::Vector2f> targets;  // Points of target image B
    std::vector<int> subsetSourcesIndex;  // Index of chosen points
    std::vector<int> subsetTargetsIndex;  // Index of chosen points
    std::unordered_map<int, int> sourcesIndexInSubset;  // key is the index from vector sources, value is its index in sourcesIndexInSubset
    std::unordered_map<int, int> targetsIndexInSubset;  // similar
    std::vector<int> assignmentMap;  // ith point in A should map to assignmentMap[i]th point in B
    Eigen::Vector3f averageOrigin;

    // For greedy algorithm
    Eigen::MatrixXf DA, DB, DAB;
    float beta = 0.5;

    // Results position after tps
    std::vector<Eigen::Vector2f> tpsPos;

    std::vector<int> photonsMap;  // photonsMap[i] = j -> sources[i] is photons[j]
};

#endif // STYLIZEDCAUSTICS_H
