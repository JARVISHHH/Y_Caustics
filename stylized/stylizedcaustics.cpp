#include "stylizedcaustics.h"
#include "imagesampler.h"
#include "photon.h"

#include <iostream>

extern bool useGreedyMethod;

std::random_device rd;
std::mt19937 g;
using namespace Eigen;

StylizedCaustics::StylizedCaustics() {
    g = std::mt19937(rd());
}

StylizedCaustics::StylizedCaustics(float width, float height)
    : width(width), height(height)
{
    g = std::mt19937(rd());
}

std::vector<Eigen::Vector2f> StylizedCaustics::sample(std::string path) {
    ImageSampler imageSampler(width, height);
    return imageSampler.sample(path);
}

void StylizedCaustics::project(const Scene& scene, const std::vector<Photon> photons, Plane& plane) {
    sources.clear();
    for(int i = 0; i < photons.size(); i++) {
        auto caustics2D = plane.projectPoint(averageOrigin, photons[i].origin);
        if(caustics2D.norm() > 100) continue;
        sources.push_back(caustics2D);
        photonsMap.push_back(i);
//        auto backPro = plane.backProjectPoint(scene, averageOrigin, caustics2D);
//        if(backPro.norm() > 100) backPro = photons[i].origin;
//        std::cout << "Original pos: " << photons[i].origin[0] << " " << photons[i].origin[1] << " " << photons[i].origin[2] << std::endl;
//        std::cout << "Back pos: " << backPro[0] << " " << backPro[1] << " " << backPro[2] << std::endl;
    }
}

void StylizedCaustics::assign(std::vector<Vector2f>& images) {
    m = sources.size();
    n = 300;  // TODO: user-define n

    std::cout << "m: " << m << " n: " << n << std::endl;

    if(m < n) {
        std::cerr << "Error: n bigger than n!" << std::endl;
        return;
    }

//    Eigen::Vector2f maxPoint(-100, -100), minPoint(100, 100);
//    for(auto& point: images) {
//        for(int i = 0; i < 2; i++) {
//            maxPoint[i] = std::max(maxPoint[i], point[i]);
//            minPoint[i] = std::min(minPoint[i], point[i]);
//        }
//    }

    std::mt19937 rng(rd());    // Random-number engine used (Mersenne-Twister in this case)

    // Sample sources and targets
    targets.reserve(m);
    int restTargets = m;
    while(images.size() < restTargets) {
        targets.insert(targets.end(), images.begin(), images.end());
        restTargets -= images.size();
    }
//    std::random_shuffle(images.begin(), images.end());
    std::shuffle(images.begin(), images.end(), g);
    targets.insert(targets.end(), images.begin(), images.begin() + restTargets);
//    std::cout << "targets size: " << targets.size() << std::endl;

    // Sample subsets
    subsetSourcesIndex.clear(), subsetTargetsIndex.clear();
    subsetSourcesIndex.reserve(n), subsetTargetsIndex.reserve(n);
    std::vector<int> sampleIndex(m);
    for(int i = 0; i < m; i++) sampleIndex[i] = i;
    std::shuffle(sampleIndex.begin(), sampleIndex.end(), g);

    //subsetSourcesIndex = A'
    for(int i = 0; i < n; i++) subsetSourcesIndex[i] = sampleIndex[i];
    std::shuffle(sampleIndex.begin(), sampleIndex.end(), g);

    //subsetSourcesIndex = B'
    for(int i = 0; i < n; i++) subsetTargetsIndex[i] = sampleIndex[i];

    // Initialize assignment map
    assignmentMap.resize(m);
    std::unordered_map<int, int> target2source;
    for(int i = 0; i < m; i++) {
        assignmentMap[i] = i;
        target2source[i] = i;
    }
    for(int i = 0; i < n; i++) {
        sourcesIndexInSubset[subsetSourcesIndex[i]] = i;
        targetsIndexInSubset[subsetTargetsIndex[i]] = i;
        int source1 = subsetSourcesIndex[i], source2 = target2source[subsetTargetsIndex[i]];
        int target1 = subsetTargetsIndex[i], target2 = assignmentMap[subsetSourcesIndex[i]];
        std::swap(assignmentMap[source1], assignmentMap[source2]);
        std::swap(target2source[target1], target2source[target2]);
    }

    if(useGreedyMethod) {
        // Initialize distance matrices
        initializeMatrices();
        // Greedy assign
        greedy();
    }
}

// given interpolation results ("positions"), populate assignmentmap
void StylizedCaustics::refine(vector<Vector2f> positions){

    // construt a balanced kd tree (PhotonMap) out of target
    // also populate map
    PhotonMap B_PhotonMap = PhotonMap(targets.size());
    unordered_map<Photon, int, photon_hash> B;
    int b_index = 0;
    for (const auto& t: targets){
        // convert targets (vector<Vector2f>) to a vector<Photon> (y = 0)
        Photon t_photon = Photon{Vector3f::Zero(), Vector3f(t(0), 0, t(1)), Vector3f::Zero(), 0};
        B_PhotonMap.store(t_photon);
        B[t_photon] = b_index;
        b_index++;
    }
    B_PhotonMap.update();
    cout << "finished kd tree construction" << endl;

    // construct a max heap containing the position b in B that is closest to each pos in positions
    // also construct a set of Photons for the next loop
//    multiset<photons_dist, less<photons_dist>> distMaxHeap;
    set<photons_dist, photons_dist_compare> distMaxHeapSet;
    unordered_map<Photon, int, photon_hash> I; // Photons in positions mapped to their indices
    int index = 0;
    for (const auto& pos : positions){
        Vector3f i_vec(pos(0), 0, pos(1));
        Photon i = Photon{Vector3f::Zero(), i_vec};
        Photon b = B_PhotonMap.getNearestPhotonFrom(i_vec);
        float distance = (pos - Vector2f(b.origin(0), b.origin(2))).norm();
        photons_dist ppd {b, i, distance * distance};
//        distMaxHeap.insert(ppd);
        distMaxHeapSet.insert(ppd);
        I[i] = index;
        index++;
    }
    cout << "finished dist max heap construction" << endl;

    // iteratively choose the max disance from distMaxHeap and store pair
    while (!I.empty()){
//        photons_dist d = *distMaxHeap.begin();
        photons_dist d = *distMaxHeapSet.begin();
        Photon b = d.p1;
        Photon i = d.p2;
        assignmentMap[B[b]] = I[i]; //store in assignment map

        B_PhotonMap.remove(b);
        I.erase(I.find(i));

        // update distMaxHeap: delete the first element w/ i
//        distMaxHeap.erase(distMaxHeap.begin());
        distMaxHeapSet.erase(distMaxHeapSet.begin());
        // for all the objects in distMaxHeap, replace the b if b is deleted
//        multiset<photons_dist, less<photons_dist>> distMaxHeapTemp;
//        auto curr_it = distMaxHeap.begin();
//        while (curr_it != distMaxHeap.end()){
//            photons_dist curr = *curr_it;
//            if (curr.p1.origin == b.origin){
//                curr.p1 = B_PhotonMap.getNearestPhotonFrom(curr.p2.origin);
//            }
//            distMaxHeapTemp.insert(curr);
//            curr_it = next(curr_it);
//        }
//        distMaxHeap.swap(distMaxHeapTemp);
        for(auto it = distMaxHeapSet.begin(); it != distMaxHeapSet.end(); ) {
            if (it->p1.origin == b.origin) {
                Photon new_b = B_PhotonMap.getNearestPhotonFrom(it->p2.origin);
                float new_distance = (Vector2f(new_b.origin(0), new_b.origin(2)) - Vector2f(it->p2.origin(0), it->p2.origin(2))).norm();
                photons_dist updated_element{new_b, it->p2, new_distance * new_distance};
                it = distMaxHeapSet.erase(it);
                distMaxHeapSet.insert(updated_element);
            } else {
                ++it;
            }
        }

//        cout << "I.size()=" << I.size() << endl;
    }
    cout << "finished iteration" << endl;
}

std::vector<Eigen::Vector2f> StylizedCaustics::move(float t) {
    std::vector<Eigen::Vector2f> res(sources.size());
    for(int i = 0; i < sources.size(); i++) {
        // results after tps
//        res[i] = t * (finalResults[i] - sources[i]) + sources[i];
        // final results
        res[i] = t * (targets[assignmentMap[i]] - sources[i]) + sources[i];
    }
    return res;
}

void StylizedCaustics::backProject(const Scene& scene, PhotonMap& pmap_caustic, Plane& plane, std::vector<Eigen::Vector2f>& currentPos) {
    pmap_caustic.box_max = Eigen::Vector3f(-1000000.0, -1000000.0, -1000000.0);
    pmap_caustic.box_min = Eigen::Vector3f(1000000.0, 1000000.0, 1000000.0);
    auto& photons = pmap_caustic.photons;
    for(int i = 0; i < sources.size(); i++) {
        auto& photon = photons[photonsMap[i]];
//        std::cout << "Photon position: " << photon.origin[0] << " " << photon.origin[1] << " " << photon.origin[2] << std::endl;
        auto hitPoint = plane.backProjectPoint(scene, averageOrigin, currentPos[i]);
        if(hitPoint.norm() > 100) hitPoint = photon.origin;
//        std::cout << "Hit point: " << hitPoint[0] << " " << hitPoint[1] << " " << hitPoint[2] << std::endl;
        photon.origin = hitPoint;
        photon.origin = photon.origin;
        photon.dir = (hitPoint - photon.lastHit).normalized();
    }
    pmap_caustic.update();
}

void StylizedCaustics::calculateAverageOrigin(const std::vector<Photon>& photons) {
    averageOrigin = {0, 0, 0};
    for(const auto& photon: photons) averageOrigin += photon.lastHit;
    averageOrigin /= photons.size();
}

std::vector<Eigen::Vector2f> StylizedCaustics::getSubsetSourcesPos() {
    std::vector<Eigen::Vector2f> res(n);
    for(int i = 0; i < n; i++) {
        res[i] = sources[subsetSourcesIndex[i]];
    }
    return res;
}

std::vector<Eigen::Vector2f> StylizedCaustics::getSubsetTargetsPos() {
    std::vector<Eigen::Vector2f> res(n);
    for(int i = 0; i < n; i++) {
        res[i] = targets[assignmentMap[subsetSourcesIndex[i]]];
    }
    return res;
}

float StylizedCaustics::energy(float a, float b) {
    return (1 - beta) / m * a + beta * b;
}

void StylizedCaustics::initializeMatrices() {
    DA = Eigen::MatrixXf(n, n), DB = Eigen::MatrixXf(n, n), DAB = Eigen::MatrixXf(n, n);
    for(int i = 0; i < n; i++) {
        for(int j = 0; j < n; j++) {
            DA(i, j) = (sources[subsetSourcesIndex[i]] - sources[subsetSourcesIndex[j]]).norm();
            DB(i, j) = (targets[subsetTargetsIndex[i]] - targets[subsetTargetsIndex[j]]).norm();
            DAB(i, j) = (sources[subsetSourcesIndex[i]] - targets[subsetTargetsIndex[j]]).norm();
        }
    }
}

void StylizedCaustics::greedy() {
    float firstTerm = (DA - DB).norm(), secondTerm = DAB.trace();
    float oldEnergy = energy(firstTerm, secondTerm), newEnergy;
    std::cout << "Original energy: " << oldEnergy << std::endl;
    bool swapAccepted = false;
    int sum = 0;
    do {
        swapAccepted = false;
        for(int j = 0; j < n; j++) {
            for(int k = 0; k < n; k++) {
                int x = targetsIndexInSubset[assignmentMap[subsetSourcesIndex[j]]], y = targetsIndexInSubset[assignmentMap[subsetSourcesIndex[k]]];
                std::swap(assignmentMap[subsetSourcesIndex[j]], assignmentMap[subsetSourcesIndex[k]]);
                // calculate new energy
                // Modify the first term
                float originalFirstTerm = firstTerm, originalSecondTerm = secondTerm;
                firstTerm *= firstTerm;
                for(int i = 0; i < n; i++) {
                    if(i == j || i == k) continue;

                    int z = targetsIndexInSubset[assignmentMap[subsetSourcesIndex[i]]];

                    firstTerm -= (DA(i, j) - DB(z, x)) * (DA(i, j) - DB(z, x));
                    firstTerm -= (DA(i, k) - DB(z, y)) * (DA(i, k) - DB(z, y));
                    firstTerm += (DA(i, j) - DB(z, y)) * (DA(i, j) - DB(z, y));
                    firstTerm += (DA(i, k) - DB(z, x)) * (DA(i, k) - DB(z, x));

                    firstTerm -= (DA(j, i) - DB(x, z)) * (DA(j, i) - DB(x, z));
                    firstTerm -= (DA(k, i) - DB(y, z)) * (DA(k, i) - DB(y, z));
                    firstTerm += (DA(j, i) - DB(y, z)) * (DA(j, i) - DB(y, z));
                    firstTerm += (DA(k, i) - DB(x, z)) * (DA(k, i) - DB(x, z));
                }
                firstTerm -= (DA(j, j) - DB(x, x)) * (DA(j, j) - DB(x, x));
                firstTerm += (DA(j, j) - DB(y, y)) * (DA(j, j) - DB(y, y));
                firstTerm -= (DA(k, k) - DB(y, y)) * (DA(k, k) - DB(y, y));
                firstTerm += (DA(k, k) - DB(x, x)) * (DA(k, k) - DB(x, x));
                firstTerm = std::sqrt(firstTerm);
                // Modify the second term
                secondTerm -= (sources[j] - targets[x]).norm();
                secondTerm -= (sources[k] - targets[y]).norm();
                secondTerm += (sources[j] - targets[y]).norm();
                secondTerm += (sources[k] - targets[x]).norm();
                newEnergy = energy(firstTerm, secondTerm);
                if(newEnergy < oldEnergy) {
                    oldEnergy = newEnergy;
                    swapAccepted = true;
                    break;
                } else {
                    std::swap(assignmentMap[subsetSourcesIndex[j]], assignmentMap[subsetSourcesIndex[k]]);
                    firstTerm = originalFirstTerm;
                    secondTerm = originalSecondTerm;
                }
            }
            if(swapAccepted) break;
        }
        sum++;
//        if(sum % n == 0) std::cout << sum << std::endl;
    } while(swapAccepted);
    std::cout << "Final energy: " << oldEnergy << std::endl;
}
