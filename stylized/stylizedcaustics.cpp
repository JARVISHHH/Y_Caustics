#include "stylizedcaustics.h"
#include "imagesampler.h"
#include "photon.h"
#include "spline.h"
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

StylizedCaustics::StylizedCaustics(std::shared_ptr<ImageParameter> imageParameter)
    :imageParameter(imageParameter), width(imageParameter->img_size[0]), height(imageParameter->img_size[1])
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
        if(imageParameter->lightIndexEnd < 0 || imageParameter->lightIndexBegin <= photons[i].lightIdx && imageParameter->lightIndexEnd >= photons[i].lightIdx){
            auto caustics2D = plane.projectPoint(averageOrigin, photons[i].origin);
            if(caustics2D.norm() > 100) continue;
            sources.push_back(caustics2D);
            photonsMap.push_back(i);
        }
    }
    std::cout << "source number: " << sources.size() << std::endl;
}

void StylizedCaustics::assign(std::vector<Vector2f>& images) {
    m = sources.size();
    n = 300;  // TODO: user-define n

    std::cout << "m: " << m << " n: " << n << std::endl;

    if(m < n) {
        std::cerr << "Error: n bigger than m!" << std::endl;
        return;
    }

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
    unordered_map<Photon, int, photon_hash> B;  // Photon -> index in photon map
    int b_index = 0;
    for (const auto& t: targets){
        // convert targets (vector<Vector2f>) to a vector<Photon> (y = 0)
        Photon t_photon = Photon(Vector3f::Zero(), Vector3f(t(0), 0, t(1)), b_index);
        B_PhotonMap.store(t_photon);
        B[t_photon] = b_index;
        b_index++;
    }
    B_PhotonMap.update();
    cout << "finished kd tree construction" << endl;

    // construct a max heap containing the position b in B that is closest to each pos in positions
    // also construct a set of Photons for the next loop
    multiset<photons_dist, photons_dist_compare> distMaxHeapSet;
    unordered_map<std::shared_ptr<Photon>, int> I;
    int index = 0;
    for (const auto& pos : positions){
        Vector3f i_vec(pos(0), 0, pos(1));
        auto i = std::make_shared<Photon>(Vector3f::Zero(), i_vec);
        Photon b = B_PhotonMap.getNearestPhotonFrom(i_vec);
        float distance = (pos - Vector2f(b.origin(0), b.origin(2))).norm();
        photons_dist ppd {b, i, distance * distance};
        distMaxHeapSet.insert(ppd);
        I[i] = index;
        index++;
    }
    cout << "finished dist max heap construction" << endl;

    std::cout << distMaxHeapSet.size() << " " << B_PhotonMap.photons.size() << std::endl;

    // iteratively choose the max disance from distMaxHeap and store pair
    while (!I.empty()){
        std::cerr << "\rHeap remaining: "<< I.size() << " B_PhotonMap: "<< B_PhotonMap.photons.size() << std::flush;
        photons_dist d = *distMaxHeapSet.begin();
        Photon b = d.p1;
        auto i = d.p2;
        assignmentMap[I[i]] = B[b];

        B_PhotonMap.remove(b);
        I.erase(I.find(i));

        // update distMaxHeap: delete the first element w/ i
        distMaxHeapSet.erase(distMaxHeapSet.begin());
        std::vector<photons_dist> tempVector;
        // for all the objects in distMaxHeap, replace the b if b is deleted
        for(auto it = distMaxHeapSet.begin(); it != distMaxHeapSet.end();) {
            if (it->p1.origin == b.origin) {
                Photon new_b = B_PhotonMap.getNearestPhotonFrom(it->p2->origin);
                float new_distance = (Vector2f(new_b.origin(0), new_b.origin(2)) - Vector2f(it->p2->origin(0), it->p2->origin(2))).norm();
                photons_dist updated_element{new_b, it->p2, new_distance * new_distance};
                it = distMaxHeapSet.erase(it);
                tempVector.push_back(updated_element);
            } else {
                it++;
            }
        }
        for(auto& updated_element: tempVector) distMaxHeapSet.insert(updated_element);
    }
//    unordered_set<int> s;
//    for (int i = 0; i < assignmentMap.size(); i++){
//        s.insert(assignmentMap[i]);
//        cout << i << " ->" << assignmentMap[i] << endl;
//    }
//    cout << "set.size=" << s.size() << ", map.size="<< assignmentMap.size() << endl;
    cout << "finished iteration" << endl;
}

void StylizedCaustics::refine_random(vector<Vector2f> positions){
    assignmentMap.resize(positions.size());

    for (int i = 0; i < assignmentMap.size(); i++){
        assignmentMap[i] = i;
    }
}

void StylizedCaustics::refine_bidirectional(vector<Vector2f> positions){

    // construt a balanced kd tree (PhotonMap) out of target
    // also populate map
    PhotonMap B_PhotonMap = PhotonMap(targets.size());
    unordered_map<Photon, int, photon_hash> B;  // Photon -> index in photon map
    int b_index = 0;
    for (const auto& t: targets){
        // convert targets (vector<Vector2f>) to a vector<Photon> (y = 0)
        Photon t_photon = Photon(Vector3f::Zero(), Vector3f(t(0), 0, t(1)), b_index);
        B_PhotonMap.store(t_photon);
        B[t_photon] = b_index;
        b_index++;
    }
    B_PhotonMap.update();
//    cout << "finished kd tree construction" << endl;

    assignmentMap.clear();
    unordered_set<Photon, photon_hash> removed_bs;
    for (const auto& pos : positions){
        Vector3f i_vec(pos(0), 0, pos(1));
        auto i = std::make_shared<Photon>(Vector3f::Zero(), i_vec);
        Photon b = B_PhotonMap.getNearestPhotonFrom(i_vec);
        assignmentMap.push_back(B[b]);
        removed_bs.insert(b);
    }

    PhotonMap I_PhotonMap = PhotonMap(positions.size());
    unordered_map<Photon, int, photon_hash> I;  // Photon -> index in photon map
    int i_index = 0;
    for (const auto& p: positions){
        Photon p_photon = Photon(Vector3f::Zero(), Vector3f(p(0), 0, p(1)), i_index);
        I_PhotonMap.store(p_photon);
        I[p_photon] = i_index;
        i_index++;
    }
    I_PhotonMap.update();

    for(const auto& b_prime : B){
        if (removed_bs.contains(b_prime.first)) continue;
        Photon i = I_PhotonMap.getNearestPhotonFrom(b_prime.first.origin);
        assignmentMap[I[i]] = b_prime.second;
    }
}

std::vector<Eigen::Vector2f> StylizedCaustics::move(float t) {
    std::vector<Eigen::Vector2f> res(sources.size());
    float linearWeight = 0.5;
    #pragma omp parallel for
    for(int i = 0; i < sources.size(); i++) {
        // results after tps
//        res[i] = t * (tpsResults[i] - sources[i]) + sources[i];
        // final results

        //Assume we have point A. We use tps to move it to B. And then we use refinement to move it to C
        //t is the time point we want to solve t \in [0,1]
        Eigen::Vector2f A = sources[i];
        Eigen::Vector2f B = tpsResults[i];
        Eigen::Vector2f C = targets[assignmentMap[i]];

        std::vector<double> X = {(double)A[0], (double)B[0], (double)C[0]};
        std::vector<double> Y = {(double)A[1], (double)B[1], (double)C[1]};
        std::vector<double> T = {0, 0.5, 1};

        Eigen::Vector2f splineResult;
        {
            tk::spline spline(T,X);
            double target_x = spline(t);
            splineResult[0] = target_x;
        }
        {
            tk::spline spline(T,Y);
            double target_y = spline(t);
            splineResult[1] = target_y;
        }

        Eigen::Vector2f linearResult = t * (targets[assignmentMap[i]] - sources[i]) + sources[i];

        res[i] = (1 - linearWeight) * splineResult + linearWeight * linearResult;

//        res[i] = splineResult;
//        res[i] = linearResult;
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
//        cout << i << flush;
    }
    cout << "finished back projection" << endl;
}

void StylizedCaustics::calculateAverageOrigin(const std::vector<Photon>& photons) {
    averageOrigin = {0, 0, 0};
    int size = 0;
    for(const auto& photon: photons) {
        if(imageParameter->lightIndexEnd < 0 || imageParameter->lightIndexBegin <= photon.lightIdx && imageParameter->lightIndexEnd >= photon.lightIdx){
            size++;
            averageOrigin += photon.lastHit;
        }
    }
    if(size > 0) averageOrigin /= size;
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
