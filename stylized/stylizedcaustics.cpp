#include "stylizedcaustics.h"
#include "imagesampler.h"
#include <iostream>

extern bool useGreedyMethod;

StylizedCaustics::StylizedCaustics() {

}

StylizedCaustics::StylizedCaustics(float width, float height)
    : width(width), height(height)
{

}

std::vector<Eigen::Vector2f> StylizedCaustics::sample(int imageWidth, int imageHeight, std::string path) {
    ImageSampler imageSampler(width, height);
    return imageSampler.sample(imageWidth, imageHeight, path);
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

void StylizedCaustics::assign(std::vector<Eigen::Vector2f>& images) {
    m = sources.size();
    n = m;  // TODO: user-define n

    std::cout << "m: " << m << " n: " << n << std::endl;

    Eigen::Vector2f maxPoint(-100, -100), minPoint(100, 100);
    for(auto& point: images) {
        for(int i = 0; i < 2; i++) {
            maxPoint[i] = std::max(maxPoint[i], point[i]);
            minPoint[i] = std::min(minPoint[i], point[i]);
        }
    }

    // Sample sources and targets
    targets.reserve(m);
    int restTargets = m;
    while(images.size() < restTargets) {
        targets.insert(targets.end(), images.begin(), images.end());
        restTargets -= images.size();
    }
//    std::random_shuffle(images.begin(), images.end());
    targets.insert(targets.end(), images.begin(), images.begin() + restTargets);
//    std::cout << "targets size: " << targets.size() << std::endl;

    // Initialize assignment map
    assignmentMap.resize(m);
    for(int i = 0; i < m; i++)
        assignmentMap[i] = i;

    if(useGreedyMethod) {
        // Initialize distance matrices
        initializeMatrices();
        // Greedy assign
        greedy();
    }
}

std::vector<Eigen::Vector2f> StylizedCaustics::move(float t) {
    std::vector<Eigen::Vector2f> res(sources.size());
    for(int i = 0; i < sources.size(); i++) {
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

float StylizedCaustics::energy(float a, float b) {
    return (1 - beta) / m * a + beta * b;
}

void StylizedCaustics::initializeMatrices() {
    DA = Eigen::MatrixXf(n, n), DB = Eigen::MatrixXf(n, n), DAB = Eigen::MatrixXf(n, n);
    for(int i = 0; i < n; i++) {
        for(int j = 0; j < n; j++) {
            DA(i, j) = (sources[i] - sources[j]).norm();
            DB(i, j) = (targets[i] - targets[j]).norm();
            DAB(i, j) = (sources[i] - targets[j]).norm();
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
                int x = assignmentMap[j], y = assignmentMap[k];
                std::swap(assignmentMap[j], assignmentMap[k]);
                // calculate new energy
                // Modify the first term
                float originalFirstTerm = firstTerm, originalSecondTerm = secondTerm;
                firstTerm *= firstTerm;
                for(int i = 0; i < n; i++) {
                    if(i == j || i == k) continue;

                    int z = assignmentMap[i];

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
                    std::swap(assignmentMap[j], assignmentMap[k]);
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
