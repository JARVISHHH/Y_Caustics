#include "stylizedcaustics.h"
#include "imagesampler.h"
#include <iostream>

StylizedCaustics::StylizedCaustics(float width, float height)
    : width(width), height(height)
{

}

std::vector<Eigen::Vector2f> StylizedCaustics::sample(int imageWidth, int imageHeight, std::string path) {
    ImageSampler imageSampler(width, height);
    return imageSampler.sample(imageWidth, imageHeight, path);
}

void StylizedCaustics::assign(std::vector<Eigen::Vector2f>& caustics, std::vector<Eigen::Vector2f>& images) {
    m = caustics.size();
    n = m;  // TODO: user-define n

    std::cout << "m: " << m << " n: " << n << std::endl;
    std::cout << "image size: " << images.size() << std::endl;

    Eigen::Vector2f maxPoint(-100, -100), minPoint(100, 100);
    for(auto& point: images) {
        for(int i = 0; i < 2; i++) {
            maxPoint[i] = std::max(maxPoint[i], point[i]);
            minPoint[i] = std::min(minPoint[i], point[i]);
        }
    }
    std::cout << "max point: " << maxPoint[0] << " " << maxPoint[1] << std::endl;
    std::cout << "min point: " << minPoint[0] << " " << minPoint[1] << std::endl;

    // Sample sources and targets
    sources = caustics;
    targets.reserve(m);
    int restTargets = m;
    while(images.size() < restTargets) {
        targets.insert(targets.end(), images.begin(), images.end());
        restTargets -= images.size();
    }
    std::random_shuffle(images.begin(), images.end());
    targets.insert(targets.end(), images.begin(), images.begin() + restTargets);
    std::cout << "targets size: " << targets.size() << std::endl;

    // Initialize assignment map
    assignmentMap.resize(m);
    for(int i = 0; i < m; i++)
        assignmentMap[i] = i;

//    // Initialize distance matrices
//    initializeMatrices();
//    // Greedy assign
//    greedy();
}

void StylizedCaustics::move(std::vector<Eigen::Vector2f>& caustics) {
//    assert(caustics.size() == sources.size());
    for(int i = 0; i < caustics.size(); i++) {
//        caustics[i] = targets[assignmentMap[i]];
        caustics[i] = targets[i];
//        caustics[i] = {0, 0};
    }
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
    bool swapAccepted = false;
    do {
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
                } else {
                    std::swap(assignmentMap[j], assignmentMap[k]);
                    firstTerm = originalFirstTerm;
                    secondTerm = originalSecondTerm;
                }
            }
        }
    } while(swapAccepted);
}
