#ifndef RANDOMGENERATOR_H
#define RANDOMGENERATOR_H

#include <memory>
#include <pcg32.h>
#include <vector>
#include <Eigen>

class RandomGenerator
{
public:
    RandomGenerator(int samplesPerPixel, int seed) : m_samplesPerPixel(samplesPerPixel), rng(seed) {};

    // A function to generate a collection of stratified samples in a square
    std::vector<Eigen::Vector2f> GenerateStratifiedSamples();

    int m_samplesPerPixel;
private:
    pcg32 rng;
};

#endif // RANDOMGENERATOR_H
