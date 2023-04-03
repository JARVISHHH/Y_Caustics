#include "randomgenerator.h"

std::vector<Eigen::Vector2f> RandomGenerator::GenerateStratifiedSamples()
{
    std::vector<Eigen::Vector2f> samples;
    // The square root of the number of samples input
    int sqrtVal = (int) (std::sqrt((float) m_samplesPerPixel) + 0.5);
    // A number useful for scaling a square of size sqrtVal x sqrtVal to 1 x 1
    float invSqrtVal = 1.f / sqrtVal;

    m_samplesPerPixel = sqrtVal * sqrtVal;
    samples.resize(m_samplesPerPixel);

    for(int i = 0; i < m_samplesPerPixel; ++i)
    {
        int y = i / sqrtVal;
        int x = i % sqrtVal;
        Eigen::Vector2f sample;

        sample = Eigen::Vector2f((x + rng.nextFloat()) * invSqrtVal, (y + rng.nextFloat()) * invSqrtVal);
        samples[i] = sample;
    }
    return samples;
}
