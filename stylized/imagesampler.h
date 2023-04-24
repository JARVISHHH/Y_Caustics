#ifndef IMAGESAMPLER_H
#define IMAGESAMPLER_H


#include <Eigen/Dense>
#include <vector>

class ImageSampler
{
public:
    ImageSampler(float width, float height);

    std::vector<Eigen::Vector2f> sample(std::string path);

private:
    float width, height;
};

#endif // IMAGESAMPLER_H
