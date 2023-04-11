#ifndef IMAGESAMPLER_H
#define IMAGESAMPLER_H


#include <Eigen/Dense>
#include <vector>

class ImageSampler
{
public:
    ImageSampler();

    std::vector<Eigen::Vector2f> sample(int width, int height, std::string path);
};

#endif // IMAGESAMPLER_H
