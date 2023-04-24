#include "imagesampler.h"
#include "qimage.h"
#include <iostream>

ImageSampler::ImageSampler(float width, float height)
    : width(width), height(height)
{

}

std::vector<Eigen::Vector2f> ImageSampler::sample(std::string path) {
    std::vector<Eigen::Vector2f> res;

    QImage image;
    if(!image.load(QString::fromStdString(path)))
        std::cout << "Load image:" << path << " failed" << std::endl;

    for(int i = 0; i < image.height(); i++) {
        for(int j = 0; j < image.width(); j++) {
            if(image.pixel(j, i) != 0xffffffff)
                res.push_back({(float)j / image.width() * width - width / 2.0f, (float)i / image.height() * height - height / 2.0f});
        }
    }

    return res;
}
