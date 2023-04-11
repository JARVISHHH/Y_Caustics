#include "imagesampler.h"
#include "qimage.h"
#include <iostream>

ImageSampler::ImageSampler()
{

}

std::vector<Eigen::Vector2f> ImageSampler::sample(int width, int height, std::string path) {
    std::vector<Eigen::Vector2f> res;

    QImage image(width, height, QImage::Format_RGB32);
    if(!image.load(QString::fromStdString(path)))
        std::cout << "Load image:" << path << " failed" << std::endl;
    for(int i = 0; i < height; i++) {
        for(int j = 0; j < width; j++) {
            if(image.pixel(j, i) != 0xffffffff)
                res.push_back({j - width / 2.0f, i - height / 2.0f});
        }
    }

    return res;
}
