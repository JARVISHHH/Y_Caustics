#include "imagesampler.h"
#include "qimage.h"
#include <iostream>

ImageSampler::ImageSampler(float width, float height)
    : width(width), height(height)
{

}

std::vector<Eigen::Vector2f> ImageSampler::sample(int imageWidth, int imageHeight, std::string path) {
    std::vector<Eigen::Vector2f> res;

    QImage image(imageWidth, imageHeight, QImage::Format_RGB32);
    if(!image.load(QString::fromStdString(path)))
        std::cout << "Load image:" << path << " failed" << std::endl;
    for(int i = 0; i < imageHeight; i++) {
        for(int j = 0; j < imageWidth; j++) {
            if(image.pixel(j, i) != 0xffffffff)
                res.push_back({(float)j / imageWidth * width - width / 2.0f, (float)i / imageHeight * height - height / 2.0f});
        }
    }

    return res;
}
