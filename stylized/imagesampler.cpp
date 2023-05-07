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
            int gray = 255 - qGray(image.pixel(j, i));
            if(gray == 0) continue;
            float x = (float)j / image.width() * width - width / 2.0f;
            float y = (float)i / image.height() * height - height / 2.0f;
            std::vector<Eigen::Vector2f> samples(gray / 32, {x, y});
            res.insert(res.end(), samples.begin(), samples.end());
        }
    }

    std::cout << "sample size: " << res.size() << std::endl;

    return res;
}
