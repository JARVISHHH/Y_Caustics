#include <QCoreApplication>
#include <QCommandLineParser>

#include <iostream>

#include "pathtracer.h"
#include "scene/scene.h"

#include <QImage>

#include "stylized/stylizedcaustics.h"
#include "util/CS123Common.h"

#include "stylized/projection/plane.h"

int main(int argc, char *argv[])
{
    srand(time(NULL));

    QCoreApplication a(argc, argv);

    QCommandLineParser parser;
    parser.addHelpOption();
    parser.addPositionalArgument("scene", "Scene file to be rendered");
    parser.addPositionalArgument("output", "Image file to write the rendered image to");
    parser.process(a);

    const QStringList args = parser.positionalArguments();
    if(args.size() != 2) {
        std::cerr << "Error: Wrong number of arguments" << std::endl;
        a.exit(1);
        return 1;
    }
    QString scenefile = args[0];
    QString output = args[1];

    bool usePhotonMapping = true;
    int samplePerPixel = 100;
    bool defocusBlurOn = false;
    bool useOrenNayerBRDF = false;
    bool importanceSampling = false;

    QImage image(IMAGE_WIDTH, IMAGE_HEIGHT, QImage::Format_RGB32);
//    QImage image(531, 171, QImage::Format_RGB32);

    Scene *scene;
    if(!Scene::load(scenefile, &scene)) {
        std::cerr << "Error parsing scene file " << scenefile.toStdString() << std::endl;
        a.exit(1);
        return 1;
    }

    // projection test
//    Plane plane(Eigen::Vector3f(0, 1, 0), Eigen::Vector3f(0, 1, 0));
//    auto projectedPoint = plane.project(Eigen::Vector3f(0, 6, 0), Eigen::Vector3f(0, -1, 0));
//    std::cout << projectedPoint[0] << " " << projectedPoint[1] << std::endl;
//    auto backProjectedPoint = plane.backProject(*scene, Eigen::Vector3f(0, 6, 0), projectedPoint);
//    std::cout << backProjectedPoint[0] << " " << backProjectedPoint[1] << " " << backProjectedPoint[2] << std::endl;

    // Image sampler test
//    StylizedCaustics StylizedCaustics;
//    auto samples = StylizedCaustics.sample(531, 171, "./example-scenes/images/CS2240.png");
//    std::cout << "number of samples: " << samples.size() << std::endl;

//    QRgb *data = reinterpret_cast<QRgb *>(image.bits());
//    for(int x = 0; x < 531; x++)
//        for(int y = 0; y < 171; y++)
//            data[x + y * 531] = qRgb(0, 0, 0);
//    for(auto& samplePoint: samples) {
//        int x = std::max(0, int(samplePoint[0] + 531 / 2.0f));
//        int y = std::max(0, int(samplePoint[1] + 171 / 2.0f));
//        data[x + y * 531] = qRgb(255, 255, 255);
//    }

    PathTracer tracer(IMAGE_WIDTH, IMAGE_HEIGHT, usePhotonMapping, samplePerPixel, defocusBlurOn, useOrenNayerBRDF, importanceSampling);

    QRgb *data = reinterpret_cast<QRgb *>(image.bits());

    tracer.traceScene(data, *scene);
    delete scene;

    std::string path = output.toStdString() + ".png";

    bool success = image.save(QString::fromStdString(path));
    if(!success) {
        success = image.save(output, "PNG");
    }
    if(success) {
        std::cout << "Wrote rendered image to " << output.toStdString() << std::endl;
    } else {
        std::cerr << "Error: failed to write image to " << output.toStdString() << std::endl;
    }
    a.exit();
}
