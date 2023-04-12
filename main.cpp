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
    parser.addPositionalArgument("time step", "Time step of the animation");
    parser.process(a);

    const QStringList args = parser.positionalArguments();
    if(args.size() != 2 && args.size() != 3) {
        std::cerr << "Error: Wrong number of arguments" << std::endl;
        a.exit(1);
        return 1;
    }
    QString scenefile = args[0];
    QString output = args[1];
    QString timeStepString = "-1";
    if(args.size() == 3) timeStepString = args[2];

    bool usePhotonMapping = true;
    int samplePerPixel = 100;
    bool defocusBlurOn = false;
    bool useOrenNayerBRDF = false;
    bool importanceSampling = false;

    Scene *scene;
    if(!Scene::load(scenefile, &scene)) {
        std::cerr << "Error parsing scene file " << scenefile.toStdString() << std::endl;
        a.exit(1);
        return 1;
    }

    PathTracer tracer(scene, IMAGE_WIDTH, IMAGE_HEIGHT, usePhotonMapping, samplePerPixel, defocusBlurOn, useOrenNayerBRDF, importanceSampling);

    if(args.size() == 2) {
        QImage image(IMAGE_WIDTH, IMAGE_HEIGHT, QImage::Format_RGB32);

        QRgb *data = reinterpret_cast<QRgb *>(image.bits());

        tracer.traceScene(data, *scene);

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
        delete scene;
    }
    else if(args.size() == 3) {
        float timeStep = timeStepString.toFloat();
        if(timeStep > 0 && timeStep < 1) {
            for(int i = 0; i <= 1; i++) {
                QImage image(IMAGE_WIDTH, IMAGE_HEIGHT, QImage::Format_RGB32);

                QRgb *data = reinterpret_cast<QRgb *>(image.bits());

                tracer.traceScene(data, *scene, i);

                std::string path = output.toStdString() + "-t" + std::to_string(i) + ".png";

                bool success = image.save(QString::fromStdString(path));
                if(!success) {
                    success = image.save(output, "PNG");
                }
                if(success) {
                    std::cout << "Wrote rendered image to " << path << std::endl;
                } else {
                    std::cerr << "Error: failed to write image to " << path << std::endl;
                }
            }
            float currentTime = timeStep;
            while(currentTime < 1) {
                QImage image(IMAGE_WIDTH, IMAGE_HEIGHT, QImage::Format_RGB32);

                QRgb *data = reinterpret_cast<QRgb *>(image.bits());

                tracer.traceScene(data, *scene, currentTime);

                std::string path = output.toStdString() + "-t" + std::to_string(currentTime) + ".png";

                bool success = image.save(QString::fromStdString(path));
                if(!success) {
                    success = image.save(output, "PNG");
                }
                if(success) {
                    std::cout << "Wrote rendered image to " << path << std::endl;
                } else {
                    std::cerr << "Error: failed to write image to " << path << std::endl;
                }

                currentTime += timeStep;
            }
        }
        else {
            std::cerr << "Error: wrong time step" << std::endl;
        }
        delete scene;
    }
    a.exit();
}
