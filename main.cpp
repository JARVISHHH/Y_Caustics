#include <QCoreApplication>
#include <QCommandLineParser>

#include <iostream>
#include <fstream>
#include <sstream>

#include "pathtracer.h"
#include "scene/scene.h"

#include <QImage>

#include "util/CS123Common.h"

int maxPhotonsNum = 50000;
float intensityDivisor = 1;

std::string trim(const std::string& str) {
    auto front = std::find_if_not(str.begin(), str.end(), [](int ch) {
        return std::isspace(ch);
    });
    auto back = std::find_if_not(str.rbegin(), str.rend(), [](int ch) {
        return std::isspace(ch);
    }).base();

    return (back <= front ? std::string() : std::string(front, back));
}

std::vector<std::string> split(const std::string& str, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::stringstream ss(str);
    while (std::getline(ss, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

std::map<std::string, std::string> parse_ini_file(const std::string& file_path) {
    std::ifstream file(file_path);
    std::map<std::string, std::string> ini_data;
    std::string line, key, value;

    if (file.is_open()) {
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == ';' || line[0] == '#') {
                continue;
            }

            std::stringstream ss(line);
            std::getline(ss, key, '=');
            std::getline(ss, value);
            key = trim(key);
            value = trim(value);

            if (!key.empty() && !value.empty()) {
                ini_data[key] = value;
            }
        }
        file.close();
    } else {
        std::cerr << "Unable to open file: " << file_path << std::endl;
    }

    return ini_data;
}

int main(int argc, char *argv[])
{
    srand(time(NULL));

    QCoreApplication a(argc, argv);

    QCommandLineParser parser;
    parser.addHelpOption();
    parser.addPositionalArgument("ini", "path to the .ini file");

//    parser.addPositionalArgument("scene", "Scene file to be rendered");
//    parser.addPositionalArgument("output", "Image file to write the rendered image to");
//    parser.addPositionalArgument("time step", "Time step of the animation");
    parser.process(a);

    const QStringList args = parser.positionalArguments();
    QString ini_path = args[0];
    std::map<std::string, std::string> ini_data = parse_ini_file(ini_path.toStdString());
    QString scenefile = QString::fromStdString(ini_data["scenefile"]);
    QString output = QString::fromStdString(ini_data["output"]);

    QString timeStepString = QString::fromStdString(ini_data["t"]);

    maxPhotonsNum = std::stoi(ini_data["photonmap_photon_num"]);
    float photonmap_max_dist = std::stof(ini_data["photonmap_max_dist"]);
    int photonmap_max_num = std::stoi(ini_data["photonmap_max_num"]);
    int photonmap_min_num = std::stoi(ini_data["photonmap_min_num"]);
    intensityDivisor = std::stof(ini_data["photonmap_intensity_divisor"]);

    bool usePhotonMapping = true;
    int samplePerPixel = std::stoi(ini_data["sample_per_pixel"]);
    bool defocusBlurOn = false;
    bool useOrenNayerBRDF = false;
    bool importanceSampling = false;

    Scene *scene;
    if(!Scene::load(scenefile, &scene)) {
        std::cerr << "Error parsing scene file " << scenefile.toStdString() << std::endl;
        a.exit(1);
        return 1;
    }

    PathTracer tracer(scene, IMAGE_WIDTH, IMAGE_HEIGHT,
                      usePhotonMapping, samplePerPixel,
                      defocusBlurOn, useOrenNayerBRDF, importanceSampling);
    float timeStep = timeStepString.toFloat();
    for(int i = 0; i <= 1; i++) {
        QImage image(IMAGE_WIDTH, IMAGE_HEIGHT, QImage::Format_RGB32);

        QRgb *data = reinterpret_cast<QRgb *>(image.bits());

        tracer.traceScene(data, *scene, photonmap_max_dist, photonmap_max_num, photonmap_min_num, i);

        std::string path = output.toStdString() + "-t" + std::to_string(float(i)) + ".png";

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
    if(timeStep > 0 && timeStep < 1) {
        float currentTime = timeStep;
        while(currentTime < 1) {
            QImage image(IMAGE_WIDTH, IMAGE_HEIGHT, QImage::Format_RGB32);

            QRgb *data = reinterpret_cast<QRgb *>(image.bits());

            tracer.traceScene(data, *scene, photonmap_max_dist, photonmap_max_num, photonmap_min_num, currentTime);

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
//    }
    a.exit();
}
