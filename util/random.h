#ifndef RANDOM_H
#define RANDOM_H
#include <Eigen/Dense>
#include <cstdlib>
#include <random>

const float PI = 3.1415925;

static std::random_device _RD;
static std::mt19937 _GEN(_RD());
static std::uniform_real_distribution<> _PDF(0, 1);

inline double random_double() {
    //return rand() / (RAND_MAX + 1.0);
    return _PDF(_GEN);
}

inline double random_double(double min, double max) {
    return min + (max - min) * random_double();
}

inline static Eigen::Vector3f random(double min, double max) {
    return Eigen::Vector3f(random_double(min, max), random_double(min, max), random_double(min, max));
}

inline const Eigen::Vector3f UniformSampleOnHemisphere() noexcept {
    double r1 = 2 * M_PI * random_double(), r2 = random_double(), r2s = sqrt(r2);
//    double u1 = random_double(0.0, 1.0);
//    double u2 = random_double(0.0, 1.0);
//    // u1 := cos_theta
//    const double sin_theta = std::sqrt(std::max(0.0, 1.0 - u1 * u1));

//    const double phi = 2.0 * PI * u2;
    return {
        cos(r1) * r2s,
        sin(r1) * r2s,
        sqrt(1 - r2)
    };
}

inline Eigen::Vector3f UniformSampleTriangle(Eigen::Vector3f v1, Eigen::Vector3f v2, Eigen::Vector3f v3) {
    Eigen::Vector3f v12 = v2 - v1;
    Eigen::Vector3f v13 = v3 - v1;
//    double u1 = random_double();
//    return v1 + u1 * v12 + (1 - u1) * v13;
    double s = random_double();
    double t = random_double();
    bool inTriangle = s + t <= 1;
    Eigen::Vector3f pos = inTriangle ? v1 + s * v12 + t * v13 : v1 + (1.0 - s) * v12 + (1.0 - t) * v13;
    return pos;
}


inline Eigen::Vector3f random_in_unit_disk() {
    while (true) {
        auto p = Eigen::Vector3f(random_double(-1, 1), random_double(-1, 1), 0);
        if (p.norm() >= 1) continue;
        return p;
    }
}

inline Eigen::Vector3f squareToDiskConcentric() {
    Eigen::Vector2f sample(random_double(), random_double());
    Eigen::Vector2f sampleOffset = 2.0f * sample - Eigen::Vector2f(1, 1);
    if (sampleOffset[0] == 0 && sampleOffset[1] == 0) {
        return Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    }

    float theta, r;
    if(std::abs(sampleOffset[0]) > std::abs(sampleOffset[1])) {
        r = sampleOffset[0];
        theta = (M_PI / 4.0f) * (sampleOffset[1] / sampleOffset[0]);
    } else {
        r = sampleOffset[1];
        theta = (M_PI / 2.0f) - (M_PI / 4.0f) * (sampleOffset[0] / sampleOffset[1]);
    }
    return r * Eigen::Vector3f(std::cos(theta), std::sin(theta), 0.0f);
}

inline Eigen::Vector3f squareToHemisphereCosine() {
    Eigen::Vector3f d = squareToDiskConcentric();
    float z = std::sqrt(std::max(0.0f, 1.0f - d[0] * d[0] - d[1] * d[1]));
    return Eigen::Vector3f(d[0], d[1], z);
}


#endif // RANDOM_H
