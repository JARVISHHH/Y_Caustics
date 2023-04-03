#ifndef COORDINATESYSTEM_H
#define COORDINATESYSTEM_H
#include <Eigen>

using namespace Eigen;

class CoordinateSystem {
private:
    Vector3f T[3];
    Vector3f INV_T[3];

public:
    CoordinateSystem(const Vector3f& N = Vector3f(0.0, 0.0, 1.0));

    Vector3f from(const Vector3f& v) const;
    Vector3f to(const Vector3f& v) const;

};

inline float CosTheta(const Vector3f &w) {
    return w[2];
}

inline float Cos2Theta(const Vector3f &w) {
    return w[2] * w[2];
}

inline float AbsCosTheta(const Vector3f &w) {
    return std::abs(w[2]);
}

inline float Sin2Theta(const Vector3f &w) {
    return std::max((float)0, (float)1 - Cos2Theta(w));
}

inline float SinTheta(const Vector3f &w) {
    return std::sqrt(Sin2Theta(w));
}

inline float CosPhi(const Vector3f &w) {
    float sinTheta = SinTheta(w);
    return (sinTheta == 0) ? 1 : std::min(std::max(w[0] / sinTheta, -1.0f), 1.0f);
}

inline float SinPhi(const Vector3f &w) {
    float sinTheta = SinTheta(w);
    return (sinTheta == 0) ? 0 : std::min(std::max(w[1] / sinTheta, -1.0f), 1.0f);
}

#endif // COORDINATESYSTEM_H
