/**
 * @file CS123Common.h
 *
 * Contains data structures and macros commonly used in CS123.
 */
#pragma once
#ifndef __CS123COMMON_H__
#define __CS123COMMON_H__

#include <math.h>
#include <Eigen>

//// glu.h in different location on macs
//#ifdef __APPLE__
//#include <glu.h>
//#else
//#include <GL/glu.h>
//#endif

// from http://en.wikipedia.org/wiki/Assertion_(computing)
#define COMPILE_TIME_ASSERT(pred) switch(0){case 0:case pred:;}

typedef float REAL;

#define IMAGE_WIDTH 512
#define IMAGE_HEIGHT 512

#define MIN(a, b) (a) < (b) ? (a) : (b)
#define MAX(a, b) (a) > (b) ? (a) : (b)

// ---------------------
// Common math utilities
// ---------------------

const float FLOAT_EPSILON = 1e-4f;
const double DOUBLE_EPSILON = 1e-8;

inline bool floatEpsEqual(float a, float b) {
    // If the difference between a and b is less than epsilon, they are equal
    return fabs(a - b) < FLOAT_EPSILON;
}

inline bool doubleEpsEqual(double a, double b)
{
    // If the difference between a and b is less than epsilon, they are equal
    return fabs(a - b) < DOUBLE_EPSILON;
}

inline Eigen::Vector4f vec3Tovec4(const Eigen::Vector3f &v, float w) {
    return Eigen::Vector4f(v.x(), v.y(), v.z(), w);
}

inline Eigen::Vector3f piecewiseMul(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2) {
    return Eigen::Vector3f(v1[0] * v2[0], v1[1] * v2[1], v1[2] * v2[2]);
}

inline Eigen::Vector3f reflect(const Eigen::Vector3f& v, const Eigen::Vector3f& n) {
    return v + 2 * (-v.dot(n)) * n;
}

inline Eigen::Vector3f refract(const Eigen::Vector3f& uv, const Eigen::Vector3f& n, double eta_over_eta) {
    auto cosTheta = fmin(-uv.dot(n), 1.0);
    Eigen::Vector3f routPerp = eta_over_eta * (uv + cosTheta * n);
    Eigen::Vector3f routParallel = -sqrt(1 - pow(routPerp.norm(), 2.0)) * n;
    return routPerp + routParallel;
}
#endif
