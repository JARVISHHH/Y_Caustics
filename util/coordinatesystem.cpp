#include "coordinatesystem.h"

CoordinateSystem::CoordinateSystem(const Vector3f& N) {
    double sign = std::copysign(1.0, N[2]);
    double a = -1.0 / (sign + N[2]);
    double b = N[0] * N[1] * a;
    T[0] = Vector3f(1.0 + sign * N[0] * N[0] * a, sign * b, -sign * N[0]);
    T[1] = Vector3f(b, sign + N[1] * N[1] * a, -N[1]);
    T[2] = N;

    INV_T[0] = Vector3f(1.0 + sign * N[0] * N[0] * a, b, N[0]);
    INV_T[1] = Vector3f(sign * b, sign + N[1] * N[1] * a, N[1]);
    INV_T[2] = Vector3f(-sign * N[0], -N[1], N[2]);
}

Vector3f CoordinateSystem::from(const Vector3f& v) const
{
    return (T[0] * v[0] + T[1] * v[1] + T[2] * v[2]).normalized();
}

Vector3f CoordinateSystem::to(const Vector3f& v) const
{
    return (INV_T[0] * v[0] + INV_T[1] * v[1] + INV_T[2] * v[2]).normalized();
}
