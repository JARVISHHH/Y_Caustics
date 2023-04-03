#include "basiccamera.h"

#include <iostream>

using namespace Eigen;

BasicCamera::BasicCamera(Vector3f position, Vector3f direction, Vector3f up, float heightAngle, float aspect)
    : m_position(position), m_direction(direction), m_up(up),
      m_heightAngle(heightAngle), m_aspectRatio(aspect)
{
    Vector3f f = m_direction.normalized();
    Vector3f u = m_up.normalized();
    Vector3f s = f.cross(u);
    u = s.cross(f);

    m_f = -f;
    m_s = s;
    m_u = u;
}

Matrix4f BasicCamera::getViewMatrix() const
{
    Matrix4f Result;
    Result << m_s.x(), m_s.y(), m_s.z(), -m_s.dot(m_position),
            m_u.x(), m_u.y(), m_u.z(), -m_u.dot(m_position),
            m_f.x(), m_f.y(), m_f.z(), -m_f.dot(m_position),
            0, 0, 0, 1;
    return Result;
}

Matrix4f BasicCamera::getScaleMatrix() const
{
    float heightAngleRads = M_PI * m_heightAngle / 360.f;//We need half the angle
    float tanThetaH = tan(heightAngleRads);
    float tanThetaW = m_aspectRatio * tanThetaH;

    Matrix4f scale = Matrix4f::Identity();
    scale(0, 0) = 1 / tanThetaW;
    scale(1, 1) = 1 / tanThetaH;
    return scale;
}
