#pragma once

#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>

class tps
{
public:

    void init(std::vector<Eigen::Vector2f> constraints);

    Eigen::Vector2f tps_solve(Eigen::Vector2f p);

private:

    std::vector<Eigen::Vector2f> _constraints;

    Eigen::MatrixXf _mW;

    float distance(Eigen::Vector2f v1, Eigen::Vector2f v2);

    float U(float r);

    Eigen::MatrixXf build_L();

    Eigen::MatrixXf build_VO();

    void prepare_mW();

};
