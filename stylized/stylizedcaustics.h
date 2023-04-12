#ifndef STYLIZEDCAUSTICS_H
#define STYLIZEDCAUSTICS_H

#include <Eigen/Dense>
#include <vector>

class StylizedCaustics
{
public:
    StylizedCaustics(float width, float height);

    std::vector<Eigen::Vector2f> sample(int width, int height, std::string path);
    void assign(std::vector<Eigen::Vector2f>& caustics, std::vector<Eigen::Vector2f>& images);
    void move(std::vector<Eigen::Vector2f>& caustics);

private:
    float energy(float a, float b);
    void initializeMatrices();
    void greedy();

    float width, height;

    int m;  // Total number of points
    int n;  // Number of points in subsets
    std::vector<Eigen::Vector2f> sources;  // Points of source caustics A
    std::vector<Eigen::Vector2f> targets;  // Points of target image B
    std::vector<int> assignmentMap;  // ith point in A should map to assignmentMap[i]th point in B

    Eigen::MatrixXf DA, DB, DAB;
    float beta = 0.5;
};

#endif // STYLIZEDCAUSTICS_H
