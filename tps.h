#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>

class ThinPlateSpline {
public:

  ThinPlateSpline(std::vector<Eigen::Vector3d> &src, std::vector<Eigen::Vector3d> &dst);

  Eigen::Vector3d interpolate(const Eigen::Vector3d &p) const;


private:

  void build_VO();
  void build_mL();
  void solve();

  static inline double radialBasis(double r) {
    return r == 0.0 ? r : r * r * log(r);
  }

  std::vector<Eigen::Vector3d> mSrcPoints;
  std::vector<Eigen::Vector3d> mDstPoints;
  Eigen::MatrixXd mW;
  Eigen::MatrixXd mL;
  Eigen::MatrixXd VO;

};
