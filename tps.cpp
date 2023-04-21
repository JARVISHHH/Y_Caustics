#include "tps.h"
#include <Eigen/QR>
#include <iostream>

ThinPlateSpline::ThinPlateSpline(std::vector<Eigen::Vector3d> &src, std::vector<Eigen::Vector3d> &dst) {

    mSrcPoints = src;
    mDstPoints = dst;
    solve();
}

void ThinPlateSpline::solve() {

  if (mSrcPoints.size() != mDstPoints.size()){
      std::cout <<"must have same constraints as targets";
      return;
  }

  build_VO();
  build_mL();
  mW = mL.colPivHouseholderQr().solve(VO);
}

Eigen::Vector3d ThinPlateSpline::interpolate(const Eigen::Vector3d &p) const {

  Eigen::Vector3d res = Eigen::Vector3d::Zero();
  int i = 0;

  for (; i < mW.rows() - (3 + 1); ++i) {
    double rb = radialBasis((mSrcPoints[std::size_t(i)] - p).norm());
    res += mW.row(i) * rb;
  }

  res += mW.row(i);
  i++;

  for (int j(0); j < 3; ++j, ++i)
    res += mW.row(i) * p[j];

  return res;
}

void ThinPlateSpline::build_VO(){
    const int num(int(mSrcPoints.size()));
    const int rows(num + 3 + 1);

   VO = Eigen::MatrixXd::Zero(rows, 3);

    for (int i(0); i < num; ++i){
      VO.row(i) = mDstPoints[std::size_t(i)];
    }
}

void ThinPlateSpline::build_mL(){
    const int num(int(mSrcPoints.size()));
    const int rows(num + 3 + 1);

    mL = Eigen::MatrixXd::Zero(rows, rows);

    for (int i(0); i < num; i++) {

      int j = i + 1;

      for (; j < num; ++j)
        mL(i, j) = mL(j, i) = radialBasis(
            (mSrcPoints[std::size_t(i)] - mSrcPoints[std::size_t(j)]).norm());

      mL(j, i) = mL(i, j) = 1.0;
      j+=1;

      for (int posElm(0); j < rows; ++posElm, ++j){
          mL(j, i) = mL(i, j) = mSrcPoints[std::size_t(i)][posElm];
      }
    }

}
