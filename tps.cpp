#include "tps.h"
#include <iostream>
#include "Eigen/Dense"

#include <Eigen/QR>

void tps::init(std::vector<Eigen::Vector2f> constraints){
    _constraints = constraints;
    prepare_mW();
}


Eigen::Vector2f tps::tps_solve(Eigen::Vector2f p){

    Eigen::Vector2f res = Eigen::Vector2f::Zero();
    int n = _constraints.size();

    for(int i=0; i<n; i++){
        float dist = distance(p, _constraints[i]);
        float u = U(dist);
        res += _mW.row(i) * u;
    }

    res += _mW.row(n);
    res += _mW.row(n+1) * p[0];
    res += _mW.row(n+2) * p[1];

    return res;
}


float tps::distance(Eigen::Vector2f v1, Eigen::Vector2f v2){
    Eigen::Vector2f diff = v1 - v2;
    return sqrt(v1[0]*v1[0] + v2[0]*v2[0]);
}

float tps::U(float r){
    return r*r*log(r);
}


Eigen::MatrixXf tps::build_L(){

    int n = _constraints.size();
    int rows = _constraints.size() + 3;

    Eigen::MatrixXf L = Eigen::MatrixXf::Zero(rows, rows);

    for(int i=0; i<n; i++){
        for(int j=0; j<n; j++){

            if(i == j){
                L(i,j) = 0;
            }else{
                float dist = distance(_constraints[i], _constraints[j]);
                L(i,j) = U(dist);
            }
        }

        L(i, n) = 1.0;
        L(n,i)= 1.0;
    }

    for(int i=0; i<n; i++){
        L(n+1, i) = _constraints[i][0];
        L(n+2, i) = _constraints[i][1];

        L(i, n+1) = _constraints[i][0];
        L(i, n+2) = _constraints[i][1];
    }

    return L;
}


Eigen::MatrixXf tps::build_VO(){
    int n = _constraints.size();
    int rows = _constraints.size() + 3;
    Eigen::MatrixXf VO = Eigen::MatrixXf::Zero(rows, 3);

    for(int i=0; i<n; i++){
        VO(i,0) = _constraints[i][0];
        VO(i,1) = _constraints[i][1];
    }

    return VO;
}


void tps::prepare_mW(){
    Eigen::MatrixXf L = build_L();
    Eigen::MatrixXf VO = build_VO();

    _mW = L.colPivHouseholderQr().solve(VO);

}

