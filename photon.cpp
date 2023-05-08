#include "photon.h"

#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <iostream>

extern float intensityDivisor;
static double gaussianFilter(double distSquared, double maxDistSquared) {
    return 0.918 * (1.0 - (1.0 - std::exp(-1.953 * distSquared / (2 * maxDistSquared))) / ( 1.0 - std::exp(-1.953)));
}

void nearest_photons_map::get_nearest_photons(const vector<Photon>& photons, int index)
{

    Photon cur_photon = photons[index];
    if (2 * index + 1 < photons.size())
    {
        double dist_axis = origin[cur_photon.divide_axis] - cur_photon.origin[cur_photon.divide_axis];
        if (dist_axis < 0 || dist_axis * dist_axis < max_dist_square)
            get_nearest_photons(photons, 2 * index + 1);
        if ((2 * index + 2 < photons.size()) && (dist_axis >= 0 || dist_axis * dist_axis < max_dist_square))
            get_nearest_photons(photons, 2 * index + 2);
    }

    float cur_dist_square = pow((cur_photon.origin - origin).norm(), 2); //
    if (cur_dist_square > max_dist_square)
        return;
    nearest_photons.push(photon_dist(cur_photon, cur_dist_square));

    if (nearest_photons.size() > max_num)
        nearest_photons.pop();
}

void nearest_photons_map::get_nearest_photon(const vector<Photon>& photons, int index)
{
    if (index >= photons.size()) {
        return;
    }

    Photon cur_photon = photons[index];

    if (2 * index + 1 < photons.size())
    {
        double dist_axis = origin[cur_photon.divide_axis] - cur_photon.origin[cur_photon.divide_axis];
        if (dist_axis < 0)
            get_nearest_photon(photons, 2 * index + 1);
        else if (2 * index + 2 < photons.size())
            get_nearest_photon(photons, 2 * index + 2);
    }
    //std::cout << "origin = " << origin(0) << ", " << origin(1) << ", " << origin(2) << std::endl;
//    std::cout << "cur_photon.origin = " << cur_photon.origin(0) << ", " << cur_photon.origin(1) << ", " << cur_photon.origin(2) << std::endl;

    float cur_dist_square = (cur_photon.origin - origin).squaredNorm();
//    std::cout << "cur_dist_square = " << cur_dist_square << std::endl;
    if (cur_dist_square < min_dist_square)
    {
//        std::cout << "min_dist_square = " << min_dist_square << std::endl;
        min_dist_square = cur_dist_square;
//        std::cout << "min_dist_square = " << min_dist_square << std::endl;
        nearest_photon = cur_photon;
    }
}


PhotonMap::PhotonMap(int _maxPhotonNum) : maxPhotonNum(_maxPhotonNum)
{
    box_min = Eigen::Vector3f(1000000.0, 1000000.0, 1000000.0);
    box_max = Eigen::Vector3f(-1000000.0, -1000000.0, -1000000.0);

    photons.reserve(maxPhotonNum);
}

void PhotonMap::insert(PhotonMap photonMap) {
    photons.insert(photons.end(), photonMap.photons.begin(), photonMap.photons.end());

    box_min = Eigen::Vector3f(min(box_min.x(), photonMap.box_min.x()), min(box_min.y(), photonMap.box_min.y()), min(box_min.z(), photonMap.box_min.z()));
    box_max = Eigen::Vector3f(max(box_max.x(), photonMap.box_max.x()), max(box_max.y(), photonMap.box_max.y()), max(box_max.z(), photonMap.box_max.z()));
}

void PhotonMap::store(Photon p) {
    if (photons.size() >= maxPhotonNum)
        return;
    photons.push_back(p);

    box_min = Eigen::Vector3f(min(box_min.x(), p.origin.x()), min(box_min.y(), p.origin.y()), min(box_min.z(), p.origin.z()));
    box_max = Eigen::Vector3f(max(box_max.x(), p.origin.x()), max(box_max.y(), p.origin.y()), max(box_max.z(), p.origin.z()));
}

float PhotonMap::get_photon_origin_axis(int index, int axis)
{
    return photons[index].origin[axis];
}

void PhotonMap::split(vector<Photon>& photons_temp, int start, int end, int mid, int axis)
{
    int l = start, r = end;
    while (l < r)
    {
        double pivot = photons_temp[r].origin[axis];
        int i = l - 1, j = r;
        while (true)
        {
            while (photons_temp[++i].origin[axis] < pivot);
            while (photons_temp[--j].origin[axis] > pivot && j > l);

            if (i >= j)
                break;
            swap(photons_temp[i], photons_temp[j]);
        }
        swap(photons_temp[i], photons_temp[r]);
        if (i >= mid)
            r = i - 1;
        if (i <= mid)
            l = i + 1;
    }
}

void PhotonMap::update() {
    box_min = Eigen::Vector3f(1000000.0, 1000000.0, 1000000.0);
    box_max = Eigen::Vector3f(-1000000.0, -1000000.0, -1000000.0);
    for(const auto& p: photons) {
        box_min = Eigen::Vector3f(min(box_min.x(), p.origin.x()), min(box_min.y(), p.origin.y()), min(box_min.z(), p.origin.z()));
        box_max = Eigen::Vector3f(max(box_max.x(), p.origin.x()), max(box_max.y(), p.origin.y()), max(box_max.z(), p.origin.z()));
    }
    balance();
}

void PhotonMap::balance()
{
    vector<Photon> phptons_temp = photons;
    balance(phptons_temp, 0, 0, photons.size() - 1);
}

void PhotonMap::balance(vector<Photon>& photons_temp, int index, int start, int end)
{
    if (index >= photons_temp.size())
        return;
    if (start == end)
    {
        photons[index] = photons_temp[start];
        return;
    }
    int mid = calculate_mid(start, end);

    int axis = 2;
    float x_boundary = box_max.x() - box_min.x(), y_boundary = box_max.y() - box_min.y(), z_boundary = box_max.z() - box_min.z();
    if (x_boundary > max(y_boundary, z_boundary))
        axis = 0;
    if (y_boundary > max(x_boundary, z_boundary))
        axis = 1;
    split(photons_temp, start, end, mid, axis);
    photons[index] = photons_temp[mid];
    photons[index].divide_axis = axis;

    if (start < mid)
    {
        float tmp = box_max[axis];
        box_max[axis] = photons[index].origin[axis];
        balance(photons_temp, index * 2 + 1, start, mid - 1); // left child
        box_max[axis] = tmp;
    }

    if (mid < end)
    {
        float tmp = box_min[axis];
        box_min[axis] = photons[index].origin[axis];
        balance(photons_temp, index * 2 + 2, mid + 1, end); // right child
        box_max[axis] = tmp;
    }
}

void PhotonMap::remove(Photon p){
    int low = 0, high = static_cast<int>(photons.size()) - 1;
    int index = -1;

    for(int i = 0; i < photons.size(); i++) {
        if(photons[i] == p) {
//            std::cout << i << std::endl;
            photons.erase(photons.begin() + i);
            update();
            break;
        }
    }

//    while (low <= high){
//        int mid = low + (high - low) / 2;

//        if (photons[mid] == p){
//            index = mid;
//            break;
//        }

//        if (photons[mid].origin[photons[mid].divide_axis] < p.origin[photons[mid].divide_axis]){
//            low = mid + 1;
//        } else {
//            high = mid - 1;
//        }
//    }

//    if (index != -1){
//        photons.erase(photons.begin() + index);
//        update();
//    }

//    std::cout << "index: " << index << std::endl;
}


Eigen::Vector3f PhotonMap::getFixedRadiusIrradiance(Eigen::Vector3f origin, Eigen::Vector3f normal, float max_dist, int max_num, int min_num)
{
    Eigen::Vector3f res(0.0, 0.0, 0.0);
    nearest_photons_map local_map(origin, max_dist * max_dist, max_num);
    local_map.get_nearest_photons(photons, 0);
    if (local_map.nearest_photons.size() <= min_num)
        return res;

//    return local_map.nearest_photons.top().p.power * (1.0 / (M_PI * max_dist * max_dist)) * (1.0 / M_PI);

    double size = local_map.nearest_photons.size();

    while (!local_map.nearest_photons.empty())
    {
        Eigen::Vector3f dir = local_map.nearest_photons.top().p.dir;
        if (normal.dot(dir) < 0)
            res += local_map.nearest_photons.top().p.power;
        local_map.nearest_photons.pop();
    }

    res *= (1.0 / (M_PI * max_dist * max_dist)) * (1.0 / M_PI) / size / 100.0;
    return res;
}
// max_dist 越大越模糊, max_num 越大越亮，min_num 越小越模糊
Eigen::Vector3f PhotonMap::getGaussianIrradiance(Eigen::Vector3f origin, Eigen::Vector3f normal, float max_dist, int max_num, int min_num)
{
    Eigen::Vector3f res(0.0, 0.0, 0.0);
    nearest_photons_map local_map(origin, max_dist * max_dist, max_num);
    local_map.get_nearest_photons(photons, 0);
    if (local_map.nearest_photons.size() <= min_num)
        return res;

//    cout << local_map.nearest_photons.size() << endl;

    while (!local_map.nearest_photons.empty())
    {
        Eigen::Vector3f dir = local_map.nearest_photons.top().p.dir;
        if (normal.dot(dir) < 0)
            res += local_map.nearest_photons.top().p.power * gaussianFilter(local_map.nearest_photons.top().dist_square, max_dist * max_dist);
        local_map.nearest_photons.pop();
    }

    res *= (1.0 / (M_PI * max_dist * max_dist)) / intensityDivisor;

//    cout << res << endl;

    return res;
}

Eigen::Vector3f PhotonMap::getGaussianIrradianceWithFixedNum(Eigen::Vector3f origin, Eigen::Vector3f normal, int max_num, int min_num)
{
    Eigen::Vector3f res(0.0, 0.0, 0.0);
    nearest_photons_map local_map(origin, 0.1 * 0.1, max_num);
    local_map.get_nearest_photons(photons, 0);
    if (local_map.nearest_photons.size() <= min_num)
        return res;

    float max_dist_square = local_map.nearest_photons.top().dist_square;

    while (!local_map.nearest_photons.empty())
    {
        Eigen::Vector3f dir = local_map.nearest_photons.top().p.dir;
        if (normal.dot(dir) < 0)
            res += local_map.nearest_photons.top().p.power * gaussianFilter(local_map.nearest_photons.top().dist_square, max_dist_square);
        local_map.nearest_photons.pop();
    }

    res *= (1.0 / (M_PI * max_dist_square)) * 10.0;
//    cout << res << endl;
    return res;
}

Eigen::Vector3f PhotonMap::visualizePhotonMap(Eigen::Vector3f origin, float max_dist, int max_num) {
    nearest_photons_map local_map(origin, max_dist * max_dist, max_num);
    local_map.get_nearest_photons(photons, 0);
    if (local_map.nearest_photons.size() <= 3)
        return Eigen::Vector3f(0.0, 0.0, 0.0);
    else
        return Eigen::Vector3f(1.0, 1.0, 1.0);
}


Photon PhotonMap::getNearestPhotonFrom(Eigen::Vector3f origin){
    nearest_photons_map local_map = nearest_photons_map(origin, 0, 1);
    local_map.get_nearest_photon(photons, 0);
    Photon out = local_map.nearest_photon;
    return out;
}


