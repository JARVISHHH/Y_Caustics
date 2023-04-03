#include "photon.h"

#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <iostream>

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

PhotonMap::PhotonMap(int _maxPhotonNum) : maxPhotonNum(_maxPhotonNum)
{
    box_min = Eigen::Vector3f(1000000.0, 1000000.0, 1000000.0);
    box_max = Eigen::Vector3f(-1000000.0, -1000000.0, -1000000.0);
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

Eigen::Vector3f PhotonMap::getIrradiance(Eigen::Vector3f origin, Eigen::Vector3f normal, float max_dist, int max_num)
{
    Eigen::Vector3f res(0.0, 0.0, 0.0);
    nearest_photons_map local_map(origin, max_dist * max_dist, max_num);
    local_map.get_nearest_photons(photons, 0);
    if (local_map.nearest_photons.size() <= 15)
        return res;


    while (!local_map.nearest_photons.empty())
    {
        Eigen::Vector3f dir = local_map.nearest_photons.top().p.dir;
        if (normal.dot(dir) < 0)
            res += local_map.nearest_photons.top().p.power;
        local_map.nearest_photons.pop();
    }

    res *= (1.0 / (M_PI * max_dist * max_dist)) * (1.0 / M_PI) / 50000.0;
//    cerr << res << endl;
    return res;
}
