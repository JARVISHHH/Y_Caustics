#ifndef PHOTON_H
#define PHOTON_H
#include <vector>
#include <Eigen>

#include <queue>
using namespace std;

typedef struct Photon {
    Eigen::Vector3f lastHit;
    Eigen::Vector3f origin;        // photon position
    Eigen::Vector3f power;      // photon power (uncompressed)
    int divide_axis;        // splitting plane for kd-tree
    Eigen::Vector3f dir;    // incoming direction
} Photon;


inline int calculate_mid(int start, int end)
{
    int num = end - start + 1;
    int smaller_num = pow(2, int(log2(num))) - 1;
    return start + std::min((num - smaller_num), (smaller_num + 1) / 2) + smaller_num / 2;
}

struct photon_dist
{
    Photon p;
    float dist_square;
    photon_dist(Photon _p = Photon(), float _dist_square = 0) : p(_p), dist_square(_dist_square) {}
};

inline bool operator<(photon_dist p1, photon_dist p2)
{
    return p1.dist_square < p2.dist_square;
}

class nearest_photons_map
{
public:
    Eigen::Vector3f origin;
    int max_num;
    float max_dist_square;
    priority_queue<photon_dist, vector<photon_dist>, less<photon_dist>> nearest_photons;
    nearest_photons_map(Eigen::Vector3f _origin, float _max_dist_square, int _max_num = 0) : origin(_origin), max_dist_square(_max_dist_square), max_num(_max_num) {}

    void get_nearest_photons(const vector<Photon>& photons, int index);
};

//inline bool operator==(const Photon& a, const Photon& b)
//{
//    return a.origin == b.origin;
//}

//inline bool operator<(const Photon& a, const Photon& b)
//{
//    return a.origin[a.divide_axis] < b.origin[b.divide_axis];
//}

//inline bool operator>(const Photon& a, const Photon& b)
//{
//    return a.origin[a.divide_axis] > b.origin[b.divide_axis];
//}

class PhotonMap
{
public:
    Eigen::Vector3f box_min, box_max; // bounding box
    int maxPhotonNum;
    std::vector<Photon> photons;

    PhotonMap(int _maxPhotonNum = 50000);

    void insert(PhotonMap photonMap);

    void store(Photon p);
    float get_photon_origin_axis(int index, int axis);
    void split(std::vector<Photon>& photons_temp, int start, int end, int mid, int axis);
//    void remove(Photon p);
    void update();
    void balance();
    void balance(std::vector<Photon>& photons_temp, int index, int start, int end);
    Eigen::Vector3f getFixedRadiusIrradiance(Eigen::Vector3f origin, Eigen::Vector3f normal, float max_dist, int max_num, int min_num);
    Eigen::Vector3f getGaussianIrradiance(Eigen::Vector3f origin, Eigen::Vector3f normal, float max_dist, int max_num, int min_num);
    Eigen::Vector3f visualizePhotonMap(Eigen::Vector3f origin, float max_dist, int max_num);

    Photon getNearestPhotonFrom(Eigen::Vector3f origin, float max_dist);
};

struct photons_dist
{
    Photon p1;
    Photon p2;
    float dist_square;
};

inline bool operator<(photons_dist d1, photons_dist d2)
{
    return d1.dist_square > d2.dist_square; // for the sake of making the multiset max heap
}


#endif // PHOTON_H
