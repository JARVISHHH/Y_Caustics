#ifndef PHOTON_H
#define PHOTON_H
#include <memory>
#include <vector>
#include <Eigen>

#include <queue>
using namespace std;

struct Photon {
    Eigen::Vector3f lastHit;
    Eigen::Vector3f origin;        // photon position

    int index = 0;
    int lightIdx;

    Eigen::Vector3f power;      // photon power (uncompressed)
    int divide_axis;        // splitting plane for kd-tree
    Eigen::Vector3f dir;    // incoming direction

    Photon(){;}

    Photon(Eigen::Vector3f lastHit, Eigen::Vector3f origin, int index = 0, int lightIdx = 0, Eigen::Vector3f power = Eigen::Vector3f::Zero(), int divide_axis = 0, Eigen::Vector3f dir = Eigen::Vector3f::Zero()) {
        this->lastHit = lastHit;
        this->origin = origin;
        this->index = index;
        this->power = power;
        this->divide_axis = divide_axis;
        this->dir = dir;
        this->lightIdx = lightIdx;
    }

    Photon(Photon const &a) {
        this->lastHit = a.lastHit;
        this->origin = a.origin;
        this->index = a.index;
        this->power = a.power;
        this->divide_axis = a.divide_axis;
        this->dir = a.dir;
        this->lightIdx = a.lightIdx;
    }
};

struct photon_hash{
    size_t operator()(const Photon& p) const {
        // Combine the hashes of individual attributes using XOR and bit shifting
//        std::hash<float> float_hash;
        std::hash<int> int_hash;
        return int_hash((int)p.index);
//        return float_hash(p.origin.x()) ^ (float_hash(p.origin.y()) << 1) ^ (float_hash(p.origin.z()) << 2);
     }
};

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
    Photon nearest_photon;
    float min_dist_square;
    nearest_photons_map(Eigen::Vector3f _origin, float _max_dist_square, int _max_num = 0) : origin(_origin), max_dist_square(_max_dist_square), max_num(_max_num), min_dist_square(std::numeric_limits<float>::max()) {}

    void get_nearest_photons(const vector<Photon>& photons, int index);
    void get_nearest_photon(const vector<Photon>& photons, int index);
};

inline bool operator==(const Photon& a, const Photon& b)
{
    return a.index == b.index;
}

class PhotonMap
{
public:
    Eigen::Vector3f box_min, box_max; // bounding box
    int maxPhotonNum;
    std::vector<Photon> photons;

    PhotonMap(int _maxPhotonNum = 200000);

    void insert(PhotonMap photonMap);

    void store(Photon p);
    float get_photon_origin_axis(int index, int axis);
    void split(std::vector<Photon>& photons_temp, int start, int end, int mid, int axis);
    void remove(Photon p);
    void update();
    void balance();
    void balance(std::vector<Photon>& photons_temp, int index, int start, int end);
    Eigen::Vector3f getFixedRadiusIrradiance(Eigen::Vector3f origin, Eigen::Vector3f normal, float max_dist, int max_num, int min_num);
    Eigen::Vector3f getGaussianIrradiance(Eigen::Vector3f origin, Eigen::Vector3f normal, float max_dist, int max_num, int min_num);
    Eigen::Vector3f getGaussianIrradianceWithFixedNum(Eigen::Vector3f origin, Eigen::Vector3f normal, int max_num, int min_num);
    Eigen::Vector3f visualizePhotonMap(Eigen::Vector3f origin, float max_dist, int max_num);

    Photon getNearestPhotonFrom(Eigen::Vector3f origin);
};

struct photons_dist
{
    Photon p1;
    std::shared_ptr<Photon> p2;
    float dist_square;
};

inline bool operator<(photons_dist d1, photons_dist d2)
{
    return d1.dist_square > d2.dist_square; // for the sake of making the multiset max heap
}

struct photons_dist_compare {
    bool operator()(const photons_dist& lhs, const photons_dist& rhs) const {
        if (lhs.dist_square == rhs.dist_square) {
            if (lhs.p2->origin(0) != rhs.p2->origin(0)) {
                return lhs.p2->origin(0) < rhs.p2->origin(0);
            } else if (lhs.p2->origin(1) != rhs.p2->origin(1)) {
                return lhs.p2->origin(1) < rhs.p2->origin(1);
            } else {
                return lhs.p2->origin(2) < rhs.p2->origin(2);
            }
        }
        return lhs.dist_square < rhs.dist_square;
    }
};


#endif // PHOTON_H
