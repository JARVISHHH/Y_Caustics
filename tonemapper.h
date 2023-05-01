#include <Eigen/Core>
#include <Eigen/StdVector>


using namespace Eigen;

class ToneMapper{
public:
    static float luminance(Vector3f in) {
        return in.dot(Vector3f(0.2126f, 0.7152f, 0.0722f));
    }

    static Vector3f reinhard(Vector3f in){
        Vector3f res = in.array() / (1.f+luminance(in));
        return res;
    }

    static Vector3f reinhard_simple(Vector3f in){
        return in.array() / (1.f+in.array());
    }

    static Vector3f gammaCorrect(Vector3f in, float gamma){
        float l_in = luminance(in);
        float l_out = pow(l_in, gamma);
        return l_out / l_in * in.array();
    }

    static Vector3f aces_approx(Vector3f v){
        v *= 0.6f;
        float a = 2.51f;
        float b = 0.03f;
        float c = 2.43f;
        float d = 0.59f;
        float e = 0.14f;
        Vector3f u = v.array() * (a*v + Vector3f::Constant(b)).array();
        Vector3f l = v.array() * (c*v + Vector3f::Constant(d)).array();
        l += Vector3f::Constant(e);
        Vector3f res = u.array()/l.array();
        return Vector3f(std::clamp(res(0), 0.f, 1.f), std::clamp(res(1), 0.f, 1.f), std::clamp(res(2), 0.f, 1.f));
    }


};
