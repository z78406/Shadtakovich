//
// Light class.
// 2022/04/22

#ifndef Light_H
#define Light_H
#include <eigen3/Eigen/Eigen>


class light {
    public:
    light() {};

    struct p_Light {// point light 
        std::vector<Eigen::Vector3f> position;
        std::vector<Eigen::Vector3f> intensity;
        p_Light(const std::vector<Eigen::Vector3f>& p, const std::vector<Eigen::Vector3f>& i) {
            position = p, intensity = i;
        };
    };

    struct a_Light {// ambient light
        Eigen::Vector3f intensity;
        a_Light() {};
        a_Light(const Eigen::Vector3f& amb) {
            intensity = amb;
        };
    };
};




#endif //Light_H