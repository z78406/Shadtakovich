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
        public:
        std::vector<Eigen::Vector3f> position;
        std::vector<Eigen::Vector3f> intensity;
        p_Light(const std::vector<Eigen::Vector3f>& p, const std::vector<Eigen::Vector3f>& i) {
            position = p, intensity = i;
        };

        void clear() {position = {}, intensity = {};}

        void update_pos(std::vector<Eigen::Vector3f> pos) {position = pos;} // update light position. 

        std::vector<Eigen::Vector4f> toVector4(std::vector<Eigen::Vector3f>& p3) { // convert every pixel vertex in p_3d into 4d coord
            std::vector<Eigen::Vector4f> res;
            std::transform(std::begin(p3), std::end(p3), std::back_inserter(res), [](auto& vec) { return Eigen::Vector4f(vec.x(), vec.y(), vec.z(), 1.f); });
            return res;
        }

        std::vector<Eigen::Vector3f> p_trans(std::vector<Eigen::Vector3f>& p3, const Eigen::Matrix4f& m) { // transform array of 3d point given transform mat
            std::vector<Eigen::Vector3f> res;
            std::vector<Eigen::Vector4f> p4 = toVector4(p3);
            for (auto p: p4) {
                Eigen::Vector4f p4_new = m * p;
                p4_new /= p4_new.w();
                res.push_back(p4_new.head<3>());
            }
            // std::cout<<m * p4[0]<<std::endl;
            // std::transform(std::begin(p4), std::end(p4), std::back_inserter(res), [&m](auto& vec) {Eigen::Vector4f new_p4 = m * vec; new_p4 /= new_p4.w(); return new_p4.head<3>();});
            // std::cout<<res[0]<<std::endl;
            return res;
        }

        Eigen::Vector4f toVector4(const Eigen::Vector3f& p3) { // convert every pixel vertex in p_3d into 4d coord
            return Eigen::Vector4f(p3.x(), p3.y(), p3.z(), 1.f);
        }
        Eigen::Vector3f p_trans(const Eigen::Vector3f& p3, const Eigen::Matrix4f& m) { // transform 3d point given transform mat
            Eigen::Vector4f p4 = toVector4(p3);
            p4 = m * p4;
            p4 /= p4.w();
            return p4.head<3>();
        }
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