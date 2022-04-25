//
// Transformation class headfile.
// 2022/04/22


#ifndef Transformation_H
#define Transformation_H

#include <eigen3/Eigen/Eigen>

class Transformation {
    Transformation() {};

    public:
        Eigen::Matrix4f get_view_matrix(const Eigen::Vector3f& eye_pos, const Eigen::Vector3f& gaze, const Eigen::Vector3f& up);            // camera to camera canonical transform
        Eigen::Matrix4f get_model_matrix(const Eigen::Vector3f& agl, const Eigen::Vector3f& scl, const Eigen::Vector3f& tsl);               // world to camera transform
        Eigen::Matrix4f get_projection_matrix(const float& eye_fov, const float& aspect_ratio,                                              // projection transform
                                      const float& zNear, const float& zFar);
        Eigen::Matrix4f get_screen_matrix(const int& width, const int& height);
        Eigen::Vector3f get_up_vector(const Eigen::Vector3f& eye_pos, const Eigen::Vector3f& origin);                                         // calc cam's up vector assume pointing at origin                                                // viewport transform
    private:


};




#endif //Transformation_H