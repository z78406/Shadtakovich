//
//
// Transformation class main file.
// 2022/04/22


#include <opencv2/opencv.hpp>
#include <math.h>
#include "Transformation.hpp"
#include "global.hpp"

inline double Degree(double angle)  {return angle*MY_PI/180.0;}



Eigen::Matrix4f Transformation::get_view_matrix(const Eigen::Vector3f& eye_pos, const Eigen::Vector3f& gaze = {0, 0, -1}, 
                                                const Eigen::Vector3f& up = {0,1,0}) {
                                                                    // canonical transform: camera to camera canonical
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f translate;
    Eigen::Matrix4f rotate;

    translate << 1,0,0,-eye_pos[0],
                0,1,0,-eye_pos[1],
                0,0,1,-eye_pos[2],
                0,0,0,1;   
    
    rotate << gaze.cross(up).x(), up.x(), -1 * gaze.x(), 0,
              gaze.cross(up).y(), up.y(), -1 * gaze.y(), 0,
              gaze.cross(up).z(), up.z(), -1 * gaze.z(), 0,
              0,                0,    0,           1;

    view = rotate * translate * view;
    return view;
}

Eigen::Matrix4f Transformation::get_model_matrix(const Eigen::Vector3f& agl, const Eigen::Vector3f& scl = {2.5, 2.5, 2.5}, const Eigen::Vector3f& tsl = {0, 0, 0}) { // model transform: world to camera coord

    Eigen::Matrix4f rotation_x, rotation_y, rotation_z, rotation;
    float angle_x = agl.x(), angle_y = agl.y(), angle_z = agl.z();
    angle_x = angle_x * MY_PI / 180.f;
    angle_y = angle_y * MY_PI / 180.f;
    angle_z = angle_z * MY_PI / 180.f;

    rotation_x << 1, 0, 0, 0,
                0, cos(angle_x), -sin(angle_x), 0,
                0, sin(angle_x), cos(angle_x), 0,
                0, 0, 0, 1;
    rotation_y << cos(angle_y), 0, sin(angle_y), 0,
                0, 1, 0, 0,
                -sin(angle_y), 0, cos(angle_y), 0,
                0, 0, 0, 1;
    rotation_z << cos(angle_z), -sin(angle_z),0, 0,
                sin(angle_z), cos(angle_z), 0, 0,
                0, 			0, 			1, 0,
                0, 			0, 			0, 1;
    rotation = rotation_x * rotation_y * rotation_z;

    Eigen::Matrix4f scale;
    scale << scl.x(), 0, 0, 0,
              0, scl.y(), 0, 0,
              0, 0, scl.z(), 0,
              0, 0, 0, 1;

    Eigen::Matrix4f translate;
    translate << tsl.x(), 0, 0, 0,
            0, tsl.y(), 0, 0,
            0, 0, tsl.z(), 0,
            0, 0, 0, 1;
            
    return  rotation * translate * scale;    
}

Eigen::Matrix4f Transformation::get_projection_matrix(const float& eye_fov, const float& aspect_ratio,    // projection transform
                                      const float& zNear, const float& zFar) {
	                                                                // projection matrix: first perspective then orthorgonal projection.
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // renaming params.
    float n = zNear, f = zFar, e = eye_fov, as = aspect_ratio;
    float half_angle = eye_fov / 2.0 / 180.0 * MY_PI;
    float t = n * std::tan(half_angle); // similar triangles.
    float r = t * as;
    float l = -1 * r;
    float b = -1 * t;
    // P2O mat: perspevtive to orthorgonal projection
    Eigen::Matrix4f P2O = Eigen::Matrix4f::Identity();
    P2O << n, 0, 0, 0,
           0, n, 0, 0,
           0, 0, n + f, -n * f,
           0, 0, 1, 0;
    // O mat: orthorgonal mat. O_s: scale mat. O_t: translate mat. O = O_s * O_t.
    Eigen::Matrix4f O_s = Eigen::Matrix4f::Identity();
    O_s <<  2 / (r - l),        0,         0,      0,
            0,          2 / (t - b),       0,      0,
            0,                  0, 2 / (n - f),    0,
            0,                  0,          0,     1;

    Eigen::Matrix4f O_t = Eigen::Matrix4f::Identity();
    O_t << 1,       0,      0,      -1 * (r + l) / 2,
           0,       1,      0,      -1 * (t + b) / 2,
           0,       0,      1,      -1 * (n + f) / 2,
           0,        0,      0,      1;

    Eigen::Matrix4f O = O_s * O_t;
    projection = O * P2O * projection;

    return projection;                                          
}

Eigen::Matrix4f Transformation::get_screen_matrix(const int& width, const int& height) { // viewport transform. From canonical cube to screen
	Eigen::Matrix4f screen = Eigen::Matrix4f::Identity();
	screen << width / 2, 0,     0, 	    width / 2,
		      0,	height / 2, 0, 		height / 2,
		      0,		 0,		1,		0,
		      0,		 0,		0,		1;
	return screen;
}

