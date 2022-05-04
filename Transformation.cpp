//
//
// Transformation class main file.
// 2022/04/22


#include <opencv2/opencv.hpp>
#include <math.h>
#include "Transformation.hpp"
#include "global.hpp"

inline double Degree(double angle)  {return angle*MY_PI/180.0;}



Eigen::Matrix4f Transformation::get_view_matrix(const Eigen::Vector3f& eye_pos,  const Eigen::Vector3f& gaze = {0, 0, -1}, 
                                                const Eigen::Vector3f& up = {0,1,0}) {
                                                                    // canonical transform: camera to camera canonical
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f translate;
    Eigen::Matrix4f rotate;
    Eigen::Vector3f gaze_norm = gaze.normalized();
    Eigen::Vector3f up_norm = up.normalized();
    translate << 1,0,0,-eye_pos[0],
                0,1,0,-eye_pos[1],
                0,0,1,-eye_pos[2],
                0,0,0,1;   
    
    rotate << gaze.cross(up).x(), up_norm.x(), gaze_norm.x(), 0, // g at -z, x at up
              gaze.cross(up).y(), up_norm.y(), gaze_norm.y(), 0,
              gaze.cross(up).z(), up_norm.z(), gaze_norm.z(), 0,
              0,                0,    0,           1;

    view = rotate.transpose() * translate * view; // rotate.inverse() == rotate.transpose()
    return view;
}

Eigen::Matrix4f Transformation::get_model_matrix(const Eigen::Vector3f& agl, 
                                                const Eigen::Vector3f& scl = {2.5, 2.5, 2.5}, const Eigen::Vector3f& tsl = {0, 0, 0}) 
{ // model transform: world to camera coord

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
    translate << 1, 0, 0, tsl.x(),
            0, 1, 0, tsl.y(),
            0, 0, 1, tsl.z(),
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
    float t = n * std::tan(half_angle); // top using similar triangles.
    float r = t * as; // right
    float l = -1 * r; // left
    float b = -1 * t; // bottom
    // P2O mat: perspevtive to orthorgonal projection
    Eigen::Matrix4f P2O = Eigen::Matrix4f::Identity();
    P2O << n, 0, 0, 0,
           0, n, 0, 0,
           0, 0, n + f, -n * f,
           0, 0, 1, 0;
    // O mat: orthorgonal mat. O_s: scale mat. O_t: translate mat. O = O_s * O_t.
    Eigen::Matrix4f O_s = Eigen::Matrix4f::Identity(); // Trick here. use 2 / (n - f) to convert z axis from negative (-z view) tp positive value.
    O_s <<  2 / (r - l),        0,         0,      0,
            0,          2 / (t - b),       0,      0,
            0,                  0, 2 / (f - n),    0,
            0,                  0,          0,     1;

    Eigen::Matrix4f O_t = Eigen::Matrix4f::Identity(); 
    O_t << 1,       0,      0,      -1 * (r + l) / 2,
           0,       1,      0,      -1 * (t + b) / 2,
           0,       0,      1,      -1 * (f + n) / 2,
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

Eigen::Vector3f Transformation::get_up_vector(const Eigen::Vector3f& eye_pos, const Eigen::Vector3f& origin) { 
    // return camera's up vector in world coordinate system given its position: eye_pos and its gaze: assume to gaze at world origin.
    return {0,0,0};
}


