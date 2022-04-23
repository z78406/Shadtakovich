//
// z78406. 2022/04/07
//


#ifndef RASTERIZER_SHADER_H
#define RASTERIZER_SHADER_H
#include <eigen3/Eigen/Eigen>
#include "Texture.hpp"

struct fragment_shader_payload
{
    fragment_shader_payload()
    {
        texture = nullptr;
    }

    fragment_shader_payload(const Eigen::Vector3f& p_3d, const Eigen::Vector3f& pixel_3d,
                            const Eigen::Vector3f& col, const Eigen::Vector3f& nor,const Eigen::Vector2f& tc, Texture* tex) :
         p_3d(p_3d), pixel_3d(pixel_3d), color(col), normal(nor), tex_coords(tc), texture(tex) {}

    Eigen::Vector3f p_3d;                   // points 3D (points after view/model transform)
    Eigen::Vector3f pixel_3d;               // pixel in 3D (points after screen/projection/view/model transofrm)
    Eigen::Vector3f color;
    Eigen::Vector3f normal;
    Eigen::Vector2f tex_coords;
    Texture* texture;
};

struct vertex_shader_payload
{
    Eigen::Vector3f position;

    vertex_shader_payload(const Eigen::Vector3f pos) : position(pos) {}
};



#endif //RASTERIZER_SHADER_H