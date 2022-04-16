//
// triangle header.
// z78406. 2022/04/05

#ifndef RASTERIZER_TRIANGLE_H
#define RASTERIZER_TRIANGLE_H

#include <eigen3/Eigen/Eigen>
#include "Texture.hpp"

using namespace Eigen;

class Triangle{
public:
    // store vertex properties in the array 
    Vector4f v[3]; /*the original coordinates of the triangle, v0, v1, v2 in counter clockwise order*/
    Vector3f color[3]; //color at each vertex;
    Vector2f tex_coords[3]; //texture u,v
    Vector3f normal[3]; //normal vector for each vertex

    //// store vertex properties in the matrix
	// MatrixXf v = MatrixXf(3,4); 			/*the original coordinates of the triangle, v0, v1, v2 in counter clockwise order*/
	// MatrixXf color =  MatrixXf(3,3); //color at each vertex;
	// MatrixXf tex_coords =  MatrixXf(3,2);		//texture u,v
	// MatrixXf normal =  MatrixXf(3,3);		//normal vector for each vertex


	Texture* tex = nullptr;
	Triangle();
    Vector4f get_first() const { return v[0]; } // return first vertex of the triangle
    Vector4f get_second() const{ return v[1]; } // return second vertex of the triangle
    Vector4f get_third() const { return v[2]; } // return thrid vertex of the triangle

    void setVertex(int ind, Vector4f ver); /*set i-th vertex coordinates */
    void setNormal(int ind, Vector3f n);   /*set i-th vertex normal vector*/
    void setColor(int ind, float r, float g, float b); /*set i-th vertex color*/

    void setNormals(const std::array<Vector3f, 3>& normals);
    void setColors(const std::array<Vector3f, 3>& colors);
    void setTexCoord(int ind,Vector2f uv ); /*set i-th vertex texture coordinate*/
    std::array<Vector4f, 3> toVector4() const;	



};

#endif //RASTERIZER_TRIANGLE_H