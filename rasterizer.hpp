//
// rasterizer header.
// z78406. 2022/04/05


#ifndef RASTERIZER_H
#define RASTERIZER_H

#include <eigen3/Eigen/Eigen>
#include <optional>
#include <algorithm>
#include "global.hpp"
#include "Shader.hpp"
#include "Triangle.hpp"
using namespace Eigen;
using namespace rst 
{

class rasterizer {
public:
	rasterizer();
	void drawLine(Eigen::Vector3f begin, Eigen::Vector3f end);
	void drawTriangle(std::vector<Triangle*> &TriangleList);
	void showImage(cv::Mat image);	
	void set_pixel(const Vector2i& point, const Eigen::Vector3f& color);	
	void set_texture(Texture tex) {texture = tex; }					

private:
	std::std::vector<Eigen::Vector3f> frame_buf; 										// save screen data
	int width, height;																	// screen size
	std::optional<Texture> texture;														// optional class

};

}

#endif // RASTERIZER_H