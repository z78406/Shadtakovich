//
// rasterizer header.
// z78406. 2022/04/05

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

private:
	std::std::vector<Eigen::Vector3f> frame_buf; 										// save screen data
	int width, height;																	// screen size
	
};

}