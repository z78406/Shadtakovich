//
// rasterizer part.
// z78406. 2022/04/05


#include <algorithm>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


void rst::rasterizer::drawLine(Eigen::Vector3f begin, Eigen::Vector3f end, Eigen::Vector3f line_color = {255, 255, 255}) {
	// note we need to convert any line into a canonical 
	// form: 1. abs(x2 - x1) > abs(y2 - y1) 2. x2 > x1 3. slope is okay if its' < 0.
    auto x1 = begin.x();
    auto y1 = begin.y();
    auto x2 = end.x();
    auto y2 = end.y();	
	bool steep = false;
	// rule 1
	if (std::abs(x2 - x1) < std::abs(y2 - y1)) {
		steep = true;
		std::swap(x1, y1);
		std::swap(x2, y2);		
	}
	// rule 2
	if (x1 > x2) {
		std::swap(x1, x2);
		std::swap(y1, y2);
	}

	float slope = std::abs((y2 - y1) / (x2 - x1));
	float error = 0;
	const int yincrs = y2 > y1? 1: -1; // increase / derease of y
	Eigen::Vector2i point = Eigen::Vector2i; // next line point position

	if (steep) {		
		for (int x = x1; x <= x2; x++) {
			point[0] = y, point[1] = x;
			set_pixel(point, line_color);
			error += slope;
			// over midpoint
			if (error > 0.5) {
				y += yincrs;
				error -= 1;
			}
		}
	}

	else {
		for (int x = x1; x <= x2; x++) {
			point[0] = x, point[1] = y;
			set_pixel(point, line_color);
			error += slope;
			// over midpoint
			if (error > 0.5) {
				y += yincrs;
				error -= 1;
			}
		}
	}
}

void rst::rasterizer::drawLine_midpoint() {
	// midpoint method to draw line
}

void rst::rasterizer::drawTriangle() {
	
}

void rst::rasterizer::showImage(cv::Mat image) {
	if(img.empty())
		std::cout << "Could not read the image:" << endl;
	if (image.type == 'CV_32FC3')
		image.convertTo(image, CV_8UC3, 1.0f);
	cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
	cv::imshow("image", image);
	cv::waitKey(0);
    return;
}

// set a pixel in the screen its color into the buffer.
void rst::rasterizer::set_pixel(const Vector2i& point, const Eigen::Vector3f& color) { 
	int ind = (height - point.y()) * width + point.x();
	frame_buf[ind] = color;
}