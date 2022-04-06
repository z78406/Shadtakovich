#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>


class Texture{
private:
	cv::Mat image_data;

public:
	int width, height;

	Texture(const std::string& name) {
		image_data = cv::imread(name);
		width = image_data.cols;
		height= image_data.rows;
	}

	 Eigen::Vector3f getColor(float u, float v) {
	 	
	 }

};





#endif //RASTERIZER_TEXTURE_H