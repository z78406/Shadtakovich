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

	Eigen::Vector3f getColor(float u, float v) { // get color of the point (v, u) from direct index ref
		u = std::clmap(u, 0.0f, 1.0f);
		v = std::clmap(v, 0.0f, 1.0f);
		auto u_img = u * width;
		auto v_img = v_img = v * height;    // if coord origin in the left-top
		// auto v_img = (1 - v) * height;   // if coord origin in the left-bot (like defined in Games 101 HW3)
		auto color = image_data.at<cv::Vec3b> (v_img, u_img);
		return Eigen::Vector3f(color[0], color[1], color[2]);
	}

	Eigen::Vector3f getColorBilinear(float u, float v) { 

	}

};





#endif //RASTERIZER_TEXTURE_H