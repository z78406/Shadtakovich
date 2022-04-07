#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>


class Texture{
private:
	cv::Mat image_data; // container of uv map (e.g. normal/texture map)

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
		// auto v_img = v_img = v * height;    // if coord origin in the left-top
		auto v_img = (1 - v) * height;   // if coord origin in the left-bot (like defined in Games 101 HW3)
		auto color = image_data.at<cv::Vec3b> (v_img, u_img);
		return Eigen::Vector3f(color[0], color[1], color[2]);
	}

	Eigen::Vector3f getColorBilinear(float u, float v) { // get color of the point (v, u) from interpolation
		u = std::clmap(u, 0.0f, 1.0f);
		v = std::clmap(v, 0.0f, 1.0f);
		auto u_img = u * width;
		// auto v_img = v_img = v * height;    // if coord origin in the left-top
		auto v_img = (1 - v) * height;   // if coord origin in the left-bot (like defined in Games 101 HW3)
        auto u_min=std::floor(u_img);
        auto u_max=std::ceil(u_img);
        auto v_min=std::floor(v_img);
        auto v_max=std::ceil(v_img);

        // Color_11 Color_12
        // Color_21 Color_22
        // v: height, u: width
        auto color_11=image_data.at<cv::Vec3b>(v_min,u_min);
        auto color_12=image_data.at<cv::Vec3b>(v_min,u_max);
        auto color_21=image_data.at<cv::Vec3b>(v_max,u_min);
        auto color_22=image_data.at<cv::Vec3b>(v_max,u_max);      
        auto color1=(u_img - u_min) * color_11 + (u_max - u_img) * color_12;
        auto color2=(u_img - u_min) * color_21 + (u_max - u_img) * color_22;        
        auto color=(v_img - v_min) * color1+(v_max - v_img) * color2;
        return Eigen::Vector3f(color[0], color[1], color[2]);        
	}

};





#endif //RASTERIZER_TEXTURE_H