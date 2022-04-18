//
// rasterizer header.
// z78406. 2022/04/05

#pragma once
#ifndef RASTERIZER_H
#define RASTERIZER_H

#include <eigen3/Eigen/Eigen>
#include <optional>
#include <algorithm>
#include "global.hpp"
#include "Shader.hpp"
#include "Triangle.hpp"
using namespace Eigen;

namespace rst 
{

	enum class Buffers {
		Color = 1,
		Depth = 2	
	};

	inline Buffers operator|(Buffers a, Buffers b) {
		return Buffers((int)a | (int)b);
	}
	inline Buffers operator&(Buffers a, Buffers b) {
		return Buffers((int)a & (int)b);
	}

	enum class Primitive {
		Line,
		Triangle
	};

    struct pos_buf_id
    {
        int pos_id = 0;
    };

    struct ind_buf_id
    {
        int ind_id = 0;
    };

    struct col_buf_id
    {
        int col_id = 0;
    };

	class rasterizer {
	public:
		rasterizer(int w, int h);
	    pos_buf_id load_positions(const std::vector<Eigen::Vector3f>& positions);
        ind_buf_id load_indices(const std::vector<Eigen::Vector3i>& indices);
        col_buf_id load_colors(const std::vector<Eigen::Vector3f>& colors);
        col_buf_id load_normals(const std::vector<Eigen::Vector3f>& normals);	
        void drawTriangle(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type);
		void drawTriangle(std::vector<Triangle*> &TriangleList);										// draw each triangle from list
		void drawLine(Eigen::Vector3f begin, Eigen::Vector3f end, Eigen::Vector3f line_color);			// Heuristic line drawing
		void drawLine_midpoint(Eigen::Vector3f begin, Eigen::Vector3f end, Eigen::Vector3f line_color); // mid-point line drawing
		void draw_line(Eigen::Vector3f begin, Eigen::Vector3f end, Eigen::Vector3f line_color);			// Bresenham's line drawing 

		void showImage(cv::Mat image);	

		void clear(Buffers buff);
		void set_pixel(const Vector2i& point, const Eigen::Vector3f& color);	
		void set_texture(Texture tex) { texture = tex; }		
	    void set_vertex_shader(std::function<Eigen::Vector3f(vertex_shader_payload)> vert_shader);
	    void set_fragment_shader(std::function<Eigen::Vector3f(fragment_shader_payload)> frag_shader);	

        void set_model(const Eigen::Matrix4f& m);
        void set_view(const Eigen::Matrix4f& v);
        void set_projection(const Eigen::Matrix4f& p);
        void set_screen(const Eigen::Matrix4f& s);

		std::vector<Eigen::Vector3f>& frame_buffer() { return frame_buf; }					// get frame_buf data
		std::vector<int> get_size() {return std::vector<int>(width, height); }				// get window size


	private:
		// transformation matrix
	    Eigen::Matrix4f model;
	    Eigen::Matrix4f view;
	    Eigen::Matrix4f projection;	
	    Eigen::Matrix4f screen;	

		std::vector<Eigen::Vector3f> frame_buf; 											// save screen data
		std::vector<float> depth_buf;														// save screen pixel depth
	    std::map<int, std::vector<Eigen::Vector3f>> pos_buf;								// save vertex pos
	    std::map<int, std::vector<Eigen::Vector3i>> ind_buf;								// save vertex ind (0,1,2 for triangle)
	    std::map<int, std::vector<Eigen::Vector3f>> col_buf;								// save vertex color
	    std::map<int, std::vector<Eigen::Vector3f>> nor_buf;								// save vertex normal

		std::optional<Texture> texture;														// optional class texture (read uv colormap)
	    std::function<Eigen::Vector3f(fragment_shader_payload)> fragment_shader;			// fragment shader in shader.hpp
	    std::function<Eigen::Vector3f(vertex_shader_payload)> vertex_shader;				// vertex shader in shader.hpp
		void rasterize_triangle(const Triangle& t, const std::array<Eigen::Vector3f, 3>& world_pos);															// screen size
        int normal_id = -1;
		int get_index(int x, int y);
        int next_id = 0;
        int get_next_id() { return next_id++; }	
		int width, height;			
	};

}

#endif // RASTERIZER_H