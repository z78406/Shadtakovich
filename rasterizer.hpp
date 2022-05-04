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
#include "Light.hpp"
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
		bool read_shadow;					// if read shadow from precomputed results
		bool visible_at_least_one = true;	// if render at least 1 part of the object into the screen
	    pos_buf_id load_positions(const std::vector<Eigen::Vector3f>& positions);
        ind_buf_id load_indices(const std::vector<Eigen::Vector3i>& indices);
        col_buf_id load_colors(const std::vector<Eigen::Vector3f>& colors);
        col_buf_id load_normals(const std::vector<Eigen::Vector3f>& normals);	
        void drawTriangle(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type);
		void drawTriangle(const std::vector<Triangle*> &TriangleList, const light::p_Light& p_l, 
                                    const light::a_Light& a_l, const Eigen::Vector3f& eye_pos);								// draw each triangle from list
		void drawLine(Eigen::Vector3f begin, Eigen::Vector3f end, Eigen::Vector3f line_color);			// Heuristic line drawing
		void drawLine_midpoint(Eigen::Vector3f begin, Eigen::Vector3f end, Eigen::Vector3f line_color); // mid-point line drawing
		void draw_line(Eigen::Vector3f begin, Eigen::Vector3f end, Eigen::Vector3f line_color);			// Bresenham's line drawing 

		void showImage(cv::Mat image);	

		void clear(Buffers buff);
		void clear_shadow();
		void set_msaa(bool use_msaa) {this->use_msaa = use_msaa;}
		void set_msaa_ratio(int ms_ratio) {this->msaa_ratio = ms_ratio;}
		void set_pixel(const Vector2i& point, const Eigen::Vector3f& color);	
		Eigen::Vector3f get_pixel(const Vector2i& point);
		void set_texture(Texture tex) { texture = tex; }		
	    void set_vertex_shader(std::function<Eigen::Vector3f(vertex_shader_payload)> vert_shader);
	    void set_fragment_shader(std::function<Eigen::Vector3f(fragment_shader_payload, light::p_Light, light::a_Light, Eigen::Vector3f)> frag_shader);	

        void set_model(const Eigen::Matrix4f& m);
        void set_view(const Eigen::Matrix4f& v);
        void set_projection(const Eigen::Matrix4f& p);
        void set_screen(const Eigen::Matrix4f& s);
		void set_cam2light(const Eigen::Matrix4f& m);

		std::vector<Eigen::Vector3f>& frame_buffer() { return frame_buf; }						// get frame_buf data
		std::vector<Eigen::Vector3f>& super_frame_buffer() { return super_frame_buf; }			// get super frame_buf data
		std::vector<float>& depth_buffer() { return depth_buf; }								// get depth_buf data	
		std::vector<float>& super_depth_buffer() { return super_depth_buf; }								// get depth_buf data		
		std::vector<Eigen::Vector3f>& shadow_buffer() { return shadow_buf; }					// get shadow_buf data
		std::vector<float>& shadow_depth_buffer() { return shadow_depth_buf; }					// get shadow coord data

		void set_frame_buffer(std::vector<Eigen::Vector3f>& frame_buffer) {frame_buf = frame_buffer;} // set frame buffer
		void set_super_frame_buffer(std::vector<Eigen::Vector3f>& super_frame_buffer) {super_frame_buf = super_frame_buffer;} // set suepr frame buffer		 
		void set_depth_buffer(std::vector<float>& depth_buffer) {depth_buf = depth_buffer;} // set depth buffer 	
		void set_super_depth_buffer(std::vector<float>& super_depth_buffer) {super_depth_buf = super_depth_buffer;} // set depth buffer 	
		void set_shadow_buffer(std::vector<Eigen::Vector3f>& shadow_buffer) {shadow_buf = shadow_buffer;} // set shadow buffer 	
		void set_shadow_depth_buffer(std::vector<float>& shadow_depth_buffer) {shadow_depth_buf = shadow_depth_buffer;} // set shadow coord buffer 				

		std::vector<int> get_size() {return std::vector<int>(width, height); }					// get window size
		std::vector<Eigen::Matrix4f> get_transform() {return std::vector<Eigen::Matrix4f> {model, view, projection, screen, cam2light}; }
		Eigen::Vector4f toVector4 (const Eigen::Vector3f& p3); 									// 3d vec to 4d
		std::vector<Eigen::Vector4f> toVector4(const std::vector<Eigen::Vector3f>& p3);			// 3d vec arr to 4d vec arr
		bool get_msaa_status() {return use_msaa;}
		int get_msaa_ratio() {return msaa_ratio;}
		
	private:
		// transformation matrix
		bool use_msaa = false;
		int msaa_ratio = 4;
	    Eigen::Matrix4f model;
	    Eigen::Matrix4f view;
	    Eigen::Matrix4f projection;	
	    Eigen::Matrix4f screen;	
		Eigen::Matrix4f cam2light;	

		std::vector<Eigen::Vector3f> frame_buf; 											// save screen data
		std::vector<Eigen::Vector3f> super_frame_buf;										// save screen data at msaa sample resolution
		std::vector<float> depth_buf;														// save screen pixel depth
		std::vector<float> super_depth_buf;													// save screen pixel depth at msaa sample resolution
		std::vector<Eigen::Vector3f> shadow_buf; 											// save shadow data from light-view depth computation.
		std::vector<float> shadow_depth_buf;												// save shadow coordinate		
	    std::map<int, std::vector<Eigen::Vector3f>> pos_buf;								// save vertex pos
	    std::map<int, std::vector<Eigen::Vector3i>> ind_buf;								// save vertex ind (0,1,2 for triangle)
	    std::map<int, std::vector<Eigen::Vector3f>> col_buf;								// save vertex color
	    std::map<int, std::vector<Eigen::Vector3f>> nor_buf;								// save vertex normal

		std::optional<Texture> texture;														// optional class texture (read uv colormap)
	    std::function<Eigen::Vector3f(fragment_shader_payload, light::p_Light, light::a_Light, Eigen::Vector3f)> fragment_shader;			// fragment shader in shader.hpp
	    std::function<Eigen::Vector3f(vertex_shader_payload)> vertex_shader;				// vertex shader in shader.hpp
		void rasterize_triangle(const Triangle& t, const std::array<Eigen::Vector3f, 3>& world_pos, const light::p_Light& p_l, 
                                    const light::a_Light& a_l, const Eigen::Vector3f& eye_pos);												// screen size
		Eigen::Vector3f p_trans(const Eigen::Vector3f& p3, const Eigen::Matrix4f& m);														// transform 3d points by mat
		std::vector<Eigen::Vector3f> p_trans(const std::vector<Eigen::Vector3f>& p3, const Eigen::Matrix4f& m);								// transform arr of 3d points by mat
        int normal_id = -1;
		int get_index(int x, int y);
		int get_super_index(int x, int y);
        int next_id = 0;
        int get_next_id() { return next_id++; }	
		int width, height;			
	};

}

#endif // RASTERIZER_H