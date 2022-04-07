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

	enum class Buffers {
		Color = 1,
		Depth = 2
		inline Buffers operator|(Buffers a, Buffers b) {
			return Buffers((int)a | (int)b);
		}
		inline Buffers operator&(Buffers a, Buffers b) {
	        return Buffers((int)a & (int)b);
	    }	
	};

	enum class Primitive {
		Line,
		Triangle
	}


	class rasterizer {
	public:
		rasterizer();
        void draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type);
        void draw(std::vector<Triangle *> &TriangleList);		
		void drawLine(Eigen::Vector3f begin, Eigen::Vector3f end);
		void rst::rasterizer::drawLine_midpoint(Eigen::Vector3f begin, Eigen::Vector3f end);
		void drawTriangle(std::vector<Triangle*> &TriangleList);

		void showImage(cv::Mat image);	

		void clear(Buffers buff);
		void set_pixel(const Vector2i& point, const Eigen::Vector3f& color);	
		void set_texture(Texture tex) {texture = tex; }		
	    void set_vertex_shader(std::function<Eigen::Vector3f(vertex_shader_payload)> vert_shader);
	    void set_fragment_shader(std::function<Eigen::Vector3f(fragment_shader_payload)> frag_shader);	

        void set_model(const Eigen::Matrix4f& m);
        void set_view(const Eigen::Matrix4f& v);
        void set_projection(const Eigen::Matrix4f& p);



	private:
		// transformation matrix
	    Eigen::Matrix4f model;
	    Eigen::Matrix4f view;
	    Eigen::Matrix4f projection;	

		std::std::vector<Eigen::Vector3f> frame_buf; 										// save screen data
		std::vector<float> depth_buf;														// save screen pixel depth
	    std::map<int, std::vector<Eigen::Vector3f>> pos_buf;								// save vertex pos
	    std::map<int, std::vector<Eigen::Vector3i>> ind_buf;								// save vertex ind (0,1,2 for triangle)
	    std::map<int, std::vector<Eigen::Vector3f>> col_buf;								// save vertex color
	    std::map<int, std::vector<Eigen::Vector3f>> nor_buf;								// save vertex normal
		int width, height;																	// screen size
		std::optional<Texture> texture;														// optional class texture (read uv colormap)

	    std::function<Eigen::Vector3f(fragment_shader_payload)> fragment_shader;			// fragment shader in shader.hpp
	    std::function<Eigen::Vector3f(vertex_shader_payload)> vertex_shader;				// vertex shader in shader.hpp
	};

}

#endif // RASTERIZER_H