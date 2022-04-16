//
// rasterizer part.
// z78406. 2022/04/05


#include <algorithm>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h) {
	frame_buf.resize(w * h);
	depth_buf.resize(w * h);

	texture = std::nullopt;
}

void rst::rasterizer::clear(rst::Buffers buff) { 						// reset frame/depth buffer in rasterizer
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }	
}

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions) {
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices) {
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols) {
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_normals(const std::vector<Eigen::Vector3f>& normals) {
    auto id = get_next_id();
    nor_buf.emplace(id, normals);

    normal_id = id;

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f) {
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

// 3d interpolate
static Eigen::Vector3f interpolate(float alpha, float beta, float gamma, const Eigen::Vector3f& vert1, const Eigen::Vector3f& vert2, const Eigen::Vector3f& vert3, float weight)
{
    return (alpha * vert1 + beta * vert2 + gamma * vert3) / weight;
}

// 2d interpolate
static Eigen::Vector2f interpolate(float alpha, float beta, float gamma, const Eigen::Vector2f& vert1, const Eigen::Vector2f& vert2, const Eigen::Vector2f& vert3, float weight)
{
    auto u = (alpha * vert1[0] + beta * vert2[0] + gamma * vert3[0]);
    auto v = (alpha * vert1[1] + beta * vert2[1] + gamma * vert3[1]);

    u /= weight;
    v /= weight;

    return Eigen::Vector2f(u, v);
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m) {				// world to camera
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v) {				// camera to camera canonical
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p) {		// camera canonical to cube
    projection = p;
}

void rst::rasterizer::set_screen(const Eigen::Matrix4f& p) {			// canonical cube to screen
    screen = p;
}

static bool insideTriangle(float x, float y, const Vector4f* _v) {
    Vector3f v[3];
    for(int i=0;i<3;i++)
        v[i] = {_v[i].x(),_v[i].y(), 1.0};
    Vector3f f0,f1,f2;
    f0 = v[1].cross(v[0]);
    f1 = v[2].cross(v[1]);
    f2 = v[0].cross(v[2]);
    Vector3f p(x,y,1.);
    if((p.dot(f0)*f0.dot(v[2])>0) && (p.dot(f1)*f1.dot(v[0])>0) && (p.dot(f2)*f2.dot(v[1])>0))
        return true;
    return false;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector4f* v){
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::drawTriangle(std::vector<Triangle*> &TriangleList) { // draw every triangle face
	Eigen::Matrix4f mvps = screen * projection * view * model; 
	// 1. read each triangle.
	// 2. homogeneous division.
	// 3. rasterize each triangle.
	for (const auto& t : TriangleList) {
		Triangle newtri = *t;
		std::array<Eigen::Vector4f, 3> mm {
			(view * model * t->v[0]),
			(view * model * t->v[1]),
			(view * model * t->v[2])
		};

		std::array<Eigen::Vector3f, 3> p_3d;
		std::transform(mm.begin(), mm.end(), p_3d.begin(), [](auto& v) {
			return v.template head<3>();
		});

		Eigen::Vector4f v[] = {
			mvps * t->v[0],
			mvps * t->v[1],
			mvps * t->v[2]
		};
		// vertex
		for (auto& vec : v) {
			vec.x() /= vec.w();
			vec.y() /= vec.w();
			vec.z() /= vec.w();
		}
		// normal
		Eigen::Matrix4f inv_trans = (view * model).inverse().transpose();
        Eigen::Vector4f n[] = {
                inv_trans * to_vec4(t->normal[0], 0.0f),
                inv_trans * to_vec4(t->normal[1], 0.0f),
                inv_trans * to_vec4(t->normal[2], 0.0f)
        };
        // set it to class triangle
        for (int i = 0; i < 3; i++) {
        	newtri.setVertex(i, v[i]);
        	newtri.setNormal(i, n[i].head<3>());
        }	
        newtri.setColor(0, 148,121.0,92.0);
        newtri.setColor(1, 148,121.0,92.0);
        newtri.setColor(2, 148,121.0,92.0);        
        // Also pass view space vertice position
        rasterize_triangle(newtri, p_3d);
	}

}

void rst::rasterizer::rasterize_triangle(const Triangle& t, const std::array<Eigen::Vector3f, 3>& p_3d) { // draw triangle on the screen given its 3 vertices
	// 1. use Barycentric to interpolate the value of pixels from 3 vertices.
	// 2. use depth-test to save nearest pixels into the depth-buffer.
	auto v = t.toVector4();
    float x_min=std::min({v[0][0], v[1][0], v[2][0]});
    float x_max=std::max({v[0][0], v[1][0], v[2][0]});
    float y_min=std::min({v[0][1], v[1][1], v[2][1]});
    float y_max=std::max({v[0][1], v[1][1], v[2][1]});  

    x_min = (int)std::floor(x_min);
    x_max = (int)std::ceil(x_max);
    y_min = (int)std::floor(y_min);
    y_max = (int)std::ceil(y_max);  

   // without anti-alising
    for(int x=x_min; x<=x_max; x++)
    {
        for(int y=y_min; y<=y_max; y++)
        {
            // we need to decide whether this point is actually inside the triangle
            if(!insideTriangle((float)x+0.5,(float)y+0.5,t.v))    
                continue; // note: we use x+0.5 here to improve the precision
            // get z value--depth
            // If so, use the following code to get the interpolated z value.
            auto [alpha, beta, gamma] = computeBarycentric2D(x + 0.5, y + 0.5, t.v);

            float Z = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
            float zp = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
            zp *= Z;

            // compare the current depth with the value in depth buffer
            if(depth_buf[get_index(x,y)] > zp)// note: we use get_index to get the index of current point in depth buffer
            {
                // we have to update this pixel
                depth_buf[get_index(x,y)] = zp; // update depth buffer

                // interpolate color
                auto interpolated_color=interpolate(alpha, beta, gamma, t.color[0], t.color[1], t.color[2], 1);
                // interpolate norm
                auto interpolated_normal=interpolate(alpha, beta, gamma, t.normal[0], t.normal[1], t.normal[2], 1);
                // interpolate texture
                auto interpolated_texcoords=interpolate(alpha, beta, gamma, t.tex_coords[0], t.tex_coords[1], t.tex_coords[2], 1);
                // interpolate shading_coords
				auto interpolated_shadingcoords=interpolate(alpha, beta, gamma, p_3d[0], p_3d[0], p_3d[0], 1);	

				// init shader struct and pass it to the class rasterizer
				fragment_shader_payload payload(interpolated_color, interpolated_normal.normalized(), interpolated_texcoords, texture ? &*texture : nullptr);
				payload.p_3d = interpolated_shadingcoords;
				auto pixel_color = fragment_shader(payload);   // read color from shader (from interpolated_color)              
				// set color
				set_pixel(Eigen::Vector2i(x,y), pixel_color);               
	        }
    	}
    }	
}

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
	Eigen::Vector3f point = Eigen::Vector2i(x1, y1);
	int y = y1;

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

void rst::rasterizer::drawLine_midpoint(Eigen::Vector3f begin, Eigen::Vector3f end, Eigen::Vector3f line_color = {255, 255, 255}) {
	// midpoint method to draw line 
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
	const int yincrs = y2 > y1? 1: -1; // increase / derease of y

	Eigen::Vector3f point = Eigen::Vector2i(x1, y1);

	// Reference: https://www.geeksforgeeks.org/mid-point-line-generation-algorithm/
	int y = y1;						  // y of the initial selected point
	int dy = std::abs(y2 - y1), dx = std::abs(x2 - x1);   // difference in y/x
	int m = dy - dx / 2; 			  // midpoint's representation in f(x,y)	
	if (steep) {		
		for (int x = x1; x <= x2; x++) {
			point[0] = y, point[1] = x;
			set_pixel(point, line_color);
			if (m < 0) 				  // mid point is above line. Choose E as the next point.
				m = m + dy;
			else {
				m = m + dy - dx; 	  // mid point is below line. Choose NE as the next point.
				y = y + yincrs;				
			}
		}
	}

	else {
		for (int x = x1; x <= x2; x++) {
			point[0] = x, point[1] = y;
			set_pixel(point, line_color);
			if (m < 0) 				  // mid point is above line. Choose E as the next point.
				m = m + dy;
			else {
				m = m + dy - dx; 	  // mid point is below line. Choose NE as the next point.
				y = y + yincrs;				
			}
		}
	}	
}



void rst::rasterizer::showImage(cv::Mat image) {
	if(image.empty())
		std::cout << "Could not read the image:" << std::endl;
	if (image.type() == 30) // for 'CV_32FC3'
		image.convertTo(image, CV_8UC3, 1.0f);
	cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
	cv::imshow("image", image);
	cv::waitKey(0);
    return;
}

// set a pixel in the screen its color into the buffer.
void rst::rasterizer::set_pixel(const Vector2i& point, const Eigen::Vector3f& color) { 
    if (point.x() < 0 || point.x() >= width ||
        point.y() < 0 || point.y() >= height) return;
    auto ind = (height-1-point.y())*width + point.x();
	frame_buf[ind] = color;
}

// load vertex shader instance into rasterizer
void rst::rasterizer::set_vertex_shader(std::function<Eigen::Vector3f(vertex_shader_payload)> vert_shader)
{
    vertex_shader = vert_shader;
}

// map 2d coord to 1d index
int rst::rasterizer::get_index(int x, int y)
{
    return (height-y)*width + x;
}


// load fragment shader instance into rasterizer
void rst::rasterizer::set_fragment_shader(std::function<Eigen::Vector3f(fragment_shader_payload)> frag_shader) {
    fragment_shader = frag_shader;
}