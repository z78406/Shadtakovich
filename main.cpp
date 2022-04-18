#include <iostream>
#include <opencv2/opencv.hpp>

#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"

inline double Degree(double angle)  {return angle*MY_PI/180.0;}

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;

    return view;
}

Eigen::Matrix4f get_model_matrix_y(float angle) { // from world space to camera space

    Eigen::Matrix4f rotation;
    angle = angle * MY_PI / 180.f;
    rotation << cos(angle), 0, sin(angle), 0,
                0, 1, 0, 0,
                -sin(angle), 0, cos(angle), 0,
                0, 0, 0, 1;

    Eigen::Matrix4f scale;
    scale << 2.5, 0, 0, 0,
              0, 2.5, 0, 0,
              0, 0, 2.5, 0,
              0, 0, 0, 1;

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    return translate * rotation * scale;
}

Eigen::Matrix4f get_model_matrix_rotat_x(float angle) { // from world space to camera space

    Eigen::Matrix4f rotation;
    angle = angle * MY_PI / 180.f;
    rotation << 1, 0, 0, 0,
                0, cos(angle), -sin(angle), 0,
                0, sin(angle), cos(angle), 0,
                0, 0, 0, 1;

    Eigen::Matrix4f scale;
    scale << 2.5, 0, 0, 0,
              0, 2.5, 0, 0,
              0, 0, 2.5, 0,
              0, 0, 0, 1;

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    return translate * rotation * scale;
}

Eigen::Matrix4f get_model_matrix_rotate_z(float angle) { // from world space to camera space

    Eigen::Matrix4f rotation;
    angle = angle * MY_PI / 180.f;
    rotation << cos(angle), -sin(angle),0, 0,
                sin(angle), cos(angle), 0, 0,
                0, 			0, 			1, 0,
                0, 			0, 			0, 1;

    Eigen::Matrix4f scale;
    scale << 2.5, 0, 0, 0,
              0, 2.5, 0, 0,
              0, 0, 2.5, 0,
              0, 0, 0, 1;

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    return translate * rotation * scale;
}



Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
	// projection matrix: first perspective then orthorgonal projection.
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // renaming params.
    float n = zNear, f = zFar, e = eye_fov, as = aspect_ratio;
    float half_angle = eye_fov / 2.0 / 180.0 * MY_PI;
    float t = n * std::tan(half_angle); // similar triangles.
    float r = t * as;
    float l = -1 * r;
    float b = -1 * t;
    // P2O mat: perspevtive to orthorgonal projection
    Eigen::Matrix4f P2O = Eigen::Matrix4f::Identity();
    P2O << n, 0, 0, 0,
           0, n, 0, 0,
           0, 0, n + f, -n * f,
           0, 0, 1, 0;
    // O mat: orthorgonal mat. O_s: scale mat. O_t: translate mat. O = O_s * O_t.
    Eigen::Matrix4f O_s = Eigen::Matrix4f::Identity();
    O_s <<  2 / (r - l),        0,         0,      0,
            0,          2 / (t - b),       0,      0,
            0,                  0, 2 / (n - f),    0,
            0,                  0,          0,     1;

    Eigen::Matrix4f O_t = Eigen::Matrix4f::Identity();
    O_t << 1,       0,      0,      -1 * (r + l) / 2,
           0,       1,      0,      -1 * (t + b) / 2,
           0,       0,      1,      -1 * (n + f) / 2,
           0,        0,      0,      1;

    Eigen::Matrix4f O = O_s * O_t;
    projection = O * P2O * projection;

    return projection;
}

Eigen::Matrix4f get_screen_matrix(const int& width, const int& height) {
	Eigen::Matrix4f screen = Eigen::Matrix4f::Identity();
	screen << width / 2, 0,     0, 	    width / 2,
		      0,	height / 2, 0, 		height / 2,
		      0,		 0,		1,		0,
		      0,		 0,		0,		1;
	return screen;
}

struct light
{
    Eigen::Vector3f position;
    Eigen::Vector3f intensity;
};

Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload) { // a child class of fragment_shader_payload
    Eigen::Vector3f return_color = {0, 0, 0};
    if (payload.texture)
    {
        // TODO: Get the texture value at the texture coordinates of the current fragment
        // without interpolation
        return_color=payload.texture->getColor(payload.tex_coords.x(), payload.tex_coords.y());
        // with interpolation
        return_color=payload.texture->getColorBilinear(payload.tex_coords.x(), payload.tex_coords.y());
    }
    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = texture_color / 255.f;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = texture_color;
    Eigen::Vector3f point = payload.p_3d;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};

    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.

        Eigen::Vector3f light_dir = light.position - point; // from surface point to light
        Eigen::Vector3f view_dir  = eye_pos - point;        // from surface point to view

        float r_square = light_dir.dot(light_dir); // decay r^2 given the distance

        // note: l, v should be normalized
        light_dir = light_dir.normalized();
        view_dir = view_dir.normalized();

        // note: ka, kd, ks are scalars in the lecture, but they are vectors here, so you should use cwiseProduct
        // ambient La=ka*Ia
        Eigen::Vector3f La = ka.cwiseProduct(amb_light_intensity); // cwiseProduct--dot product

        // diffuse Ld=kd(I/r^2)max(0, nl)
        Eigen::Vector3f Ld = kd.cwiseProduct(light.intensity / r_square) * std::max(0.0f, normal.normalized().dot(light_dir));

        // specular Ls=ks(I/r^2)max(0, nh)^p
        Eigen::Vector3f h_dir = (light_dir + view_dir).normalized();
        Eigen::Vector3f Ls = ks.cwiseProduct(light.intensity / r_square) * std::pow(std::max(0.0f, normal.normalized().dot(h_dir)), p);
        
        result_color += (La + Ld + Ls);
    }

    return result_color * 255.f;	
}


Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005); // ambient
    Eigen::Vector3f kd = payload.color;                        // diffuse. Read from setColor in drawTriangle()
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937); // specular

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150; // power factor of the specular term

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.p_3d;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};
    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.

        Eigen::Vector3f light_dir = light.position - point; // from surface point to light
        Eigen::Vector3f view_dir  = eye_pos - point;        // from surface point to view

        float r_square = light_dir.dot(light_dir); // decay r^2 given the distance

        // note: l, v should be normalized
        light_dir = light_dir.normalized();
        view_dir = view_dir.normalized();

        // note: ka, kd, ks are scalars in the lecture, but they are vectors here, so you should use cwiseProduct
        // ambient La=ka*Ia
        Eigen::Vector3f La = ka.cwiseProduct(amb_light_intensity); // cwiseProduct--dot product

        // diffuse Ld=kd(I/r^2)max(0, nl)
        Eigen::Vector3f Ld = kd.cwiseProduct(light.intensity / r_square) * std::max(0.0f, normal.normalized().dot(light_dir));

        // specular Ls=ks(I/r^2)max(0, nh)^p
        Eigen::Vector3f h_dir = (light_dir + view_dir).normalized();
        Eigen::Vector3f Ls = ks.cwiseProduct(light.intensity / r_square) * std::pow(std::max(0.0f, normal.normalized().dot(h_dir)), p);
        
        result_color += (La + Ld + Ls);
    }
    return result_color * 255.f;
}


Eigen::Vector3f vertex_shader(const vertex_shader_payload& payload) {
    return payload.position;
}





int main(int argc, const char** argv) {

	// define rasterizer class 
	int screen_width = 700, screen_height = 700;
	rst::rasterizer r(screen_width, screen_height); // screen object and its size
    float angle = 140.0;										// set view-angle
    bool command_line = false;									// set cmd line flag
    Eigen::Vector3f eye_pos = {0,0,10};							// set view position    
	std::vector<Triangle*> TriangleList;						// triangle face container
    std::string filename = "output.png";
	objl::Loader Loader;

	 bool loadout = Loader.LoadFile("../models/spot/spot_triangulated_good.obj");
	 for(auto mesh : Loader.LoadedMeshes) {
	 	for (int i = 0; i < mesh.Vertices.size(); i += 3) { // each face(triangel)
	 		Triangle* t = new Triangle();
	 		for (int j = 0; j < 3; j++) { // each vertex
	 			// set vertex
                t->setVertex(j,Vector4f(mesh.Vertices[i+j].Position.X,mesh.Vertices[i+j].Position.Y,mesh.Vertices[i+j].Position.Z,1.0));
                t->setNormal(j,Vector3f(mesh.Vertices[i+j].Normal.X,mesh.Vertices[i+j].Normal.Y,mesh.Vertices[i+j].Normal.Z));
                t->setTexCoord(j,Vector2f(mesh.Vertices[i+j].TextureCoordinate.X, mesh.Vertices[i+j].TextureCoordinate.Y));
                // std::cout<< t->v[0]<<std::endl;
                // return -1;
	 		}
	 		TriangleList.push_back(t);
	 	}
	}




    std::string texture_path = "../models/spot/hmap.jpg"; 			// default colormap
    r.set_texture(Texture(texture_path));
     // define a polymorphism function that returns type: Eigen::Vector3f.
    // fragment_shader_payload is the parameters shared by all the member (that would be called)
    // actually, variable active_shader (type Eigen::Vector3f) switches between different instances
    // e.g. phong/texture/normal shader.
    std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = phong_fragment_shader;   

    if (argc >= 2) {
        command_line = true;
        filename = std::string(argv[1]);
        if (argc == 3 && std::string(argv[2]) == "texture") { 	// apply texture map
            std::cout << "Rasterizing using the texture shader\n";
            active_shader = texture_fragment_shader;
            texture_path = "../models/spot/spot_texture.png";
            r.set_texture(Texture(texture_path));
        }            	
    }	
    // set specified shader type from input
    r.set_vertex_shader(vertex_shader);							// set vertex shader
    r.set_fragment_shader(active_shader);    					// set fragment shader 

    if (command_line) { 										// extra command from cmd line
    	r.clear(rst::Buffers::Color | rst::Buffers::Depth);     // reset buffer
    	r.set_model(get_model_matrix_y(angle));						// world to camera space
        r.set_view(get_view_matrix(eye_pos));						// camera to camera canonical
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));    	// camera canonical to cube 
        r.set_screen(get_screen_matrix(screen_width, screen_height));				// cube to screen
        r.drawTriangle(TriangleList);									// draw every face into screen
        cv::Mat image(screen_width, screen_height, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imwrite(filename, image);  

        return 0;      
    }
    int key = 0;
    while(key != 27) {
    	r.clear(rst::Buffers::Color | rst::Buffers::Depth);     // reset buffer
    	r.set_model(get_model_matrix_y(angle));						// world to camera space
        r.set_view(get_view_matrix(eye_pos));						// camera to camera canonical
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));    	// camera canonical to cube 
        r.set_screen(get_screen_matrix(screen_width, screen_height));				// cube to screen
        r.drawTriangle(TriangleList);									// draw every face into screen
        cv::Mat image(screen_width, screen_height, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imshow("image", image);
        cv::imwrite(filename, image);
        key = cv::waitKey(10);

        if (key == 'a' )
        {
            angle -= 0.1;
        }
        else if (key == 'd')
        {
            angle += 0.1;
        }
    }    

	return 0;
}