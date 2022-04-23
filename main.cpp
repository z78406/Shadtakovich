#include <iostream>
#include <opencv2/opencv.hpp>

#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"
#include "Transformation.hpp"
#include "Light.hpp"




Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload, const light::p_Light& p_light, const light::a_Light& a_light, 
                                        const Eigen::Vector3f& eye_pos) { // a child class of fragment_shader_payload. Add texture to shading
    Eigen::Vector3f return_color = {0, 0, 0};
    if (payload.texture) // use texture color as color instead of default interpolated pixel color from vertex.
    {
        // Get the texture value at the texture coordinates of the current fragment
        // without interpolation
        // return_color=payload.texture->getColor(payload.tex_coords.x(), payload.tex_coords.y());
        // with interpolation
        return_color = payload.texture->getColorBilinear(payload.tex_coords.x(), payload.tex_coords.y());
    }
    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = texture_color / 255.f;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);


    // auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    // auto l2 = light{{-20, 20, 0}, {500, 500, 500}};
    // std::vector<light> lights = {l1, l2};   
    // Eigen::Vector3f amb_light_intensity{10, 10, 10}; 
    // Eigen::Vector3f eye_pos{0, 0, 10};

    auto pl_pos = p_light.position, pl_itn = p_light.intensity;
    auto al_itn = a_light.intensity;
    float p = 150;

    Eigen::Vector3f color = texture_color;
    Eigen::Vector3f point = payload.p_3d;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};

    for (int i = 0; i < pl_pos.size(); i++)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.

        Eigen::Vector3f light_dir = pl_pos[i] - point; // from surface point to light
        Eigen::Vector3f view_dir  = eye_pos - point;        // from surface point to view

        float r_square = light_dir.dot(light_dir); // decay r^2 given the distance

        // note: l, v should be normalized
        light_dir = light_dir.normalized();
        view_dir = view_dir.normalized();

        // note: ka, kd, ks are scalars in the lecture, but they are vectors here, so you should use cwiseProduct
        // ambient La=ka*Ia
        Eigen::Vector3f La = ka.cwiseProduct(al_itn); // cwiseProduct--dot product

        // diffuse Ld=kd(I/r^2)max(0, nl)
        Eigen::Vector3f Ld = kd.cwiseProduct(pl_itn[i] / r_square) * std::max(0.0f, normal.normalized().dot(light_dir));

        // specular Ls=ks(I/r^2)max(0, nh)^p
        Eigen::Vector3f h_dir = (light_dir + view_dir).normalized();
        Eigen::Vector3f Ls = ks.cwiseProduct(pl_itn[i] / r_square) * std::pow(std::max(0.0f, normal.normalized().dot(h_dir)), p);
        
        result_color += (La + Ld + Ls);
    }

    return result_color * 255.f;	
}


Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload, const light::p_Light& p_light, const light::a_Light& a_light, 
                                        const Eigen::Vector3f& eye_pos)
{
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005); // ambient
    Eigen::Vector3f kd = payload.color;                        // diffuse. Read from setColor in drawTriangle()
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937); // specular

    // auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    // auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    // std::vector<light> lights = {l1, l2};
    // Eigen::Vector3f amb_light_intensity{10, 10, 10};
    // Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150; // power factor of the specular term
    auto pl_pos = p_light.position, pl_itn = p_light.intensity;
    auto al_itn = a_light.intensity;
    Eigen::Vector3f color = payload.color;                     
    Eigen::Vector3f point = payload.p_3d;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};
    for (int i = 0; i < pl_pos.size(); i++)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.

        Eigen::Vector3f light_dir = pl_pos[i] - point; // from surface point to light
        Eigen::Vector3f view_dir  = eye_pos - point;        // from surface point to view

        float r_square = light_dir.dot(light_dir); // decay r^2 given the distance

        // note: l, v should be normalized
        light_dir = light_dir.normalized();
        view_dir = view_dir.normalized();

        // note: ka, kd, ks are scalars in the lecture, but they are vectors here, so you should use cwiseProduct
        // ambient La=ka*Ia
        Eigen::Vector3f La = ka.cwiseProduct(al_itn); // cwiseProduct--dot product

        // diffuse Ld=kd(I/r^2)max(0, nl)
        Eigen::Vector3f Ld = kd.cwiseProduct(pl_itn[i] / r_square) * std::max(0.0f, normal.normalized().dot(light_dir));

        // specular Ls=ks(I/r^2)max(0, nh)^p
        Eigen::Vector3f h_dir = (light_dir + view_dir).normalized();
        Eigen::Vector3f Ls = ks.cwiseProduct(pl_itn[i] / r_square) * std::pow(std::max(0.0f, normal.normalized().dot(h_dir)), p);
        
        result_color += (La + Ld + Ls);
    }
    return result_color * 255.f;
}

Eigen::Vector3f depth_fragment_shader(const fragment_shader_payload& payload, const light::p_Light& p_light, const light::a_Light& a_light, 
                                        const Eigen::Vector3f& eye_pos) { // return depth map according to the point's depth
    float max_depth = 1.2;
    Eigen::Vector3f default_color = {255.0, 255.0, 255.0};
    return default_color * (std::abs(payload.pixel_3d.z()) / max_depth); // set 1.2 as max depth
}



Eigen::Vector3f vertex_shader(const vertex_shader_payload& payload) {
    return payload.position;
}





int main(int argc, const char** argv) {

	// define rasterizer class 
    bool command_line = false;									                // set cmd line flag
    bool use_shadow_mapping = true;                                             // if apply shadow mapping to rendering results
	const int screen_width = 700, screen_height = 700;                          
    // Eigen::Vector3f angle = {0.0, 90.0, 0.0};			                        // set rotation (world -> camera)  
    // Eigen::Vector3f scale = {1, 1, 1};			                        // set scale (world -> camera) 
    // Eigen::Vector3f translation = {0.0, 0.0, 0.0};			                    // set translation (world -> camera)
    // Eigen::Vector3f eye_pos = {0,0,10}, gaze = {0, 0, -1}, up = {0,1,0};		// set view position 
    // std::vector<Eigen::Vector3f> point_light_pos = {{20, 20, 20}, {-20, 20, 0}};      // point light position
    // std::vector<Eigen::Vector3f> point_light_itn = {{500, 500, 500}, {500, 500, 500}};// point light itensity

    Eigen::Vector3f angle = {0, 90.0,0};			                        // set rotation (world -> camera)  
    Eigen::Vector3f scale = {1, 1, 1};			                        // set scale (world -> camera) 
    Eigen::Vector3f translation = {0.0, 0.0, 0.0};			                    // set translation (world -> camera)    
    Eigen::Vector3f eye_pos = {0,0,10}, gaze = {0, 0, -1}, up = {0,1,0};		// set view position 
    std::vector<Eigen::Vector3f> point_light_pos = {{0, 5, 5}};      // point light position
    std::vector<Eigen::Vector3f> point_light_itn = {{500, 500, 500}};// point light itensity
    std::vector<Eigen::Vector3f> point_light_gaze, point_light_up;// point light gaze and up direction.

 

    Eigen::Vector3f ambient_light_itn = {10, 10, 10};                           // ambient light intensity
    light::p_Light p_l(point_light_pos, point_light_itn);                       // point light struct
    light::a_Light a_l(ambient_light_itn);                                      // ambient light struct

	std::vector<Triangle*> TriangleList;						                // triangle face container
    std::string filename = "output.png";                                        // output img file name
	objl::Loader Loader;
    Transformation* Tf;


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
     // define a polymorphism function that returns type: Eigen::Vector3f.
    // fragment_shader_payload is the parameters shared by all the member (that would be called)
    // actually, variable active_shader (type Eigen::Vector3f) switches between different instances
    // e.g. phong/texture/normal shader.
    std::function<Eigen::Vector3f(fragment_shader_payload, light::p_Light, light::a_Light, Eigen::Vector3f)> active_shader = phong_fragment_shader;   




    rst::rasterizer r_shadow(screen_width, screen_height);                                        // set shadow screen object and its size 
    r_shadow.read_shadow = false;                                                                  // Generate shadow instead of read shadow
    // set up shadow mapping
    if (use_shadow_mapping) {
        std::cout<<"Apply shadow mapping!"<<std::endl;
        active_shader = depth_fragment_shader;                                                    // set rendering shader type
        r_shadow.clear(rst::Buffers::Color | rst::Buffers::Depth);                                // reset frame/depth buffer
        r_shadow.clear_shadow();                                                                  // reset shadow/shadow_coord buffer
        // first build shadow buffer from light source viewing
        r_shadow.set_model(Tf->get_model_matrix(angle, scale, translation));	                  // world to camera space
        r_shadow.set_projection(Tf->get_projection_matrix(45.0, 1, 0.1, 50));                     // camera canonical to cube         
        r_shadow.set_screen(Tf->get_screen_matrix(screen_width, screen_height));                  // cube to screen
        r_shadow.set_fragment_shader(active_shader);                                        
        for (int i = 0; i < point_light_pos.size(); i++) {
            r_shadow.set_view(Tf->get_view_matrix(point_light_pos[i], gaze, up));				  // light to light canonical            
            r_shadow.drawTriangle(TriangleList, p_l, a_l, point_light_pos[i]);
            cv::Mat image(screen_width, screen_height, CV_32FC3, r_shadow.frame_buffer().data());
            image.convertTo(image, CV_8UC3, 1.0f); 
            cv::imshow("image", image);
            // cv::imwrite(filename, image);
            cv::waitKey(0);   
            cv::destroyAllWindows();                   
        }
        std::cout<<"Finish shadow mapping computation"<<std::endl;
        // return -1;
    }


    std::vector<Eigen::Vector3f> shadow_buf = r_shadow.frame_buffer();                              // obtain shadow buf data
    std::vector<float> shadow_depth_buf = r_shadow.depth_buffer();                                  // obtain shadow depth buf data
    std::vector<Eigen::Matrix4f> shadow_transform = r_shadow.get_transform();
    const Eigen::Matrix4f s_model = shadow_transform[0], s_view = shadow_transform[1], 
    s_projection = shadow_transform[2], s_screen = shadow_transform[3], s_cam2light = shadow_transform[4];      
    const Eigen::Matrix4f& shadow_mvps = s_screen * s_projection * s_view * s_model;                        // obtain shadow-related transform mat (world to light)


    rst::rasterizer r(screen_width, screen_height);                             // set screen buffer object and its size  
    if (use_shadow_mapping)
        r.read_shadow = true;                                           // read shadow-related buffer
    std::string texture_path = "../models/spot/hmap.jpg"; 			// default colormap
    r.set_texture(Texture(texture_path));
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

    if (command_line) { 										                                            // extra command from cmd line
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);                                                 // reset frame/depth buffer 
        r.clear_shadow();                                                                                   // reset shadow/shadow_coord buffer
    	r.set_model(Tf->get_model_matrix(angle, scale, translation));	                                    // world to camera space
        r.set_view(Tf->get_view_matrix(eye_pos, gaze, up));				                                    // camera to camera canonical
        r.set_projection(Tf->get_projection_matrix(45.0, 1, 0.1, 50));                                      // camera canonical to cube 
        r.set_screen(Tf->get_screen_matrix(screen_width, screen_height));                                   // cube to screen
        std::vector<Eigen::Matrix4f> cam_transform = r.get_transform();                                     // get transformation matrix 
        const Eigen::Matrix4f model = cam_transform[0], view = cam_transform[1], 
        projection = cam_transform[2], screen = cam_transform[3], cam2light = cam_transform[4]; 
                                                    //    std::cout<<"working fine!"<<model<<view<<projection<<screen<<std::endl;       
        std::vector<Eigen::Vector3f> new_point_light_pos = p_l.p_trans(p_l.position, view * model);         // update light pos from world to camera space
        // std::cout<<p_l.position[0]<<" "<<new_point_light_pos[0]<<std::endl;
        p_l.position = new_point_light_pos;
        if (use_shadow_mapping) {
            // apply transform from shadow buff to frame buf. Compare and update color.
            r.set_shadow_depth_buffer(shadow_depth_buf);                                                    // set precomputed shadow buffer     
            const Eigen::Matrix4f& mvps_inv = (screen * projection * view * model).inverse();             // obtain shadow-related transform mat (world to light)
            r.set_cam2light(shadow_mvps * mvps_inv);                                                        // set transformation matrix (cam to light) = * world to light * came to world 
        }

        r.drawTriangle(TriangleList, p_l, a_l, eye_pos);						                            // draw every face into screen
        cv::Mat image(screen_width, screen_height, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imwrite(filename, image);  
        return 0;     
    }

    int key = 0;
    int save_img_idx = 1;
    while(key != 27) {

    	r.clear(rst::Buffers::Color | rst::Buffers::Depth);                                                 // reset buffer
    	r.set_model(Tf->get_model_matrix(angle, scale, translation));	                                    // world to camera space
        r.set_view(Tf->get_view_matrix(eye_pos, gaze, up));				                                    // camera to camera canonical
        r.set_projection(Tf->get_projection_matrix(45.0, 1, 0.1, 50));                                      // camera canonical to cube 
        r.set_screen(Tf->get_screen_matrix(screen_width, screen_height));                                   // cube to screen
        r.drawTriangle(TriangleList, p_l, a_l, eye_pos);						                            // draw every face into screen
        cv::Mat image(screen_width, screen_height, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imshow("image", image);
        // cv::imwrite(filename, image);
        key = cv::waitKey(10);
        std::cout<<"Current angle: "<<angle<<std::endl;
        if (key == 'a' )     {
            angle.y() -= 10;
        }
        else if (key == 'd') {
            angle.y() += 10;
        }
        else if (key == 's') {
            std::string new_fname = filename.substr(0, filename.size()-4) + std::to_string(save_img_idx) + ".png";
            cv::imwrite(new_fname, image);
        }

    }   

	return 0;
}