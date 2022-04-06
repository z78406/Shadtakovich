#include <iostream>
#include <opencv2/opencv.hpp>
#include "rasterizer.hpp"
#include "OBJ_Loader.h"

int main(int argc, const char** argv) {

	std::vector<Triangle*> TriangleList;
	 objl::Loader Loader;

	 bool loadout = Loader.LoadFile("../models/spot/spot_triangulated_good.obj");
	 for(auto mesh : Loader.LoadedMeshes) {
	 	for (int i = 0; i < mesh.Vertices.size(); i += 3) { // each face(triangel)
	 		Triangle* = new Triangle();
	 		for (int j = 0; j < 3; j++) { // each vertex
                t->setVertex(j,Vector4f(mesh.Vertices[i+j].Position.X,mesh.Vertices[i+j].Position.Y,mesh.Vertices[i+j].Position.Z,1.0));
                t->setNormal(j,Vector3f(mesh.Vertices[i+j].Normal.X,mesh.Vertices[i+j].Normal.Y,mesh.Vertices[i+j].Normal.Z));
                t->setTexCoord(j,Vector2f(mesh.Vertices[i+j].TextureCoordinate.X, mesh.Vertices[i+j].TextureCoordinate.Y));
	 		}
	 		TriangleList.push_back(t);
	 	}
	}

	rst::rasterizer r(700, 700); // screen object and its size
    string texture_path = "../models/spot/hmap.jpg";
    r.set_texture(Texture(texture_path));
    // define a polymorphism function that returns type: Eigen::Vector3f.
    // fragment_shader_payload is the parameters shared by all the member (that would be called)
    // actually, variable active_shader (type Eigen::Vector3f) switches between different instances
    // e.g. phong/texture/normal shader.
	std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = phong_fragment_shader;

    if (argc >= 2) {
        command_line = true;
        filename = std::string(argv[1]);
        if (argc == 3 && std::string(argv[2]) == "texture")
        {
            std::cout << "Rasterizing using the texture shader\n";
            active_shader = texture_fragment_shader;
            texture_path = "../models/spot/spot_texture.png";
            r.set_texture(Texture(texture_path));
        }            	
    }	

	return 0;
}