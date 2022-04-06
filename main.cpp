#include <iostream>
#include <opencv2/opencv.hpp>
#include "rasterizer.hpp"
#include "OBJ_Loader.h"

int main(int argc, const char** argv) {

	std::vector<Triangle*> TriangleList;
	 objl::Loader Loader;

	 bool loadout = Loader.LoadFile("../models/spot/spot_triangulated_good.obj");
	 for(auto mesh : Loader.LoadedMeshes) {
	 	for (int i = 0; i < mesh.Vertices.size(); i += 3) {
	 		Triangle* = new Triangle();
	 		for (int j = 0; j < 3; j++) {
	 			t->setVertex();
	 			t->setNormal();
	 			t->setTexCoord();
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
    	
    }	

	return 0;
}