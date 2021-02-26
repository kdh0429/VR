#include "render.h"
#include "std_msgs/Bool.h"
#include <mutex>
#include <thread>
#include <vector>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/opengl.hpp>
#include <sstream> 
#include <windows.h>

#ifndef rvizOverlay_h
#define rvizOverlay_h




class ImageConverter {

public:
	ImageConverter(int arc, char* arv[]);
	~ImageConverter();
	void init();
	void runMainLoop();
	void rosTasks();

	void renderOverlay();
	int argc_arg;
	char** argv_arg;


	std::mutex render_mutex;

	cv::Mat img;

	vr::VROverlayHandle_t overlayHandle;
	vr::Texture_t tex_vr;

	GLuint rvizTex;
    GLuint testTex;


	ros::Publisher nothing_pub;
	
};

#endif