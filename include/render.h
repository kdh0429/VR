

#define SDL_MAIN_HANDLED
#include <GL/glew.h>
#include <GL/glut.h>
#include <openvr.h>
#include <SDL.h>
#include <SDL_opengl.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <Eigen/Dense>
#include <boost/bind.hpp>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt8MultiArray.h"
#include "VR/matrix_3_4.h"
#include "ricohStream.h"
#include <time.h>
#include <iostream>
#include "Matrices.h"
#include "lodepng.h"
#include "pathtools.h"

#include <glm/glm.hpp>
#define _USE_MATH_DEFINES
#include <math.h>
#include <cstdlib>
#include <stdio.h>

#if defined(POSIX)
#include "unistd.h"
#endif

#if !defined(M_PI)
#    define M_PI 3.1415926535897f
#endif 

#if !defined(M_HALF_PI)
#    define M_HALF_PI M_PI / 2.f
#endif 

#ifndef render_h
#define render_h

class RenderModel {

public:
	RenderModel(const std::string& sRenderModelName);
	~RenderModel();

	bool BInit(const vr::RenderModel_t & vrModel, const vr::RenderModel_TextureMap_t& vrDiffuseTexture);
	void Cleanup();
	void Draw();
	const std::string& GetName() const { return m_sModelName; }

private:
	GLuint m_glVertBuffer;
	GLuint m_glIndexBuffer;
	GLuint m_glVertArray;
	GLuint m_glTexture;
	GLsizei m_unVertexCount;
	std::string m_sModelName;

};


#endif 