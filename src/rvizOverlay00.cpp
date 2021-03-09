#include <openvr.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>
#include "hmdHandler.h"

#include <QOpenGLFramebufferObjectFormat>


using namespace vr;

vr::HmdMatrix34_t transform = {								//overlay position
	1.0f, 0.0f, 0.0f, 0.12f,								//+: upper axis
	0.0f, 1.0f, 0.0f, 0.08f,								//+: right horizontal axis
	0.0f, 0.0f, 1.0f, -0.3f									//-: forward axis
};

VROverlayHandle_t overlayHandle;
GLuint ovtexture;
vr::Texture_t tex_vr = {};
vr::EVRInitError error;

//void check_error(int line, EVRInitError error) { if (error != 0) printf("%d: error %s\n", line, VR_GetVRInitErrorAsSymbol(error)); }

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    glGenTextures(1, &ovtexture);
    image_sub_ = it_.subscribe("/rviz1/camera1/image", 10, 
      &ImageConverter::imageCb, this);
    
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    cv::Mat img = cv_ptr->image.clone();
    std::cout << "3" << std::endl;

    glBindTexture(GL_TEXTURE_2D, ovtexture); //good
    std::cout << "4" << std::endl;

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    std::cout << "5" << std::endl;
    // set texture filtering parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    std::cout << "6" << std::endl;
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img.cols, img.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, img.data);
    std::cout << "7" << std::endl;
    //glGenerateMipmap(GL_TEXTURE_2D);
    std::cout << "8" << std::endl;
    
    tex_vr = {(void*)(uintptr_t)ovtexture, vr::TextureType_OpenGL, vr::ColorSpace_Auto };
    

    //VROverlay()->SetOverlayTexture(overlayHandle, &tex_vr);
    //check_error(__LINE__, error);
    
  }
};

int main(int argc, char** argv)
{
  VR_Init(&error, vr::VRApplication_Overlay);
  VROverlay()->CreateOverlay ("image", "rvizView", &overlayHandle); /* key has to be unique, name doesn't matter */
    


  VROverlay()->SetOverlayFromFile(overlayHandle, "C:/Users/Dyros/Desktop/avatar/src/VR/src/rvizView.png");
  //ROverlay()->SetOverlayTexture(overlayHandle, &tex_vr);

  VROverlay()->SetOverlayAlpha(overlayHandle, 0.8);						//opacity
	VROverlay()->SetOverlayWidthInMeters(overlayHandle, 0.3f);				//overlay size

  VROverlay()->SetOverlayTransformTrackedDeviceRelative(overlayHandle, vr::k_unTrackedDeviceIndex_Hmd, &transform);
	  
  VROverlay()->ShowOverlay(overlayHandle);
    
  ros::init(argc, argv, "image_converter");
  // ImageConverter ic;
  ros::spin();
  return 0;
}