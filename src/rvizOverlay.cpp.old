#include <rvizOverlay.h>

using namespace vr; 

//VROverlayHandle_t overlayHandle;

void imageCb(const sensor_msgs::ImageConstPtr& msg, ImageConverter* icPtr);

vr::HmdMatrix34_t transform = {								//overlay position
	1.0f, 0.0f, 0.0f, 0.12f,								//+: upper axis
	0.0f, 1.0f, 0.0f, 0.08f,								//+: right horizontal axis
	0.0f, 0.0f, 1.0f, -0.3f									//-: forward axis
};


static const std::string OPENCV_WINDOW = "Image window";


ImageConverter::ImageConverter(int arc, char* arv[])
{
    this->argc_arg = arc;
    this->argv_arg = arv;
}

ImageConverter::~ImageConverter()
{
    cv::destroyWindow(OPENCV_WINDOW);
}

void ImageConverter::init()
{

    //cv::namedWindow(OPENCV_WINDOW);
    std::string sExecutableDirectory = Path_StripFilename(Path_GetExecutablePath());
    std::string strFullPath = Path_MakeAbsolute("C:/Users/Dyros/Desktop/avatar/src/VR/src/panorama.png", sExecutableDirectory);
    std::cout << ros::ok() << std::endl;
    cv::Mat imageDefault = cv::imread(strFullPath, 1);
    //cv::imshow("hihi", imageDefault);
    cv::waitKey(1);
    cv::namedWindow("cvwindow");

    glPixelStorei (GL_UNPACK_ALIGNMENT, 1);
    glGenTextures(1, &rvizTex);
    glBindTexture(GL_TEXTURE_2D, rvizTex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, imageDefault.cols, imageDefault.rows,
            0, GL_RGBA, GL_UNSIGNED_BYTE, &imageDefault.data[0]);
    //glGenerateMipmap(GL_TEXTURE_2D);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

    glBindTexture(GL_TEXTURE_2D, 0);

/*
    //cv::imshow("hi", imageDefault);
    cv::namedWindow("ogl", cv::WINDOW_OPENGL);
    cv::ogl::Texture2D cvtex(imageDefault);
    //cv::UMat fts;
    //cv::ogl::convertFromGLTexture2D(rvizTex, fts);
    cv::imshow("ogl", cvtex);
    cv::waitKey(1);
*/
    VROverlay()->CreateOverlay ("image", "rvizView", &overlayHandle);
}

void ImageConverter::rosTasks()
{
    //ros::Rate r(1);
    /*while(ros::ok()){
        r.sleep();
        std::cout << "1" << std::endl;
        ros::spinOnce();
        //std::cout << "2" << std::endl;
    }*/
}


void ImageConverter::runMainLoop()
{

    
    ros::init(this->argc_arg, this->argv_arg, "rvizOverlay");
    ros::NodeHandle nh_;
//    std::cout << ros::ok() << std::endl;
    std::cout << "55" << std::endl;
    image_transport::ImageTransport it_(nh_);
    image_transport::Subscriber image_sub_;
    nothing_pub = nh_.advertise<std_msgs::Bool>("nothing", 1000);
    
    // Subscribe input image
    image_sub_ = it_.subscribe("/rviz1/camera1/image", 1,
                            boost::bind(imageCb, _1, this));



//    std::cout << "9" << std::endl;
    //std::thread t (&ImageConverter::rosTasks, this);
//    std::cout << "10" << std::endl;
    ros::Rate r(2);
//    std::cout << "11" << std::endl;
    std::cout << ros::ok() << std::endl;
    while(ros::ok())
    {
        //std::cout << "13" << std::endl;
    //    std::cout << "2" << std::endl;
        ros::spinOnce();
        //std::cout << "12" << std::endl;
        //renderOverlay();
        //std::cout << "3" << std::endl;
    }
    //t.join();
}

void imageCb(const sensor_msgs::ImageConstPtr& msg, ImageConverter* icPtr)
{
    
    //std::lock_guard<std::mutex> guard(icPtr->render_mutex);
    //std::cout << "4" << std::endl;
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
    cv::imshow("cvwindow", cv_ptr->image);
    cv::waitKey(1);
    icPtr->img = cv_ptr->image.clone();
        
    //cv::imshow(OPENCV_WINDOW, icPtr->img);
    //std::cout << "5" << std::endl;
    cv::waitKey(1);
    icPtr->renderOverlay();
}

void ImageConverter::renderOverlay()
{
    if(img.cols<=0 || img.rows<=0){
        std::cout << "img not loaded" << std::endl;
        return;
    }
    cv::imshow(OPENCV_WINDOW, img);
    render_mutex.lock();

    
    std::cout << "start" << std::endl;
    //std::cout << "6" << std::endl;
    
    // Update GUI Window  

    glBindTexture(GL_TEXTURE_2D, rvizTex);

    //glPixelStorei(GL_PACK_ALIGNMENT, (img.step & 3) ? 1 : 4);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glPixelStorei(GL_PACK_ROW_LENGTH, img.step/img.elemSize());
    glTexImage2D(GL_TEXTURE_2D, 0, 3,  img.cols, img.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, &img.data[0]);


//    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB_INTEGER, img.cols, img.rows, 0, GL_BGR_INTEGER, GL_UNSIGNED_BYTE, &img.data[0]);
    
    //glGenerateMipmap(GL_TEXTURE_2D);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    
    glBindTexture(GL_TEXTURE_2D, 0);

    /*
    cv::namedWindow("ogl", cv::WINDOW_OPENGL);
    std::cout << "here" << std::endl;
    cv::ogl::Texture2D cvtex(img);
    static cv::UMat fts;
    //cv::ogl::convertFromGLTexture2D(cvtex, fts);
    std::cout << "there" << std::endl;
    cv::imshow("ogl", rvizTex);
    std::cout << "where" << std::endl;
    cv::waitKey(1);
    */
    //cv::imwrite( "C:/Users/Dyros/Desktop/avatar/src/VR/src/rvizView.png",  cv_ptr->image );
    
    
    std::cout << "mid1" << std::endl;

    
    std::cout << "mid2" << std::endl;
    
    std::cout << "tt  " << testTex << std::endl;
    
    if(testTex==0){
        std::cout << "not loaded" << std::endl;
    }
    
    std::cout << "tt2 " << testTex << std::endl;
    
    //cv::ogl::Texture2D cvTex(img);

/*
    tex_vr.handle = reinterpret_cast<void*>(static_cast<uintptr_t>(cvTex));
    tex_vr.eType = vr::ETextureType::TextureType_OpenGL;
    tex_vr.eColorSpace = vr::EColorSpace::ColorSpace_Auto;
*/

    //tex_vr = {(void*)(uintptr_t)testTex, vr::TextureType_OpenGL, vr::ColorSpace_Auto};
    
	//ros::Duration(3).sleep();
    
    //VROverlay()->SetOverlayTexture(overlayHandle, &tex_vr);
    //VROverlay()->SetOverlayFromFile(overlayHandle, "C:/Users/Dyros/Desktop/avatar/src/VR/src/rvizView.png");      
    //VROverlay()->SetOverlayAlpha(overlayHandle, 0.5);						//opacity
	// VROverlay()->SetOverlayWidthInMeters(overlayHandle, 0.2f);				//overlay size
	// VROverlay()->SetOverlayTransformTrackedDeviceRelative(overlayHandle, vr::k_unTrackedDeviceIndex_Hmd, &transform);
	// VROverlay()->ShowOverlay(overlayHandle);

    
    std::cout << "end" << std::endl;
    //cv::waitKey(1000);
    //ros::Duration(1).sleep();
    //VROverlay()->ClearOverlayTexture(overlayHandle);
    //std::cout << "7" << std::endl;
    
    render_mutex.unlock();
    
}