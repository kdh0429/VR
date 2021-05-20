#include "hmdHandler.h"



void ThreadSleep(unsigned long nMilliseconds)
{
#if defined(_WIN32)
    ::Sleep(nMilliseconds);
#elif defined(POSIX)
    usleep(nMilliseconds * 1000);
#endif
}


/* HMD Constructor IMPLEMENTATION*/
HMD::HMD(int arc, char* arv[])
    : m_pCompanionWindow(NULL)
    , m_pContext(NULL)
    , m_nCompanionWindowWidth(640)
    , m_nCompanionWindowHeight(320)
    , m_unSceneProgramID(0)
    , m_unCompanionWindowProgramID(0)
    , m_unControllerTransformProgramID(0)
    , m_unRenderModelProgramID(0)
    , m_bDebugOpenGL(false)
    , m_bVerbose(false)
    , m_bPerf(false)
    , m_bVblank(false)
    , m_bGlFinishHack(true)
    , m_glControllerVertBuffer(0)
    , m_unControllerVAO(0)

    , m_nSceneMatrixLocation(-1)
    , m_nControllerMatrixLocation(-1)
    , m_nRenderModelMatrixLocation(-1)
    , m_iTrackedControllerCount(0)
    , m_iTrackedControllerCount_Last(-1)
    , m_iValidPoseCount(0)
    , m_iValidPoseCount_Last(-1)
    , m_iSceneVolumeInit(1)
    , m_strPoseClasses("")
    , m_bShowCubes(true)

{
    this->argc_arg = arc;
    this->argv_arg = arv;
    checkControllers = false;
    checkTrackers = true;
    pubPose = true;
    memset(m_rDevClassChar, 0, sizeof(m_rDevClassChar));

}

/* HMD Destructor IMPLEMENTATION*/
HMD::~HMD() {
    // memory release is done in Shutdown 
}

std::string GetTrackedDeviceString(vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError* peError = NULL) {
    uint32_t unRequiredBufferLen = vr::VRSystem()->GetStringTrackedDeviceProperty(unDevice, prop, NULL, 0, peError);
    if (unRequiredBufferLen == 0) return "";
    
    char* pchBuffer = new char[unRequiredBufferLen];
    unRequiredBufferLen = vr::VRSystem()->GetStringTrackedDeviceProperty(unDevice, prop, pchBuffer, unRequiredBufferLen, peError);
    std::string sResult = pchBuffer;
    delete[] pchBuffer;
    return sResult;

}


//////////////left//////////////////

void createCubeMapFace_leftBack(HMD* hmdPtr){
    hmdPtr->createCubeMapFace(hmdPtr->leftCvEquirect, hmdPtr->LeftcubeBack, CubeFaceName::Back, 0);
    cv::flip(hmdPtr->LeftcubeBack, hmdPtr->LeftcubeBack, 1);
    cv::cvtColor(hmdPtr->LeftcubeBack, hmdPtr->LeftcubeBack, CV_BGR2RGBA);
    
}


void createCubeMapFace_leftRight(HMD* hmdPtr){
    hmdPtr->createCubeMapFace(hmdPtr->leftCvEquirect, hmdPtr->LeftcubeRight, CubeFaceName::Right, 0);
    cv::flip(hmdPtr->LeftcubeRight, hmdPtr->LeftcubeRight, 1);
    cv::cvtColor(hmdPtr->LeftcubeRight, hmdPtr->LeftcubeRight, CV_BGR2RGBA);
    
}

void createCubeMapFace_leftLeft(HMD* hmdPtr){
    hmdPtr->createCubeMapFace(hmdPtr->leftCvEquirect, hmdPtr->LeftcubeLeft, CubeFaceName::Left, 0);
    cv::flip(hmdPtr->LeftcubeLeft, hmdPtr->LeftcubeLeft, 1);
    cv::cvtColor(hmdPtr->LeftcubeLeft, hmdPtr->LeftcubeLeft, CV_BGR2RGBA);
    
}

void createCubeMapFace_leftBottom(HMD* hmdPtr){
    hmdPtr->createCubeMapFace(hmdPtr->leftCvEquirect, hmdPtr->LeftcubeBottom, CubeFaceName::Bottom, 0);
    cv::flip(hmdPtr->LeftcubeBottom, hmdPtr->LeftcubeBottom, 0);
    cv::rotate(hmdPtr->LeftcubeBottom, hmdPtr->LeftcubeBottom, cv::ROTATE_90_COUNTERCLOCKWISE);

    cv::cvtColor(hmdPtr->LeftcubeBottom, hmdPtr->LeftcubeBottom, CV_BGR2RGBA);
    
}

////////////right//////////////


void createCubeMapFace_rightBack(HMD* hmdPtr){
    hmdPtr->createCubeMapFace(hmdPtr->rightCvEquirect, hmdPtr->RightcubeBack, CubeFaceName::Back, 1);
    cv::flip(hmdPtr->RightcubeBack, hmdPtr->RightcubeBack, 1);
    cv::cvtColor(hmdPtr->RightcubeBack, hmdPtr->RightcubeBack, CV_BGR2RGBA);
    
}


void createCubeMapFace_rightRight(HMD* hmdPtr){
    hmdPtr->createCubeMapFace(hmdPtr->rightCvEquirect, hmdPtr->RightcubeRight, CubeFaceName::Right, 1);
    cv::flip(hmdPtr->RightcubeRight, hmdPtr->RightcubeRight, 1);
    cv::cvtColor(hmdPtr->RightcubeRight, hmdPtr->RightcubeRight, CV_BGR2RGBA);
    
}

void createCubeMapFace_rightLeft(HMD* hmdPtr){
    hmdPtr->createCubeMapFace(hmdPtr->rightCvEquirect, hmdPtr->RightcubeLeft, CubeFaceName::Left, 1);
    cv::flip(hmdPtr->RightcubeLeft, hmdPtr->RightcubeLeft, 1);
    cv::cvtColor(hmdPtr->RightcubeLeft, hmdPtr->RightcubeLeft, CV_BGR2RGBA);
    
}

void createCubeMapFace_rightBottom(HMD* hmdPtr){
    hmdPtr->createCubeMapFace(hmdPtr->rightCvEquirect, hmdPtr->RightcubeBottom, CubeFaceName::Bottom, 1);
    cv::flip(hmdPtr->RightcubeBottom, hmdPtr->RightcubeBottom, 0);
    cv::rotate(hmdPtr->RightcubeBottom, hmdPtr->RightcubeBottom, cv::ROTATE_90_COUNTERCLOCKWISE);

    cv::cvtColor(hmdPtr->RightcubeBottom, hmdPtr->RightcubeBottom, CV_BGR2RGBA);
    
}



void createCubeMapFace_left_thread(HMD* hmdPtr, ricohRos* streamPtr)
{
    sws_scale(streamPtr->ricohContext, streamPtr->ricohleftFrame->data,
        streamPtr->ricohleftFrame->linesize, 0,
        hmdPtr->height,
        (hmdPtr->leftDat).data, (hmdPtr->leftDat).linesize);
   
    //std::thread lt1(createCubeMapFace_leftFront,hmdPtr);
    std::thread lt2(createCubeMapFace_leftBack,hmdPtr);
    //std::thread lt3(createCubeMapFace_leftTop,hmdPtr);
    std::thread lt4(createCubeMapFace_leftRight,hmdPtr);
    std::thread lt5(createCubeMapFace_leftLeft,hmdPtr);
    std::thread lt6(createCubeMapFace_leftBottom,hmdPtr);

    //lt1.join();
    lt2.join();
    //lt3.join();
    lt4.join();
    lt5.join();
    lt6.join();


}


void createCubeMapFace_right_thread(HMD* hmdPtr, ricohRos* streamPtr)
{
    sws_scale(streamPtr->ricohContext, streamPtr->ricohrightFrame->data,
        streamPtr->ricohrightFrame->linesize, 0,
        hmdPtr->height,
        (hmdPtr->rightDat).data, (hmdPtr->rightDat).linesize);
    //std::thread rt1(createCubeMapFace_rightFront,hmdPtr);
    std::thread rt2(createCubeMapFace_rightBack,hmdPtr);
    //std::thread rt3(createCubeMapFace_rightTop,hmdPtr);
    std::thread rt4(createCubeMapFace_rightRight,hmdPtr);
    std::thread rt5(createCubeMapFace_rightLeft,hmdPtr);
    std::thread rt6(createCubeMapFace_rightBottom,hmdPtr);

    //rt1.join();
    rt2.join();
    //rt3.join();
    rt4.join();
    rt5.join();
    rt6.join();



}

void HMD::hmd_para_callback(const std_msgs::Float32MultiArray data)
{
    // if ((data.data[0]) != depth){
    //     depth = data.data[0];
    //     m_fScaleSpacing = depth;
    //     //CreateAllShaders();
    //     SetupTexturemaps();
    //     SetupScene();
    //     SetupCameras();
    //     SetupStereoRenderTargets();
    //     SetupCompanionWindow();
    //     if (!vr::VRCompositor()) {
    //     std::cout << "Compositor initialization failed" << std::endl; // it doesn't work
    //     return;
    // }
    // }
    eye_angle = data.data[0] - 0.5;
    eye_angle_cali = eye_angle * 10 * M_PI / 180 ;// -5 to 5 degree
    distance = data.data[1]; // 0~1
    distance_cali = -0.15 +distance * 0.3; // you can change the disteance between eyes with distance_cali. MAX : 0.15, MIN : -0.15 // Best Condition : distance_cali == 0
    
    m_mat4eyePosLeft = GetHMDMatrixPoseEye(vr::Eye_Left, distance_cali, eye_angle_cali);
    m_mat4eyePosRight =GetHMDMatrixPoseEye(vr::Eye_Right, distance_cali , eye_angle_cali); 

}

void decode_thread_l(ricohRos* streamPtr,HMD* hmdPtr,int gotFrame)
{
    int left_len = avcodec_decode_video2(streamPtr->c_l, streamPtr->ricohleftFrame, &gotFrame, &(streamPtr->ricohPacket_l));
    if (left_len < 0 || !gotFrame) {
        std::cout << "Could not Decode left ricoh H264 stream" << std::endl;
        hmdPtr->is_stream_process_finished_l = true;
        return; 
    }
}
void decode_thread_r(ricohRos* streamPtr,HMD* hmdPtr,int gotFrame)
{
    int right_len = avcodec_decode_video2(streamPtr->c_r, streamPtr->ricohrightFrame, &gotFrame, &(streamPtr->ricohPacket_r));
    if (right_len < 0 || !gotFrame) {
        std::cout << "Could not Decode right ricoh H264 stream" << std::endl;
        hmdPtr->is_stream_process_finished_r = true;
        return;
    }
}


void streamCallbackBackground(ricohRos* streamPtr, HMD* hmdPtr)
{

    // viz_stream_mutex.
    // hmdPtr->loop_tick_++;
    // if(hmdPtr->loop_tick_%10 != 0){
    //     return;
    // }
    std::lock_guard<std::mutex> guard(hmdPtr->render_mutex);
    const auto streamPacket = hmdPtr->stored_stream_packet;

    
    clock_t start, end,s,e;
    start = clock();
    s=clock();
    //::cout << streamPacket->layout.dim[4].label << std::endl;


    //std::cout << "test" << std::endl;
    // cout << streamPtr->ricohPacket.size << endl;
    //memcpy(streamPtr->ricohPacket.data, tmp, streamPtr->ricohPacket.size);

    
    int gotFrame_l = 0, gotFrame_r = 0;
    
    // std::scoped_lock<std::mutex> _(hmdPtr->render_mutex);
    // hmdPtr->render_mutex.lock
    if (!(hmdPtr->streamParamInit)) {
        std::cout << "First streaminng input is recognized.. streaming parameters starts to be initialized" << std::endl;
        hmdPtr->width = streamPacket->layout.dim[1].size;
        hmdPtr->height = streamPacket->layout.dim[0].size;
        hmdPtr->leftCvEquirect = cv::Mat(hmdPtr->height, hmdPtr->width, CV_8UC3);
        hmdPtr->leftDat.data[0] = (uint8_t*)(hmdPtr->leftCvEquirect).data;
        hmdPtr->rightCvEquirect = cv::Mat(hmdPtr->height, hmdPtr->width, CV_8UC3);
        hmdPtr->rightDat.data[0] = (uint8_t*)(hmdPtr->rightCvEquirect).data;


        avpicture_fill((AVPicture*)&(hmdPtr->leftDat), (hmdPtr->leftDat).data[0], AV_PIX_FMT_BGR24, hmdPtr->width, hmdPtr->height);
        avpicture_fill((AVPicture*)&(hmdPtr->rightDat), (hmdPtr->rightDat).data[0], AV_PIX_FMT_BGR24, hmdPtr->width, hmdPtr->height);

        hmdPtr->streamParamInit = true;
    }

    streamPtr->ricohPacket_l.data = (uint8_t*)&(streamPacket->data[0]);
    streamPtr->ricohPacket_l.size = streamPacket->layout.dim[3].size;
    
    // int left_len = avcodec_decode_video2(streamPtr->c, streamPtr->ricohleftFrame, &gotFrame, &(streamPtr->ricohPacket_l));
    // if (left_len < 0 || !gotFrame) {
    //     std::cout << "Could not Decode left ricoh H264 stream" << std::endl;
    //     hmdPtr->is_stream_process_finished_l = true;
    //     return;
    // }

    //gotFrame = 0;
    // start = clock();
    streamPtr->ricohPacket_r.size = streamPacket->layout.dim[4].size;
    streamPtr->ricohPacket_r.data = (uint8_t*)(&(streamPacket->data[0]) + streamPacket->layout.dim[3].size);
    std::thread t1(decode_thread_l,streamPtr, hmdPtr,gotFrame_l);
    std::thread t2(decode_thread_r,streamPtr, hmdPtr,gotFrame_r);
    t1.join();
    t2.join();
    
    end = clock();
    double time = (double)(end - start)/ CLOCKS_PER_SEC;
    std::cout << "time(decode) : " << time << std::endl;
    start = clock();

    // int right_len = avcodec_decode_video2(streamPtr->c, streamPtr->ricohrightFrame, &gotFrame, &(streamPtr->ricohPacket_r));
    // if (right_len < 0 || !gotFrame) {
    //     std::cout << "Could not Decode right ricoh H264 stream" << std::endl;
    //     hmdPtr->is_stream_process_finished_r = true;
    //     return;
    // }

    // std::thread decode_thread_l(decode_thread_func_l,streamPtr,hmdPtr,streamPacket,gotFrame_l,left_len);
    // std::thread decode_thread_r(decode_thread_func_r,streamPtr,hmdPtr,streamPacket,gotFrame_r,right_len);

    // decode_thread_l.join();
    // decode_thread_r.join();

    // int gotFrame = gotFrame_l * gotFrame_r;
    
    // stream_packet_mutex.unlock();
    
    
    // 여기 까지 2개의 avcodec_decode_video2 함수를 거치며 0.03초 
    //cout << (double)(end - start) / CLOCKS_PER_SEC << endl;


    hmdPtr->leftCvEquirect = cv::Mat(hmdPtr->height, hmdPtr->width, CV_8UC3);
    (hmdPtr->leftDat).data[0] = (uint8_t*)(hmdPtr->leftCvEquirect).data;

    hmdPtr->rightCvEquirect = cv::Mat(hmdPtr->height, hmdPtr->width, CV_8UC3);
    (hmdPtr->rightDat).data[0] = (uint8_t*)(hmdPtr->rightCvEquirect).data;
    
    //avpicture_fill((AVPicture*)&dst, dst.data[0], AV_PIX_FMT_BGR24, hmdPtr->width, hmdPtr->height);



    // if (left_len < 0 || !gotFrame) {
    //     std::cout << "Could not Decode ricoh H264 stream" << std::endl;
    //     hmdPtr->is_stream_process_finished = true;
    //     return;
    // }

    hmdPtr->left_Pixfmt = (enum AVPixelFormat)streamPtr->ricohleftFrame->format;
    hmdPtr->right_Pixfmt = (enum AVPixelFormat)streamPtr->ricohrightFrame->format;

    if (hmdPtr->left_Pixfmt != hmdPtr->right_Pixfmt) {
        std::cout << "Left, Right Pixel Format are different. Please adjust format" << std::endl;
        hmdPtr->is_stream_process_finished = true;
        return;
    }

    end = clock();
    time = (double)(end - start)/ CLOCKS_PER_SEC;
    std::cout << "time(before context) : " << time << std::endl;
    start = clock();

    streamPtr->ricohContext = sws_getContext(hmdPtr->width, hmdPtr->height, hmdPtr->left_Pixfmt, hmdPtr->width, hmdPtr->height, AV_PIX_FMT_BGR24,
        SWS_FAST_BILINEAR, NULL, NULL, NULL);

    end = clock();
    time = (double)(end - start)/ CLOCKS_PER_SEC;
    std::cout << "time(after context) : " << time << std::endl;
    start = clock();

    if (streamPtr->ricohContext == NULL) {
        fprintf(stderr, "Cannot initialize context\n");
        std::cout << "Cannot initialize context" << std::endl;
        hmdPtr->is_stream_process_finished = true;
        return;
    }





    bool LeftconversionSuccess = true;
    bool RightconversionSuccess = true;


    // start = clock();
    end = clock();
    time = (double)(end - start)/ CLOCKS_PER_SEC;
    std::cout << "time(before cubemap) : " << time << std::endl;
    start = clock();

    std::thread left_cube(createCubeMapFace_left_thread,hmdPtr,streamPtr);
    std::thread right_cube(createCubeMapFace_right_thread,hmdPtr,streamPtr);
    // std::cout << "eqwidth : " << hmdPtr->leftCvEquirect.cols << std::endl;
    // std::cout << "eqheight : " << hmdPtr->leftCvEquirect.rows << std::endl;
    // std::cout << "cubewidth : " << hmdPtr->LeftcubeBack.cols << std::endl;
    // std::cout << "cubeheight : " << hmdPtr->LeftcubeBack.rows << std::endl;
    // createcubemapface 0.016초 소요
    //cube_threads.clear();
    
    left_cube.join();
    right_cube.join();

   
    
    
    
    //cout << (double)(end - start) / CLOCKS_PER_SEC << endl;
    
    //cv::flip등의 처리 0.007초 소요//



//// left

//////


    //cv::resize(hmdPtr->leftCvEquirect, hmdPtr->leftCvEquirect, cv::Size(640, 480), 0, 0, CV_INTER_NN);
    //cv::resize(hmdPtr->rightCvEquirect, hmdPtr->rightCvEquirect, cv::Size(640, 480), 0, 0, CV_INTER_NN);
    //cv::imshow("left equirect", hmdPtr->leftCvEquirect);
    //cv::imshow("right equirect", hmdPtr->rightCvEquirect);


   
   /* cv::imshow("LEFT back face", hmdPtr->LeftcubeBack);
    cv::imshow("LEFT left face", hmdPtr->LeftcubeLeft);
    cv::imshow("LEFT Right face", hmdPtr->LeftcubeRight);
    cv::imshow("LEFT Top face", hmdPtr->LeftcubeTop);
    cv::imshow("LEFT Bottom face", hmdPtr->LeftcubeBottom);

    cv::imshow("RIGHT back face", hmdPtr->RightcubeBack);
    cv::imshow("RIGHT left face", hmdPtr->RightcubeLeft);
    cv::imshow("RIGHT Right face", hmdPtr->RightcubeRight);
    cv::imshow("RIGHT Top face", hmdPtr->RightcubeTop);
    cv::imshow("RIGHT Bottom face", hmdPtr->RightcubeBottom);*/
    //cv::imshow("rviz", hmdPtr->RvizScreen);
    end = clock();
    e= clock();
    time = (double)(end - start)/ CLOCKS_PER_SEC;
    std::cout << "time : " << time << std::endl;
    std::cout << "fps : " << (double)1/(e-s)*CLOCKS_PER_SEC << std::endl;
    // std::cout << "width : " << hmdPtr->m_nRenderWidth << std::endl;
    // std::cout << "height : " << hmdPtr->m_nRenderHeight << std::endl;
    //ROS_INFO("%f",1/(end - start));
    cv::waitKey(1);
    hmdPtr->first_data = false;
    hmdPtr->is_stream_process_finished = true;
    av_free_packet(&(streamPtr->ricohPacket_l));
    av_free_packet(&(streamPtr->ricohPacket_l));

}


void streamCallback(const std_msgs::UInt8MultiArray::ConstPtr& streamPacket, ricohRos* streamPtr, HMD* hmdPtr) 
{
    if (hmdPtr->is_stream_process_finished && hmdPtr->is_stream_process_finished_l && hmdPtr->is_stream_process_finished_r )
    {
        if (hmdPtr->process_stream_thread.joinable())
        {
            hmdPtr->process_stream_thread.join();
        }
        hmdPtr->is_stream_process_finished = false;
        hmdPtr->stored_stream_packet = std::make_shared<std_msgs::UInt8MultiArray>(*streamPacket);
        // streamCallbackBackground( streamPtr, hmdPtr);
        hmdPtr->process_stream_thread = std::thread(streamCallbackBackground, streamPtr, hmdPtr);
    }
    else
    {
        // ROS_WARN("st ream data is not processed yet");
    }
}
void streamCallbackRviz(const sensor_msgs::ImageConstPtr& rvizImg, HMD* hmdPtr)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(rvizImg, sensor_msgs::image_encodings::RGBA8);
        std::cout << "hi" << std::endl;
        hmdPtr->RvizScreen = cv_ptr->image;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}



/* Shader Compiler  */
GLuint HMD::CompileGLShader(const char* pchShaderName, const char* pchVertexShader, const char* pchFragmentShader)
{
    GLuint unProgramID = glCreateProgram();

    GLuint nSceneVertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(nSceneVertexShader, 1, &pchVertexShader, NULL);
    glCompileShader(nSceneVertexShader);

    GLint vShaderCompiled = GL_FALSE;
    glGetShaderiv(nSceneVertexShader, GL_COMPILE_STATUS, &vShaderCompiled);
    if (vShaderCompiled != GL_TRUE)
    {
        std::cout << "Unable to compile vertex shader!" << std::endl;

        glDeleteProgram(unProgramID);
        glDeleteShader(nSceneVertexShader);
        return 0;
    }
    glAttachShader(unProgramID, nSceneVertexShader);
    glDeleteShader(nSceneVertexShader); // the program hangs onto this once it's attached

    GLuint  nSceneFragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(nSceneFragmentShader, 1, &pchFragmentShader, NULL);
    glCompileShader(nSceneFragmentShader);

    GLint fShaderCompiled = GL_FALSE;
    glGetShaderiv(nSceneFragmentShader, GL_COMPILE_STATUS, &fShaderCompiled);
    if (fShaderCompiled != GL_TRUE)
    {
        std::cout<<"Unable to compile fragment shader" << std::endl;

        glDeleteProgram(unProgramID);
        glDeleteShader(nSceneFragmentShader);
        return 0;
    }

    glAttachShader(unProgramID, nSceneFragmentShader);
    glDeleteShader(nSceneFragmentShader); // the program hangs onto this once it's attached

    glLinkProgram(unProgramID);

    GLint programSuccess = GL_TRUE;
    glGetProgramiv(unProgramID, GL_LINK_STATUS, &programSuccess);
    if (programSuccess != GL_TRUE)
    {
        std::cout<<"Error linking program" << std::endl;
        glDeleteProgram(unProgramID);
        return 0;
    }

    glUseProgram(unProgramID);
    glUseProgram(0);

    return unProgramID;
}

bool HMD::CreateAllShaders()
{
    m_unSceneProgramID = CompileGLShader(
        "Scene",

        // Vertex Shader
        "#version 410\n"
        "uniform mat4 matrix;\n"
        "layout(location = 0) in vec4 position;\n"
        "layout(location = 1) in vec2 v2UVcoordsIn;\n"
        "layout(location = 2) in vec3 v3NormalIn;\n"
        "out vec2 v2UVcoords;\n"
        "void main()\n"
        "{\n"
        "	v2UVcoords = v2UVcoordsIn;\n"
        "	gl_Position = matrix * position;\n"
        "}\n",

        // Fragment Shader
        "#version 410 core\n"
        "uniform sampler2D mytexture;\n"
        "in vec2 v2UVcoords;\n"
        "out vec4 outputColor;\n"
        "void main()\n"
        "{\n"
        "   outputColor = texture(mytexture, v2UVcoords);\n"
        "}\n"
    );
    m_nSceneMatrixLocation = glGetUniformLocation(m_unSceneProgramID, "matrix");
    if (m_nSceneMatrixLocation == -1)
    {
        std::cout << "Unable to find matrix uniform in scene shader" << std::endl;
        return false;
    }

    m_unControllerTransformProgramID = CompileGLShader(
        "Controller",

        // vertex shader
        "#version 410\n"
        "uniform mat4 matrix;\n"
        "layout(location = 0) in vec4 position;\n"
        "layout(location = 1) in vec3 v3ColorIn;\n"
        "out vec4 v4Color;\n"
        "void main()\n"
        "{\n"
        "	v4Color.xyz = v3ColorIn; v4Color.a = 1.0;\n"
        "	gl_Position = matrix * position;\n"
        "}\n",

        // fragment shader
        "#version 410\n"
        "in vec4 v4Color;\n"
        "out vec4 outputColor;\n"
        "void main()\n"
        "{\n"
        "   outputColor = v4Color;\n"
        "}\n"
    );
    m_nControllerMatrixLocation = glGetUniformLocation(m_unControllerTransformProgramID, "matrix");
    if (m_nControllerMatrixLocation == -1)
    {
        std::cout << "Unable to find matrix uniform in controller shader" << std::endl;
        return false;
    }

    m_unRenderModelProgramID = CompileGLShader(
        "render model",

        // vertex shader
        "#version 410\n"
        "uniform mat4 matrix;\n"
        "layout(location = 0) in vec4 position;\n"
        "layout(location = 1) in vec3 v3NormalIn;\n"
        "layout(location = 2) in vec2 v2TexCoordsIn;\n"
        "out vec2 v2TexCoord;\n"
        "void main()\n"
        "{\n"
        "	v2TexCoord = v2TexCoordsIn;\n"
        "	gl_Position = matrix * vec4(position.xyz, 1);\n"
        "}\n",

        //fragment shader
        "#version 410 core\n"
        "uniform sampler2D diffuse;\n"
        "in vec2 v2TexCoord;\n"
        "out vec4 outputColor;\n"
        "void main()\n"
        "{\n"
        "   outputColor = texture( diffuse, v2TexCoord);\n"
        "}\n"

    );
    m_nRenderModelMatrixLocation = glGetUniformLocation(m_unRenderModelProgramID, "matrix");
    if (m_nRenderModelMatrixLocation == -1)
    {
        std::cout<<"Unable to find matrix uniform in render model shader"<<std::endl;

        return false;
    }

    m_unCompanionWindowProgramID = CompileGLShader(
        "CompanionWindow",

        // vertex shader
        "#version 410 core\n"
        "layout(location = 0) in vec4 position;\n"
        "layout(location = 1) in vec2 v2UVIn;\n"
        "noperspective out vec2 v2UV;\n"
        "void main()\n"
        "{\n"
        "	v2UV = vec2(v2UVIn.s, 1.0 - v2UVIn.t);\n"
        "	gl_Position = position;\n"
        "}\n",

        // fragment shader
        "#version 410 core\n"
        "uniform sampler2D mytexture;\n"
        "noperspective in vec2 v2UV;\n"
        "out vec4 outputColor;\n"
        "void main()\n"
        "{\n"
        "		outputColor = texture(mytexture, v2UV);\n"
        "}\n"
    );

    return m_unSceneProgramID != 0
        && m_unControllerTransformProgramID != 0
        && m_unRenderModelProgramID != 0
        && m_unCompanionWindowProgramID != 0;
}

/* Setup OpenGL Texture using image  */
bool HMD::SetupTexturemaps()
{
    std::string sExecutableDirectory = Path_StripFilename(Path_GetExecutablePath());
    std::string strFullPath = Path_MakeAbsolute("C:/Users/Dyros/Desktop/avatar/src/VR/src/panoramaa.jpg", sExecutableDirectory);

    std::vector<unsigned char> imageRGBA;
 
    unsigned nImageWidth, nImageHeight;
    unsigned nError = lodepng::decode(imageRGBA, nImageWidth, nImageHeight, strFullPath.c_str());

    if (nError != 0) {
        std::cout << "Setupe Texture map process failed.. program exit" << std::endl;
        return false;
    }
    //std::cout << imageRGBA.size() << std::endl; 
    for (int i = 0; i < 6; i++) {
        glGenTextures(1, &m_Texture1[i]);
        glGenTextures(1, &m_Texture2[i]);
        glBindTexture(GL_TEXTURE_2D, m_Texture1[i]);
        glBindTexture(GL_TEXTURE_2D, m_Texture2[i]);


        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, nImageWidth, nImageHeight,
            0, GL_RGBA, GL_UNSIGNED_BYTE, &imageRGBA[0]);

        glGenerateMipmap(GL_TEXTURE_2D);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);

        GLfloat fLargest;
        glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &fLargest);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, fLargest);

        glBindTexture(GL_TEXTURE_2D, 0);
    }
    return true;
}

void HMD::AddCubeVertex(float fl0, float fl1, float fl2, float fl3, float fl4, std::vector<float>& vertdata)
{
    vertdata.push_back(fl0);
    vertdata.push_back(fl1);
    vertdata.push_back(fl2);
    vertdata.push_back(fl3);
    vertdata.push_back(fl4);
}

/* Make Cube geometry  */
void HMD::AddCubeToScene(Matrix4 mat, std::vector<float>& vertdata, int flag)
{
    // Matrix4 mat( outermat.data() );
    
    if (hmd_init == false){
    VRSystem->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseSeated, 0, m_rTrackedDevicePose, vr::k_unMaxTrackedDeviceCount);
    HMD_init = map2eigen(m_rTrackedDevicePose[HMD_INDEX].mDeviceToAbsoluteTracking.m);
    Matrix3f hmd_rotation= HMD_init.block(0,0,3,3);
    Vector3d rpy = rot2Euler(hmd_rotation);
    yaw_angle = -rpy(1);
    hmd_init = true;
    }
    // Matrix4d hmd_yaw;
    // hmd_yaw.setIdentity();

    // hmd_yaw(0, 0) = cos(yaw_angle);
    // hmd_yaw(1, 0) = sin(yaw_angle);
    // hmd_yaw(2, 0) = 0.0;

    // hmd_yaw(0, 1) = -sin(yaw_angle);
    // hmd_yaw(1, 1) = cos(yaw_angle);
    // hmd_yaw(2, 1) = 0.0;

    // hmd_yaw(0, 2) = 0.0;
    // hmd_yaw(1, 2) = 0.0;
    // hmd_yaw(2, 2) = 1.0;

    //Matrix4 hmd_yaw(cos(yaw_angle),0.0, sin(yaw_angle),0.0,     0.0, 1.0, 0.0, 0.0,     -sin(yaw_angle),0.0, cos(yaw_angle) , 0.0,      0.0, 0.0, 0.0, 1.0 );
    // std::cout<<"Yaw angle "<<yaw_angle << std::endl;
    /*
    Vector4 A = mat * hmd_yaw * Vector4(-0.5, 0, -0.5, 1);
    Vector4 B = mat * hmd_yaw * Vector4(0.5, 0, -0.5, 1);
    Vector4 C = mat * hmd_yaw * Vector4(0.5, 1, -0.5, 1);
    Vector4 D = mat * hmd_yaw * Vector4(-0.5, 1, -0.5, 1);
    Vector4 E = mat * hmd_yaw * Vector4(-0.5, 0, 0.5, 1);
    Vector4 F = mat * hmd_yaw * Vector4(0.5, 0, 0.5, 1);
    Vector4 G = mat * hmd_yaw * Vector4(0.5, 1, 0.5, 1);
    Vector4 H = mat * hmd_yaw * Vector4(-0.5, 1, 0.5, 1);
    */
    
    Vector4 A = mat * Vector4(-0.5, 0, -0.5, 1);
    Vector4 B = mat * Vector4(0.5, 0, -0.5, 1);
    Vector4 C = mat * Vector4(0.5, 1, -0.5, 1);
    Vector4 D = mat * Vector4(-0.5, 1, -0.5, 1);
    Vector4 E = mat * Vector4(-0.5, 0, 0.5, 1);
    Vector4 F = mat * Vector4(0.5, 0, 0.5, 1);
    Vector4 G = mat * Vector4(0.5, 1, 0.5, 1);
    Vector4 H = mat * Vector4(-0.5, 1, 0.5, 1);
    
    if (flag == 0){
        // triangles instead of quads
        AddCubeVertex(E.x, E.y, E.z, 0, 1, vertdata); //Front
        AddCubeVertex(F.x, F.y, F.z, 1, 1, vertdata);
        AddCubeVertex(G.x, G.y, G.z, 1, 0, vertdata);
        AddCubeVertex(G.x, G.y, G.z, 1, 0, vertdata);
        AddCubeVertex(H.x, H.y, H.z, 0, 0, vertdata);
        AddCubeVertex(E.x, E.y, E.z, 0, 1, vertdata);
    }
    else if (flag == 1) {
        AddCubeVertex(B.x, B.y, B.z, 0, 1, vertdata); //Back
        AddCubeVertex(A.x, A.y, A.z, 1, 1, vertdata);
        AddCubeVertex(D.x, D.y, D.z, 1, 0, vertdata);
        AddCubeVertex(D.x, D.y, D.z, 1, 0, vertdata);
        AddCubeVertex(C.x, C.y, C.z, 0, 0, vertdata);
        AddCubeVertex(B.x, B.y, B.z, 0, 1, vertdata);
    }
    else if (flag == 2) {
        AddCubeVertex(H.x, H.y, H.z, 0, 1, vertdata); //Top
        AddCubeVertex(G.x, G.y, G.z, 1, 1, vertdata);
        AddCubeVertex(C.x, C.y, C.z, 1, 0, vertdata);
        AddCubeVertex(C.x, C.y, C.z, 1, 0, vertdata);
        AddCubeVertex(D.x, D.y, D.z, 0, 0, vertdata);
        AddCubeVertex(H.x, H.y, H.z, 0, 1, vertdata);
    }
    else if (flag == 3) {
        AddCubeVertex(A.x, A.y, A.z, 0, 1, vertdata); //Bottom
        AddCubeVertex(B.x, B.y, B.z, 1, 1, vertdata);
        AddCubeVertex(F.x, F.y, F.z, 1, 0, vertdata);
        AddCubeVertex(F.x, F.y, F.z, 1, 0, vertdata);
        AddCubeVertex(E.x, E.y, E.z, 0, 0, vertdata);
        AddCubeVertex(A.x, A.y, A.z, 0, 1, vertdata);
    }
    else if (flag == 4) {
        AddCubeVertex(F.x, F.y, F.z, 0, 1, vertdata); //Right
        AddCubeVertex(B.x, B.y, B.z, 1, 1, vertdata);
        AddCubeVertex(C.x, C.y, C.z, 1, 0, vertdata);
        AddCubeVertex(C.x, C.y, C.z, 1, 0, vertdata);
        AddCubeVertex(G.x, G.y, G.z, 0, 0, vertdata);
        AddCubeVertex(F.x, F.y, F.z, 0, 1, vertdata);
    }
    else if (flag == 5) {
        AddCubeVertex(A.x, A.y, A.z, 0, 1, vertdata); //Left
        AddCubeVertex(E.x, E.y, E.z, 1, 1, vertdata);
        AddCubeVertex(H.x, H.y, H.z, 1, 0, vertdata);
        AddCubeVertex(H.x, H.y, H.z, 1, 0, vertdata);
        AddCubeVertex(D.x, D.y, D.z, 0, 0, vertdata);
        AddCubeVertex(A.x, A.y, A.z, 0, 1, vertdata);
    }
}

void HMD::AddScreenToScene(Matrix4 mat, std::vector<float>& vertdata)
{
    // Matrix4 mat( outermat.data() );
    //Matrix4 poseHMD = GetHMDMatrixPoseEye(vr::Eye_Left);
    //Matrix4 staticMat = poseHMD.invertEuclidean() * mat;
    Vector4 I = mat * Vector4(0.1, 0.1, 0.25, 1);
    Vector4 J = mat * Vector4(-0.1, 0.1, 0.25, 1);
    Vector4 K = mat * Vector4(-0.1, 0.3, 0.25, 1);
    Vector4 L = mat * Vector4(0.1, 0.3, 0.25, 1);
 
    AddCubeVertex(J.x, J.y, J.z, 0, 1, vertdata); //Back
    AddCubeVertex(I.x, I.y, I.z, 1, 1, vertdata);
    AddCubeVertex(L.x, L.y, L.z, 1, 0, vertdata);
    AddCubeVertex(L.x, L.y, L.z, 1, 0, vertdata);
    AddCubeVertex(K.x, K.y, K.z, 0, 0, vertdata);
    AddCubeVertex(J.x, J.y, J.z, 0, 1, vertdata);
}




/* create sea of cubes */
void HMD::SetupScene()
{
    if (!VRSystem) {
        std::cout << "Setup scene process failed.. program exit" << std::endl;
        return;
    }

    std::vector<std::vector<float>> vertdataarrays;
    for (int i = 0; i < 6; i++) {
        std::vector<float> array;
        vertdataarrays.push_back(array);
    }

    Matrix4 matScale;
    matScale.scale(m_fScale, m_fScale, m_fScale);
    Matrix4 matTransform;
    matTransform.translate(
        -((float)m_iSceneVolumeWidth * m_fScaleSpacingWidth) / 2.f,
        -((float)m_iSceneVolumeHeight * m_fScaleSpacingHeight) / 2.f,
        -((float)m_iSceneVolumeDepth * m_fScaleSpacingDepth) / 2.f);

    
    
    for (int i = 0; i < 6; i++) {

        Matrix4 mat = matScale * matTransform;

        for (int z = 0; z < m_iSceneVolumeDepth; z++)
        {
            for (int y = 0; y < m_iSceneVolumeHeight; y++)
            {
                for (int x = 0; x < m_iSceneVolumeWidth; x++)
                {
                    if (i < 6) {
                        AddCubeToScene(mat, vertdataarrays[i], i);
                    }
                    else {
                        AddScreenToScene(mat, vertdataarrays[i]);
                    }
                    mat = mat * Matrix4().translate(m_fScaleSpacing, 0, 0);
                }
                mat = mat * Matrix4().translate(-((float)m_iSceneVolumeWidth) * m_fScaleSpacing, m_fScaleSpacing, 0);
            }
            mat = mat * Matrix4().translate(0, -((float)m_iSceneVolumeHeight) * m_fScaleSpacing, m_fScaleSpacing);
        }
        m_uiVertcount[i] = vertdataarrays[i].size() / 5;

        glGenVertexArrays(1, &m_unSceneVAO[i]);
        glBindVertexArray(m_unSceneVAO[i]);

        glGenBuffers(1, &m_glSceneVertBuffer[i]);
        glBindBuffer(GL_ARRAY_BUFFER, m_glSceneVertBuffer[i]);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertdataarrays[i].size(), &vertdataarrays[i][0], GL_STATIC_DRAW);

        GLsizei stride = sizeof(VertexDataScene);
        uintptr_t offset = 0;

        glEnableVertexAttribArray(0); //////
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (const void*)offset);

        offset += sizeof(Vector3);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, stride, (const void*)offset);

        glBindVertexArray(0);
        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);
    }
}

Matrix4 HMD::GetHMDMatrixProjectionEye(vr::Hmd_Eye nEye)
{
    if (!VRSystem) {
        std::cout << "Failed to Get HMD Projection matrix" << std::endl;
        return Matrix4();
    }
    vr::HmdMatrix44_t mat = VRSystem->GetProjectionMatrix(nEye, m_fNearClip, m_fFarClip);
    // for (int i =0; i<4 ; i++){
    //     for (int j = 0; j < 4 ; j++){
    //         std::cout << "mat.m" << i << j << " : " <<mat.m[i][j] << std::endl;
    //     }
    //}
    return Matrix4(
        mat.m[0][0], mat.m[1][0], mat.m[2][0], mat.m[3][0],
        mat.m[0][1], mat.m[1][1], mat.m[2][1], mat.m[3][1],
        mat.m[0][2], mat.m[1][2], mat.m[2][2], mat.m[3][2],
        mat.m[0][3], mat.m[1][3], mat.m[2][3], mat.m[3][3]
    );
}

Matrix4 HMD::GetHMDMatrixPoseEye(vr::Hmd_Eye nEye, const float eye_distance = 0.0, const float eye_angle = 0.0)
{
    if (!VRSystem)
    {
        std::cout << "Failed to Get Pose matrix" << std::endl;
        return Matrix4();
    }
    vr::HmdMatrix34_t matEyeRight = VRSystem->GetEyeToHeadTransform(nEye);
    std::cout <<"which eye ? : " << nEye << std::endl;
    
    if (nEye == vr::Eye_Right) {
        matEyeRight.m[0][3] = eye_distance; //  눈 사이 거리//
        matEyeRight.m[0][0] = cos(eye_angle);
        matEyeRight.m[0][2] = sin(eye_angle);
        matEyeRight.m[2][0] = -sin(eye_angle);    // right : +x, front: -z, top : +y
        matEyeRight.m[2][2] = cos(eye_angle); 
    }
    else {
        matEyeRight.m[0][3] = -eye_distance;
        matEyeRight.m[0][0] = cos(eye_angle);
        matEyeRight.m[0][2] = -sin(eye_angle);
        matEyeRight.m[2][0] = sin(eye_angle);    // right : +x, front: -z, top : +y
        matEyeRight.m[2][2] = cos(eye_angle);   //matEyeRight = rotation matrix
               
        
    }

    // for (int i =0; i<3 ; i++){
    //     for (int j = 0; j < 4 ; j++){
    //         std::cout << nEye <<"  matEyeRight.m" <<  i << j << " : " << matEyeRight.m[i][j] << std::endl;
    //     }
    //}
    Matrix4 matrixObj(
        matEyeRight.m[0][0], matEyeRight.m[1][0], matEyeRight.m[2][0], 0.0,
        matEyeRight.m[0][1], matEyeRight.m[1][1], matEyeRight.m[2][1], 0.0,
        matEyeRight.m[0][2], matEyeRight.m[1][2], matEyeRight.m[2][2], 0.0,
        matEyeRight.m[0][3], matEyeRight.m[1][3], matEyeRight.m[2][3], 1.0f
    );

    return matrixObj.invert();
}


/*
projection
left
mat.m00 : 0.77164
mat.m01 : 0
mat.m02 : -0.18781
mat.m03 : 0
mat.m10 : 0
mat.m11 : 0.709185
mat.m12 : 0.00246117
mat.m13 : 0
mat.m20 : 0
mat.m21 : 0
mat.m22 : -1.00334
mat.m23 : -0.100334

right
mat.m00 : 0.771024
mat.m01 : 0
mat.m02 : 0.187732
mat.m03 : 0
mat.m10 : 0
mat.m11 : 0.708744
mat.m12 : 0.00221074
mat.m13 : 0
mat.m20 : 0
mat.m21 : 0
mat.m22 : -1.00334
mat.m23 : -0.100334

pose
left
matEyeRight.m00 : 1
matEyeRight.m01 : 0
matEyeRight.m02 : 0
matEyeRight.m03 : -0.035 // 눈 사이 거리??
matEyeRight.m10 : 0
matEyeRight.m11 : 1
matEyeRight.m12 : 0
matEyeRight.m13 : 0
matEyeRight.m20 : 0
matEyeRight.m21 : 0
matEyeRight.m22 : 1
matEyeRight.m23 : 0

right
matEyeRight.m00 : 1
matEyeRight.m01 : 0
matEyeRight.m02 : 0
matEyeRight.m03 : 0.035
matEyeRight.m10 : 0
matEyeRight.m11 : 1
matEyeRight.m12 : 0
matEyeRight.m13 : 0
matEyeRight.m20 : 0
matEyeRight.m21 : 0
matEyeRight.m22 : 1
matEyeRight.m23 : 0


*/

void HMD::SetupCameras()
{
    m_mat4ProjectionLeft = GetHMDMatrixProjectionEye(vr::Eye_Left);
    m_mat4ProjectionRight = GetHMDMatrixProjectionEye(vr::Eye_Right);
    m_mat4eyePosLeft = GetHMDMatrixPoseEye(vr::Eye_Left, 0.0f);
    m_mat4eyePosRight = GetHMDMatrixPoseEye(vr::Eye_Right, 0.0f);
}


/* Create Frame buffer  */
bool HMD::CreateFrameBuffer(int nWidth, int nHeight, FramebufferDesc& framebufferDesc)
{
    glGenFramebuffers(1, &framebufferDesc.m_nRenderFramebufferId);
    glBindFramebuffer(GL_FRAMEBUFFER, framebufferDesc.m_nRenderFramebufferId);

    glGenRenderbuffers(1, &framebufferDesc.m_nDepthBufferId);
    glBindRenderbuffer(GL_RENDERBUFFER, framebufferDesc.m_nDepthBufferId);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, 4, GL_DEPTH_COMPONENT, nWidth, nHeight);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, framebufferDesc.m_nDepthBufferId);

    glGenTextures(1, &framebufferDesc.m_nRenderTextureId);
    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, framebufferDesc.m_nRenderTextureId);
    glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, 4, GL_RGBA8, nWidth, nHeight, true);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D_MULTISAMPLE, framebufferDesc.m_nRenderTextureId, 0);

    glGenFramebuffers(1, &framebufferDesc.m_nResolveFramebufferId);
    glBindFramebuffer(GL_FRAMEBUFFER, framebufferDesc.m_nResolveFramebufferId);

    glGenTextures(1, &framebufferDesc.m_nResolveTextureId);
    glBindTexture(GL_TEXTURE_2D, framebufferDesc.m_nResolveTextureId);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, nWidth, nHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, framebufferDesc.m_nResolveTextureId, 0);

    // check FBO status
    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if (status != GL_FRAMEBUFFER_COMPLETE)
    {
        return false;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    return true;
}

bool HMD::SetupStereoRenderTargets()
{
    if (!VRSystem) {
        std::cout << "Setup Stereo Render Target process failed.. program exit" << std::endl;
        return false;
    }
    VRSystem->GetRecommendedRenderTargetSize(&m_nRenderWidth, &m_nRenderHeight);
    // m_nRenderWidth = 1440;
    // m_nRenderHeight = 1660;

    CreateFrameBuffer(m_nRenderWidth, m_nRenderHeight, leftEyeDesc);
    CreateFrameBuffer(m_nRenderWidth, m_nRenderHeight, rightEyeDesc);

    return true;
}

void HMD::SetupCompanionWindow()
{
    if (!VRSystem)
        return;

    std::vector<VertexDataWindow> vVerts;
    
    // left eye verts
    vVerts.push_back(VertexDataWindow(Vector2(-1, -1), Vector2(0, 1)));
    vVerts.push_back(VertexDataWindow(Vector2(0, -1), Vector2(1, 1)));
    vVerts.push_back(VertexDataWindow(Vector2(-1, 1), Vector2(0, 0)));
    vVerts.push_back(VertexDataWindow(Vector2(0, 1), Vector2(1, 0)));

    // right eye verts
    vVerts.push_back(VertexDataWindow(Vector2(0, -1), Vector2(0, 1)));
    vVerts.push_back(VertexDataWindow(Vector2(1, -1), Vector2(1, 1)));
    vVerts.push_back(VertexDataWindow(Vector2(0, 1), Vector2(0, 0)));
    vVerts.push_back(VertexDataWindow(Vector2(1, 1), Vector2(1, 0)));

    GLushort vIndices[] = { 0, 1, 3,   0, 3, 2,   4, 5, 7,   4, 7, 6 }; //four triangles
    m_uiCompanionWindowIndexSize = _countof(vIndices);

    glGenVertexArrays(1, &m_unCompanionWindowVAO);
    glBindVertexArray(m_unCompanionWindowVAO);

    glGenBuffers(1, &m_glCompanionWindowIDVertBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, m_glCompanionWindowIDVertBuffer);
    glBufferData(GL_ARRAY_BUFFER, vVerts.size() * sizeof(VertexDataWindow), &vVerts[0], GL_STATIC_DRAW);

    glGenBuffers(1, &m_glCompanionWindowIDIndexBuffer);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_glCompanionWindowIDIndexBuffer);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_uiCompanionWindowIndexSize * sizeof(GLushort), &vIndices[0], GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(VertexDataWindow), (void*)offsetof(VertexDataWindow, position));

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(VertexDataWindow), (void*)offsetof(VertexDataWindow, texCoord));

    glBindVertexArray(0);

    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

/* Returns true if the action is active and had a rising edge*/
bool GetDigitalActionRisingEdge(vr::VRActionHandle_t action, vr::VRInputValueHandle_t* pDevicePath = nullptr)
{
    vr::InputDigitalActionData_t actionData;
    vr::VRInput()->GetDigitalActionData(action, &actionData, sizeof(actionData), vr::k_ulInvalidInputValueHandle);
    if (pDevicePath)
    {
        *pDevicePath = vr::k_ulInvalidInputValueHandle;
        if (actionData.bActive)
        {
            vr::InputOriginInfo_t originInfo;
            if (vr::VRInputError_None == vr::VRInput()->GetOriginTrackedDeviceInfo(actionData.activeOrigin, &originInfo, sizeof(originInfo)))
            {
                *pDevicePath = originInfo.devicePath;
            }
        }
    }
    return actionData.bActive && actionData.bChanged && actionData.bState;
}

/* Returns true if the action is active and its state is true */
bool GetDigitalActionState(vr::VRActionHandle_t action, vr::VRInputValueHandle_t* pDevicePath = nullptr)
{
    vr::InputDigitalActionData_t actionData;
    vr::VRInput()->GetDigitalActionData(action, &actionData, sizeof(actionData), vr::k_ulInvalidInputValueHandle);
    if (pDevicePath)
    {
        *pDevicePath = vr::k_ulInvalidInputValueHandle;
        if (actionData.bActive)
        {
            vr::InputOriginInfo_t originInfo;
            if (vr::VRInputError_None == vr::VRInput()->GetOriginTrackedDeviceInfo(actionData.activeOrigin, &originInfo, sizeof(originInfo)))
            {
                *pDevicePath = originInfo.devicePath;
            }
        }
    }
    return actionData.bActive && actionData.bState;
}


void HMD::ProcessVREvent(const vr::VREvent_t& event)
{
    switch (event.eventType)
    {
    case vr::VREvent_TrackedDeviceDeactivated:
    {
        std::cout << "Device detached" << std::endl;;
    }
    break;
    case vr::VREvent_TrackedDeviceUpdated:
    {
        std::cout << "Device updated" << std::endl;;
    }
    break;
    }
}

RenderModel* HMD::FindOrLoadRenderModel(const char* pchRenderModelName)
{   
    /// ros_warn 했는데 출력이 안됨.
    RenderModel* pRenderModel = NULL;
    for (std::vector< RenderModel* >::iterator i = m_vecRenderModels.begin(); i != m_vecRenderModels.end(); i++)
    {
        if (!stricmp((*i)->GetName().c_str(), pchRenderModelName))
        {
            pRenderModel = *i;
            break;
        }
    }

    // load the model if we didn't find one
    if (!pRenderModel)
    {
        vr::RenderModel_t* pModel;
        vr::EVRRenderModelError error;
        while (1)
        {
            error = vr::VRRenderModels()->LoadRenderModel_Async(pchRenderModelName, &pModel);
            if (error != vr::VRRenderModelError_Loading)
                break;

            ThreadSleep(1);
        }

        if (error != vr::VRRenderModelError_None)
        {
            std::cout << "Unable to load render model" <<  pchRenderModelName << vr::VRRenderModels()->GetRenderModelErrorNameFromEnum(error) << std::endl;
            return NULL; // move on to the next tracked device
        }

        vr::RenderModel_TextureMap_t* pTexture;
        while (1)
        {
            error = vr::VRRenderModels()->LoadTexture_Async(pModel->diffuseTextureId, &pTexture);
            if (error != vr::VRRenderModelError_Loading)
                break;

            ThreadSleep(1);
        }

        if (error != vr::VRRenderModelError_None)
        {
            std::cout<<"Unable to load render texture id:  " << pModel->diffuseTextureId << "for render model: " << pchRenderModelName << std::endl;
            vr::VRRenderModels()->FreeRenderModel(pModel);
            return NULL; // move on to the next tracked device
        }

        pRenderModel = new RenderModel(pchRenderModelName);
        if (!pRenderModel->BInit(*pModel, *pTexture))
        {
            std::cout<<"Unable to create GL model from render model: " <<  pchRenderModelName << std::endl;
            delete pRenderModel;
            pRenderModel = NULL;
        }
        else
        {
            m_vecRenderModels.push_back(pRenderModel);
        }
        vr::VRRenderModels()->FreeRenderModel(pModel);
        vr::VRRenderModels()->FreeTexture(pTexture);
    }
    return pRenderModel;
}

Matrix4 HMD::ConvertSteamVRMatrixToMatrix4(const vr::HmdMatrix34_t& matPose)
{
    Matrix4 matrixObj(
        matPose.m[0][0], matPose.m[1][0], matPose.m[2][0], 0.0,
        matPose.m[0][1], matPose.m[1][1], matPose.m[2][1], 0.0,
        matPose.m[0][2], matPose.m[1][2], matPose.m[2][2], 0.0,
        matPose.m[0][3], matPose.m[1][3], matPose.m[2][3], 1.0f
    );
    return matrixObj;
}

/* Event Handler */
bool HMD::HandleInput()
{
    SDL_Event sdlEvent;
    bool bRet = false;

    while (SDL_PollEvent(&sdlEvent) != 0)
    {
        if (sdlEvent.type == SDL_QUIT)
        {
            bRet = true;
        }
        else if (sdlEvent.type == SDL_KEYDOWN)
        {
            if (sdlEvent.key.keysym.sym == SDLK_ESCAPE
                || sdlEvent.key.keysym.sym == SDLK_q)
            {
                bRet = true;
            }
            if (sdlEvent.key.keysym.sym == SDLK_c)
            {
                m_bShowCubes = !m_bShowCubes;
            }
        }
    }
    // Process SteamVR events
    vr::VREvent_t event;
    while (VRSystem->PollNextEvent(&event, sizeof(event)))
    {
        ProcessVREvent(event);
    }

    // Process SteamVR action state
    // UpdateActionState is called each frame to update the state of the actions themselves. The application
    // controls which action sets are active with the provided array of VRActiveActionSet_t structs.
    vr::VRActiveActionSet_t actionSet = { 0 };
    actionSet.ulActionSet = m_actionsetDemo;
    vr::VRInput()->UpdateActionState(&actionSet, sizeof(actionSet), 1);

    m_bShowCubes = !GetDigitalActionState(m_actionHideCubes);

    vr::VRInputValueHandle_t ulHapticDevice;
    if (GetDigitalActionRisingEdge(m_actionTriggerHaptic, &ulHapticDevice))
    {
        if (ulHapticDevice == m_rHand[Left].m_source)
        {
            vr::VRInput()->TriggerHapticVibrationAction(m_rHand[Left].m_actionHaptic, 0, 1, 4.f, 1.0f, vr::k_ulInvalidInputValueHandle);
        }
        if (ulHapticDevice == m_rHand[Right].m_source)
        {
            vr::VRInput()->TriggerHapticVibrationAction(m_rHand[Right].m_actionHaptic, 0, 1, 4.f, 1.0f, vr::k_ulInvalidInputValueHandle);
        }
    }

    vr::InputAnalogActionData_t analogData;
    if (vr::VRInput()->GetAnalogActionData(m_actionAnalongInput, &analogData, sizeof(analogData), vr::k_ulInvalidInputValueHandle) == vr::VRInputError_None && analogData.bActive)
    {
        m_vAnalogValue[0] = analogData.x;
        m_vAnalogValue[1] = analogData.y;
    }

    m_rHand[Left].m_bShowController = true;
    m_rHand[Right].m_bShowController = true;

    vr::VRInputValueHandle_t ulHideDevice;
    if (GetDigitalActionState(m_actionHideThisController, &ulHideDevice))
    {
        if (ulHideDevice == m_rHand[Left].m_source)
        {
            m_rHand[Left].m_bShowController = false;
        }
        if (ulHideDevice == m_rHand[Right].m_source)
        {
            m_rHand[Right].m_bShowController = false;
        }
    }

    for (EHand eHand = Left; eHand <= Right; ((int&)eHand)++)
    {
        vr::InputPoseActionData_t poseData;
        if (vr::VRInput()->GetPoseActionDataForNextFrame(m_rHand[eHand].m_actionPose, vr::TrackingUniverseStanding, &poseData, sizeof(poseData), vr::k_ulInvalidInputValueHandle) != vr::VRInputError_None
            || !poseData.bActive || !poseData.pose.bPoseIsValid)
        {
            m_rHand[eHand].m_bShowController = false;
        }
        else
        {
            m_rHand[eHand].m_rmat4Pose = ConvertSteamVRMatrixToMatrix4(poseData.pose.mDeviceToAbsoluteTracking);

            vr::InputOriginInfo_t originInfo;
            if (vr::VRInput()->GetOriginTrackedDeviceInfo(poseData.activeOrigin, &originInfo, sizeof(originInfo)) == vr::VRInputError_None
                && originInfo.trackedDeviceIndex != vr::k_unTrackedDeviceIndexInvalid)
            {
                std::string sRenderModelName = GetTrackedDeviceString(originInfo.trackedDeviceIndex, vr::Prop_RenderModelName_String);
                if (sRenderModelName != m_rHand[eHand].m_sRenderModelName)
                {

                    //m_rHand[eHand].m_pRenderModel = FindOrLoadRenderModel(sRenderModelName.c_str()); 없어도 잘 돌아감.
                    m_rHand[eHand].m_sRenderModelName = sRenderModelName;
                }
            }
        }
    }

    return bRet;
}



/* HMD Initialization IMPLEMENTATION*/


void HMD::init() {
    ros::init(this->argc_arg, this->argv_arg, "HMD");
    ros::NodeHandle node;
    // ros::AsyncSpinner spinner(0);
    // spinner.start();
    hmd_pub = node.advertise<VR::matrix_3_4>("HMD", 1000);
    leftCon_pub = node.advertise<VR::matrix_3_4>("LEFTCONTROLLER", 1000);
    rightCon_pub = node.advertise<VR::matrix_3_4>("RIGHTCONTROLLER", 1000);
    for (int i=0; i<trackerNum; i++)
    {
        std::string topic_name = "TRACKER" + std::to_string(i);
        tracker_pub[i] = node.advertise<VR::matrix_3_4>(topic_name, 1000);
    }
    tracker_status_pub = node.advertise<std_msgs::Bool>("TRACKERSTATUS", 1000);

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) < 0)
    {   
        printf("%s - SDL could not initialize! SDL Error: %s\n", __FUNCTION__, SDL_GetError());
        return;
    }

    eError = vr::VRInitError_None;
    VRSystem = vr::VR_Init(&eError, vr::VRApplication_Scene);
    // VRSystem->ResetSeatedZeroPose();
    std::cout<<"Init"<<std::endl;
    if (eError != vr::VRInitError_None)
    {
        char buf[1024];
        sprintf_s(buf, sizeof(buf), "Unable to init VR runtime: %s", vr::VR_GetVRInitErrorAsEnglishDescription(eError));
        std::cout <<  "Unable to init VR runtime" << std::endl;
        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "VR_Init Failed", buf, NULL);
        return;
    }
    /* SDL based Window Creation Process Start */

    int nWindowPosX = 700;
    int nWindowPosY = 100;
    Uint32 unWindowFlags = SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN;

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1);
    //SDL_GL_SetAttribute( SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_COMPATIBILITY );
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);

    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 0);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 0);

    if (m_bDebugOpenGL)
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_DEBUG_FLAG);

    m_pCompanionWindow = SDL_CreateWindow("hellovr", nWindowPosX, nWindowPosY, m_nCompanionWindowWidth, m_nCompanionWindowHeight, unWindowFlags);

    if (m_pCompanionWindow == NULL)
    {
        printf("%s - Window could not be created! SDL Error: %s\n", __FUNCTION__, SDL_GetError());
        return;
    }

    m_pContext = SDL_GL_CreateContext(m_pCompanionWindow);
    if (m_pContext == NULL)
    {
        printf("%s - OpenGL context could not be created! SDL Error: %s\n", __FUNCTION__, SDL_GetError());
        return;
    }

    glewExperimental = GL_TRUE;
    GLenum nGlewError = glewInit();


    if (nGlewError != GLEW_OK)
    {
        printf("%s - Error initializing GLEW! %s\n", __FUNCTION__, glewGetErrorString(nGlewError));
        return;
    }
    glGetError(); // toG clear the error caused deep in LEW

    if (SDL_GL_SetSwapInterval(m_bVblank ? 1 : 0) < 0)
    {
        printf("%s - Warning: Unable to set VSync! SDL Error: %s\n", __FUNCTION__, SDL_GetError());
        return;
    }

    m_strDriver = "No Driver";
    m_strDisplay = "No Display";

    m_strDriver = GetTrackedDeviceString(vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_TrackingSystemName_String);
    m_strDisplay = GetTrackedDeviceString(vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_SerialNumber_String);

    // std::string strWindowTitle = "hellovr - " + m_strDriver + " " + m_strDisplay;
    // SDL_SetWindowTitle(m_pCompanionWindow, strWindowTitle.c_str());

    // cube array
    m_iSceneVolumeWidth = m_iSceneVolumeInit;
    m_iSceneVolumeHeight = m_iSceneVolumeInit;
    m_iSceneVolumeDepth = m_iSceneVolumeInit;

    //cube size
    m_fScale = 6.0f;

    m_fScaleSpacing = 0;

    //cube position
    m_fScaleSpacingWidth = 0;
    m_fScaleSpacingHeight = 0.5;
    m_fScaleSpacingDepth = 0;

    m_fNearClip = 0.5f;
    m_fFarClip = 25.0f; 

    /* Window Creation Process End */

    /* OpenGL Initialization Start*/

    if (!CreateAllShaders())
        return;

    std::cout << "Start Connection Check" << std::endl;   
    checkConnection();                                        
    //ros::Duration(5.0).sleep();
    SetupTexturemaps();
    SetupScene();
    SetupCameras();
    SetupStereoRenderTargets();
    SetupCompanionWindow();

    /* OpenGL Initialization End*/

    if (!vr::VRCompositor()) {
        std::cout << "Compositor initialization failed" << std::endl;
        return;
    }

    std::cout << "VR Compositor Initialization Success!" << std::endl;
    



    ricoh_sub = node.subscribe<std_msgs::UInt8MultiArray>(  "/ricoh_h264_stream", 
                                                            1, 
                                                            boost::bind(streamCallback, _1, &streamObj, this));
                                                            //ros::TransportHints().udp());
    hmd_para_sub = node.subscribe("/tocabi/dg/vr_caliabration_param", 10, &HMD::hmd_para_callback, this);
   
    // //image_transport::ImageTransport it(node);
    //image_transport::Subscriber rviz_sub = it.subscribe("/rviz1/camera1/image", 2, boost::bind(streamCallbackRviz, _1, this));
    std::cout<<"Before" << std::endl;
    vr::VRInput()->SetActionManifestPath(Path_MakeAbsolute("C:/Users/Dyros/Desktop/avatar/src/VR/src/hellovr_actions.json", Path_StripFilename(Path_GetExecutablePath())).c_str());
    std::cout<<"After" << std::endl;
    vr::VRInput()->GetActionHandle("/actions/demo/in/HideCubes", &m_actionHideCubes);
    vr::VRInput()->GetActionHandle("/actions/demo/in/HideThisController", &m_actionHideThisController);
    vr::VRInput()->GetActionHandle("/actions/demo/in/TriggerHaptic", &m_actionTriggerHaptic);
    vr::VRInput()->GetActionHandle("/actions/demo/in/AnalogInput", &m_actionAnalongInput);

    vr::VRInput()->GetActionSetHandle("/actions/demo", &m_actionsetDemo);

    vr::VRInput()->GetActionHandle("/actions/demo/out/Haptic_Left", &m_rHand[Left].m_actionHaptic);
    vr::VRInput()->GetInputSourceHandle("/user/hand/left", &m_rHand[Left].m_source);
    vr::VRInput()->GetActionHandle("/actions/demo/in/Hand_Left", &m_rHand[Left].m_actionPose);

    vr::VRInput()->GetActionHandle("/actions/demo/out/Haptic_Right", &m_rHand[Right].m_actionHaptic);
    vr::VRInput()->GetInputSourceHandle("/user/hand/right", &m_rHand[Right].m_source);
    vr::VRInput()->GetActionHandle("/actions/demo/in/Hand_Right", &m_rHand[Right].m_actionPose);

    hmd_init_bool = true;
    // ros::waitForShutdown();
}


void HMD::ROSTasks()
{
    ros::Rate r(1000);
    while (ros::ok())
    {
        rosPublish();
        r.sleep();
        ros::spinOnce();
    }
}

void HMD::RunMainLoop()
{   
    bool bQuit = false;
    SDL_StartTextInput();
    SDL_ShowCursor(SDL_DISABLE);
    //loop_tick_ = 0;
    std::thread t (&HMD::ROSTasks, this);

    while (ros::ok() && !bQuit)
    {
        // if (!bQuit)
        // {
        //     std::cout<<"Shutdown Finished" << std::endl;
        //     vr::VR_Shutdown();
        // }
        bQuit = HandleInput();
        RenderFrame();
    }

    t.join();
   SDL_StopTextInput();
}

// void HMD::RunMainLoop()
// {
//     bool bQuit = false;

//     SDL_StartTextInput();
//     SDL_ShowCursor(SDL_DISABLE);
//     //loop_tick_ = 0;
//     // ros::AsyncSpinner spinner(0);
//     // spinner.start();
   
//     while (ros::ok())
//     {
//         bQuit = HandleInput();
//         rosPublish();
//         RenderFrame();
//         //if(loop_tick_%10 == 0)
//         ros::spinOnce();
//         //loop_tick_++;
//     }
//    SDL_StopTextInput();
//     // ros::waitForShutdown();
// }

// void HMD::RunRosLoop()
// {
//     ros::Rate rate(1000);
//     while (ros::ok())
//     {
//         rosPublish();
//         rate.sleep();
//     }
// }

void HMD::RenderFrame()
{
    ros::Rate r(30);
    while (ros::ok())
    {
        // clock_t start, end;
        // start=clock();
        r.sleep();
        // for now as fast as possible
        
        if (first_data) //// 없어도 작동함//
        {
            continue;
        }

        render_mutex.lock();
        for (int i = 0; i < 6; i++) {
            glBindTexture(GL_TEXTURE_2D, m_Texture1[i]);
            switch (i) {
            case 0:
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, LeftcubeFront.cols, LeftcubeFront.rows,
                    0, GL_RGBA, GL_UNSIGNED_BYTE, &LeftcubeFront.data[0]);
                break;
            case 1:
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, LeftcubeBack.cols, LeftcubeBack.rows,
                    0, GL_RGBA, GL_UNSIGNED_BYTE, &LeftcubeBack.data[0]);
                break;
            case 2:
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, LeftcubeTop.cols, LeftcubeTop.rows,
                    0, GL_RGBA, GL_UNSIGNED_BYTE, &LeftcubeTop.data[0]);
                break;
            case 3:
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, LeftcubeBottom.cols, LeftcubeBottom.rows,
                    0, GL_RGBA, GL_UNSIGNED_BYTE, &LeftcubeBottom.data[0]);
                break;
            case 4:
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, LeftcubeLeft.cols, LeftcubeLeft.rows,
                    0, GL_RGBA, GL_UNSIGNED_BYTE, &LeftcubeLeft.data[0]);
                break;
            case 5:
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, LeftcubeRight.cols, LeftcubeRight.rows,
                    0, GL_RGBA, GL_UNSIGNED_BYTE, &LeftcubeRight.data[0]);
                break;
            }
            
            glGenerateMipmap(GL_TEXTURE_2D);

            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);

            GLfloat fLargest;
            glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &fLargest);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, fLargest);

            glBindTexture(GL_TEXTURE_2D, 0);

        }
        // std::cout << "width : " << LeftcubeBack.cols << std::endl;
        // std::cout << "width : " << LeftcubeTop.cols << std::endl;
        // std::cout << "height : " << LeftcubeBack.rows << std::endl;
        // std::cout << "height : " << LeftcubeTop.rows << std::endl;
/////////////////////////////////right/////////////////
        for (int i = 0; i < 6; i++) {
            glBindTexture(GL_TEXTURE_2D, m_Texture2[i]);
            switch (i) {
            case 0:
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, RightcubeFront.cols, RightcubeFront.rows,
                    0, GL_RGBA, GL_UNSIGNED_BYTE, &RightcubeFront.data[0]);
                break;
            case 1:
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, RightcubeBack.cols, RightcubeBack.rows,
                    0, GL_RGBA, GL_UNSIGNED_BYTE, &RightcubeBack.data[0]);
                break;
            case 2:
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, RightcubeTop.cols, RightcubeTop.rows,
                    0, GL_RGBA, GL_UNSIGNED_BYTE, &RightcubeTop.data[0]);
                break;
            case 3:
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, RightcubeBottom.cols, RightcubeBottom.rows,
                    0, GL_RGBA, GL_UNSIGNED_BYTE, &RightcubeBottom.data[0]);
                break;
            case 4:
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, RightcubeLeft.cols, RightcubeLeft.rows,
                    0, GL_RGBA, GL_UNSIGNED_BYTE, &RightcubeLeft.data[0]);
                break;
            case 5:
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, RightcubeRight.cols, RightcubeRight.rows,
                    0, GL_RGBA, GL_UNSIGNED_BYTE, &RightcubeRight.data[0]);
                break;
            }
            
            glGenerateMipmap(GL_TEXTURE_2D);

            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);

            GLfloat fLargest;
            glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &fLargest);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, fLargest);

            glBindTexture(GL_TEXTURE_2D, 0);

        }
        //////////////////////////////////////////////

        if (VRSystem)
        {   
            RenderControllerAxes();
            RenderStereoTargets();
            //RenderCompanionWindow();
            
            vr::Texture_t leftEyeTexture = { (void*)(uintptr_t)leftEyeDesc.m_nResolveTextureId, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
            vr::VRCompositor()->Submit(vr::Eye_Left, &leftEyeTexture);
            vr::Texture_t rightEyeTexture = { (void*)(uintptr_t)rightEyeDesc.m_nResolveTextureId, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
            vr::VRCompositor()->Submit(vr::Eye_Right, &rightEyeTexture);
            // end = clock();
            // double time = (double)(end - start)/ CLOCKS_PER_SEC;
            // std::cout << "real real time : " << time << std::endl;
        }

        if (m_bVblank && m_bGlFinishHack)
        {
            //$ HACKHACK. From gpuview profiling, it looks like there is a bug where two renders and a present
            // happen right before and after the vsync causing all kinds of jittering issues. This glFinish()
            // appears to clear that up. Temporary fix while I try to get nvidia to investigate this problem.
            // 1/29/2014 mikesart
            glFinish();
        }

        // SwapWindow
        {
            SDL_GL_SwapWindow(m_pCompanionWindow);
        }

        // Clear
        {
            // We want to make sure the glFinish waits for the entire present to complete, not just the submission
            // of the command. So, we do a clear here right here so the glFinish will wait fully for the swap.
            glClearColor(0, 0, 0, 1);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        }

        // Flush and wait for swap.
        if (m_bVblank)
        {
            glFlush();
            glFinish();
        }

        // Spew out the controller and pose count whenever they change.
        if (m_iTrackedControllerCount != m_iTrackedControllerCount_Last || m_iValidPoseCount != m_iValidPoseCount_Last)
        {
            m_iValidPoseCount_Last = m_iValidPoseCount;
            m_iTrackedControllerCount_Last = m_iTrackedControllerCount;

            printf("PoseCount:%d(%s) Controllers:%d\n", m_iValidPoseCount, m_strPoseClasses.c_str(), m_iTrackedControllerCount);
        }

        UpdateHMDMatrixPose();

        render_mutex.unlock();  
    }

}

void HMD::UpdateHMDMatrixPose()
{
    if (!VRSystem)
        return;

    vr::VRCompositor()->WaitGetPoses(m_rTrackedDevicePose, vr::k_unMaxTrackedDeviceCount, NULL, 0);

    m_iValidPoseCount = 0;
    m_strPoseClasses = "";
    for (int nDevice = 0; nDevice < vr::k_unMaxTrackedDeviceCount; ++nDevice)
    {
        if (m_rTrackedDevicePose[nDevice].bPoseIsValid)
        {
            m_iValidPoseCount++;
            //if(VRSystem->GetTrackedDeviceClass(nDevice) == vr::TrackedDeviceClass_HMD)
                //  std::cout << m_rTrackedDevicePose[nDevice].mDeviceToAbsoluteTracking.m[2][3] << std::endl;

            m_rmat4DevicePose[nDevice] = ConvertSteamVRMatrixToMatrix4(m_rTrackedDevicePose[nDevice].mDeviceToAbsoluteTracking);
            if (m_rDevClassChar[nDevice] == 0)
            {
                switch (VRSystem->GetTrackedDeviceClass(nDevice))
                {
                case vr::TrackedDeviceClass_Controller:        m_rDevClassChar[nDevice] = 'C'; break;
                case vr::TrackedDeviceClass_HMD:               m_rDevClassChar[nDevice] = 'H'; break;
                case vr::TrackedDeviceClass_Invalid:           m_rDevClassChar[nDevice] = 'I'; break;
                case vr::TrackedDeviceClass_GenericTracker:    m_rDevClassChar[nDevice] = 'G'; break;
                case vr::TrackedDeviceClass_TrackingReference: m_rDevClassChar[nDevice] = 'T'; break;
                default:                                       m_rDevClassChar[nDevice] = '?'; break;
                }
            }
            m_strPoseClasses += m_rDevClassChar[nDevice];
        }
    }

    if (m_rTrackedDevicePose[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid)
    {
        m_mat4HMDPose = m_rmat4DevicePose[vr::k_unTrackedDeviceIndex_Hmd];
        //std::cout << m_mat4HMDPose << std::endl;

        m_mat4HMDPose.invert();
    }
}



// Purpose: Draw all of the controllers as X/Y/Z lines
//-----------------------------------------------------------------------------
void HMD::RenderControllerAxes()
{
    // Don't attempt to update controllers if input is not available
    if (!VRSystem->IsInputAvailable())
        return;

    std::vector<float> vertdataarray;

    m_uiControllerVertcount = 0;
    m_iTrackedControllerCount = 0;

    for (EHand eHand = Left; eHand <= Right; ((int&)eHand)++)
    {
        if (!m_rHand[eHand].m_bShowController)
            continue;

        const Matrix4& mat = m_rHand[eHand].m_rmat4Pose;

        Vector4 center = mat * Vector4(0, 0, 0, 1);

        for (int i = 0; i < 3; ++i)
        {
            Vector3 color(0, 0, 0);
            Vector4 point(0, 0, 0, 1);
            point[i] += 0.05f;  // offset in X, Y, Z
            color[i] = 1.0;  // R, G, B
            point = mat * point;
            vertdataarray.push_back(center.x);
            vertdataarray.push_back(center.y);
            vertdataarray.push_back(center.z);

            vertdataarray.push_back(color.x);
            vertdataarray.push_back(color.y);
            vertdataarray.push_back(color.z);

            vertdataarray.push_back(point.x);
            vertdataarray.push_back(point.y);
            vertdataarray.push_back(point.z);

            vertdataarray.push_back(color.x);
            vertdataarray.push_back(color.y);
            vertdataarray.push_back(color.z);

            m_uiControllerVertcount += 2;
        }

        Vector4 start = mat * Vector4(0, 0, -0.02f, 1);
        Vector4 end = mat * Vector4(0, 0, -39.f, 1);
        Vector3 color(.92f, .92f, .71f);

        vertdataarray.push_back(start.x); vertdataarray.push_back(start.y); vertdataarray.push_back(start.z);
        vertdataarray.push_back(color.x); vertdataarray.push_back(color.y); vertdataarray.push_back(color.z);

        vertdataarray.push_back(end.x); vertdataarray.push_back(end.y); vertdataarray.push_back(end.z);
        vertdataarray.push_back(color.x); vertdataarray.push_back(color.y); vertdataarray.push_back(color.z);
        m_uiControllerVertcount += 2;
    }

    // Setup the VAO the first time through.
    if (m_unControllerVAO == 0)
    {
        glGenVertexArrays(1, &m_unControllerVAO);
        glBindVertexArray(m_unControllerVAO);

        glGenBuffers(1, &m_glControllerVertBuffer);
        glBindBuffer(GL_ARRAY_BUFFER, m_glControllerVertBuffer);

        GLuint stride = 2 * 3 * sizeof(float);
        uintptr_t offset = 0;

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (const void*)offset);

        offset += sizeof(Vector3);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, (const void*)offset);

        glBindVertexArray(0);
    }

    glBindBuffer(GL_ARRAY_BUFFER, m_glControllerVertBuffer);

    // set vertex data if we have some
    if (vertdataarray.size() > 0)
    {
        //$ TODO: Use glBufferSubData for this...
        glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertdataarray.size(), &vertdataarray[0], GL_STREAM_DRAW);
    }
}

Matrix4 HMD::GetCurrentViewProjectionMatrix(vr::Hmd_Eye nEye)
{
    Matrix4 matMVP;
    Matrix4 matTransform;
    
    //matTransform.translate(-3.5, 0.0, 0.0);
    //std::cout << m_mat4eyePosLeft << std::endl;
    //std::cout << m_mat4HMDPose << std::endl << std::endl;
    
    if (nEye == vr::Eye_Left)
    {
        matMVP = m_mat4ProjectionLeft * m_mat4eyePosLeft * m_mat4HMDPose;
    }
    else if (nEye == vr::Eye_Right)
    {
        matMVP = m_mat4ProjectionRight * m_mat4eyePosRight * m_mat4HMDPose;
    }
    //matMVP = m_mat4ProjectionLeft * m_mat4eyePosLeft * m_mat4HMDPose;
    return matMVP;
}



/* Render scene respect to each eye */
void HMD::RenderScene1(vr::Hmd_Eye nEye)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    if (m_bShowCubes)
    {
        glUseProgram(m_unSceneProgramID);
        glUniformMatrix4fv(m_nSceneMatrixLocation, 1, GL_FALSE, GetCurrentViewProjectionMatrix(nEye).get());
        // 7 -> 6로 변경. 
        ///여기 지우면 안나옴.
        for (int i = 0; i < 6; i++) {
            glBindVertexArray(m_unSceneVAO[i]);
            glBindTexture(GL_TEXTURE_2D, m_Texture1[i]);
            glDrawArrays(GL_TRIANGLES, 0, m_uiVertcount[i]);
            glBindVertexArray(0);
        }
        ///
    }

    bool bIsInputAvailable = VRSystem->IsInputAvailable();

    if (bIsInputAvailable)
    {
        // draw the controller axis lines
        glUseProgram(m_unControllerTransformProgramID);
        glUniformMatrix4fv(m_nControllerMatrixLocation, 1, GL_FALSE, GetCurrentViewProjectionMatrix(nEye).get());
        glBindVertexArray(m_unControllerVAO);
        glDrawArrays(GL_LINES, 0, m_uiControllerVertcount);
        glBindVertexArray(0);
    }

    // ----- Render Model rendering -----
    glUseProgram(m_unRenderModelProgramID);

    // for (EHand eHand = Left; eHand <= Right; ((int&)eHand)++)
    // {
    //     if (!m_rHand[eHand].m_bShowController || !m_rHand[eHand].m_pRenderModel)
    //         continue;

    //     const Matrix4& matDeviceToTracking = m_rHand[eHand].m_rmat4Pose;
    //     Matrix4 matMVP = GetCurrentViewProjectionMatrix(nEye) * matDeviceToTracking;
    //     glUniformMatrix4fv(m_nRenderModelMatrixLocation, 1, GL_FALSE, matMVP.get());

    //     m_rHand[eHand].m_pRenderModel->Draw();
    // }
    // 의미 없음.
    glUseProgram(0);
}

//////////////////////////////////////right//////////
void HMD::RenderScene2(vr::Hmd_Eye nEye)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    
    if (m_bShowCubes)
    {
        glUseProgram(m_unSceneProgramID);
        glUniformMatrix4fv(m_nSceneMatrixLocation, 1, GL_FALSE, GetCurrentViewProjectionMatrix(nEye).get());
        
        ///여기 지우면 안나옴.
        for (int i = 0; i < 6; i++) {
            glBindVertexArray(m_unSceneVAO[i]);
            glBindTexture(GL_TEXTURE_2D, m_Texture2[i]);
            glDrawArrays(GL_TRIANGLES, 0, m_uiVertcount[i]);
            glBindVertexArray(0);
        }
        ///
    }

    bool bIsInputAvailable = VRSystem->IsInputAvailable();

    if (bIsInputAvailable)
    {
        // draw the controller axis lines
        glUseProgram(m_unControllerTransformProgramID);
        glUniformMatrix4fv(m_nControllerMatrixLocation, 1, GL_FALSE, GetCurrentViewProjectionMatrix(nEye).get());
        glBindVertexArray(m_unControllerVAO);
        glDrawArrays(GL_LINES, 0, m_uiControllerVertcount);
        glBindVertexArray(0);
    }

    // ----- Render Model rendering -----
    glUseProgram(m_unRenderModelProgramID);

    // for (EHand eHand = Left; eHand <= Right; ((int&)eHand)++)
    // {
    //     if (!m_rHand[eHand].m_bShowController || !m_rHand[eHand].m_pRenderModel)
    //         continue;

    //     const Matrix4& matDeviceToTracking = m_rHand[eHand].m_rmat4Pose;
    //     Matrix4 matMVP = GetCurrentViewProjectionMatrix(nEye) * matDeviceToTracking;
    //     glUniformMatrix4fv(m_nRenderModelMatrixLocation, 1, GL_FALSE, matMVP.get());

    //     m_rHand[eHand].m_pRenderModel->Draw();
    // }
    //의미 없음.
    glUseProgram(0);
}
/////////////////////////////////////////////////



void HMD::RenderStereoTargets()
{
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

    glEnable(GL_MULTISAMPLE);
    
    // Left Eye
    glBindFramebuffer(GL_FRAMEBUFFER, leftEyeDesc.m_nRenderFramebufferId);
    glViewport(0, 0, m_nRenderWidth, m_nRenderHeight);
    RenderScene1(vr::Eye_Left);   //없으면 왼쪽눈 render 안됨//
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    glDisable(GL_MULTISAMPLE);

    glBindFramebuffer(GL_READ_FRAMEBUFFER, leftEyeDesc.m_nRenderFramebufferId);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, leftEyeDesc.m_nResolveFramebufferId);

    glBlitFramebuffer(0, 0, m_nRenderWidth, m_nRenderHeight, 0, 0, m_nRenderWidth, m_nRenderHeight,
        GL_COLOR_BUFFER_BIT,
        GL_LINEAR);

    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);



    glEnable(GL_MULTISAMPLE);

    // Right Eye
    glBindFramebuffer(GL_FRAMEBUFFER, rightEyeDesc.m_nRenderFramebufferId);
    glViewport(0, 0, m_nRenderWidth, m_nRenderHeight);
    RenderScene2(vr::Eye_Right);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    glDisable(GL_MULTISAMPLE);

    glBindFramebuffer(GL_READ_FRAMEBUFFER, rightEyeDesc.m_nRenderFramebufferId);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, rightEyeDesc.m_nResolveFramebufferId);

    glBlitFramebuffer(0, 0, m_nRenderWidth, m_nRenderHeight, 0, 0, m_nRenderWidth, m_nRenderHeight,
        GL_COLOR_BUFFER_BIT,
        GL_LINEAR);

    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
}

void HMD::RenderCompanionWindow()
{
    glDisable(GL_DEPTH_TEST);
    glViewport(0, 0, m_nCompanionWindowWidth, m_nCompanionWindowHeight);
    glBindVertexArray(m_unCompanionWindowVAO);
    glUseProgram(m_unCompanionWindowProgramID);
    // render left eye (first half of index array )
    glBindTexture(GL_TEXTURE_2D, leftEyeDesc.m_nResolveTextureId);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glDrawElements(GL_TRIANGLES, m_uiCompanionWindowIndexSize / 2, GL_UNSIGNED_SHORT, 0);

    // render right eye (second half of index array )
    glBindTexture(GL_TEXTURE_2D, rightEyeDesc.m_nResolveTextureId);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glDrawElements(GL_TRIANGLES, m_uiCompanionWindowIndexSize / 2, GL_UNSIGNED_SHORT, (const void*)(uintptr_t)(m_uiCompanionWindowIndexSize));

    glBindVertexArray(0);
    glUseProgram(0);
}





std::string HMD::faceNameToString(CubeFaceName faceName) {
    switch (faceName)
    {
    case CubeFaceName::Front:  return "Front";
    case CubeFaceName::Right:  return "Right";
    case CubeFaceName::Back:   return "Back";
    case CubeFaceName::Left:   return "Left";
    case CubeFaceName::Top:    return "Top";
    case CubeFaceName::Bottom: return "Bottom";
    default:           return "UNKNOWN";
    }
}

int HMD::faceNameToInteger(CubeFaceName faceName) {
    switch (faceName)
    {
    case CubeFaceName::Front:  return 0;
    case CubeFaceName::Right:  return 1;
    case CubeFaceName::Back:   return 2;
    case CubeFaceName::Left:   return 3;
    case CubeFaceName::Top:    return 4;
    case CubeFaceName::Bottom: return 5;
    }
}

void HMD::createCubeMapFace(const cv::Mat& in, cv::Mat& face, CubeFaceName faceName, int stereo) {
    // we have to enforce input image dimensions to be equirectangular
    if (in.size().height != in.size().width / 2)
    {
        return ;
    }
    MapCoord* MAP_COORDS;
    if(stereo == 0)   MAP_COORDS = &LEFT_MAP_COORDS[0];
    else MAP_COORDS = &RIGHT_MAP_COORDS[0];

    const int faceId = (int)faceName;

    const float inWidth = in.cols;
    const float inHeight = in.rows;


    // calculate adjacent (ak) and opposite (an) of the
    // triangle that is spanned from the sphere center 
    // to our cube face
    const float an = sin(M_PI / 4);
    const float ak = cos(M_PI / 4);

    const float ftu = facesTable[faceId].polarCoords[0];
    const float ftv = facesTable[faceId].polarCoords[1];
    int index = faceNameToInteger(faceName);

    // for each point in the target image, 
    // calculate the corresponding source coordinates
    if (!MAP_COORDS[index].isSet) {
        MAP_COORDS[index].mapx = new cv::Mat(targetDim, targetDim, CV_32F);
        MAP_COORDS[index].mapy = new cv::Mat(targetDim, targetDim, CV_32F);
        std::cout << "Create mapping arrays..." << std::endl;
        for (int y = 0; y < targetDim; y++)
        {
            for (int x = 0; x < targetDim; x++)
            {
                // map face pixel coordinates to [-1, 1] on plane
                float nx = (float)y / (float)targetDim - 0.5f;
                float ny = (float)x / (float)targetDim - 0.5f;

                nx *= 2;
                ny *= 2;

                // map [-1, 1] plane coords to [-an, an]
                // thats the coordinates in respect to a unit sphere 
                // that contains our box
                nx *= an;
                ny *= an;

                float u, v;

                // project from plane to sphere surface
                if (ftv == 0)
                {
                    // center faces
                    u = atan2(nx, ak);
                    v = atan2(ny * cos(u), ak);
                    u += ftu;
                }
                else if (ftv > 0)
                {
                    // bottom face 
                    float d = sqrt(nx * nx + ny * ny);
                    v = M_PI / 2 - atan2(d, ak);
                    u = atan2(ny, nx);
                }
                else
                {
                    // top face
                    float d = sqrt(nx * nx + ny * ny);
                    v = -M_PI / 2 + atan2(d, ak);
                    u = atan2(-ny, nx);
                }

                // map from angular coordinates to [-1, 1], respectively
                u = u / (M_PI);
                v = v / (M_PI / 2);

                // warp around, if our coordinates are out of bounds
                while (v < -1)
                {
                    v += 2;
                    u += 1;
                }

                while (v > 1)
                {
                    v -= 2;
                    u += 1;
                }

                while (u < -1)
                {
                    u += 2;
                }

                while (u > 1)
                {
                    u -= 2;
                }

                // map from [-1, 1] to in texture space
                u = u / 2.0f + 0.5f;
                v = v / 2.0f + 0.5f;

                u = u * (inWidth - 1);
                v = v * (inHeight - 1);

                // save the result for this pixel in map
                MAP_COORDS[index].mapx[0].at<float>(x, y) = u;
                MAP_COORDS[index].mapy[0].at<float>(x, y) = v;
            }
        }
        MAP_COORDS[index].isSet = true;
        std::cout << "Transformation calculation between equirectangular & cubemap done" << std::endl;
    }

    // recreate output image if it has wrong size or type
    if (face.cols != targetDim || face.rows != targetDim || face.type() != in.type())
    {
        face = cv::Mat(targetDim, targetDim, in.type());
    }

    // run actual resampling using OpenCV's remap
    cv::remap(in, face, MAP_COORDS[index].mapx[0], MAP_COORDS[index].mapy[0], cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

    return ;
}







/* Check HMD, Controllers connection state */
void HMD::checkConnection() {
    std::cout << "Maximum Number of Device that can be tracked: " << vr::k_unMaxTrackedDeviceCount << std::endl;
    std::cout << "Number of tracker to find: " << trackerNum << std::endl;
    std::cout << "Please Connect Your HMD and two controllers to start this program" << std::endl;

    while (true) {
        int HMD_count = 0;
        int controller_count = 0;
        int tracker_count = 0;
        for (int i = 0; i < vr::k_unMaxTrackedDeviceCount; i++){
            vr::ETrackedDeviceClass trackedStatus = VRSystem->GetTrackedDeviceClass(i);
            switch (int(trackedStatus)) {
            case 0:  continue;
            case vr::ETrackedDeviceClass::TrackedDeviceClass_HMD:{
                this->HMD_INDEX = i;
                HMD_count += 1;
                continue;
            }
            case vr::ETrackedDeviceClass::TrackedDeviceClass_Controller: {
                std::cout << "controller:   " << i << std::endl;
                vr::ETrackedControllerRole controllerRole = VRSystem->GetControllerRoleForTrackedDeviceIndex(vr::TrackedDeviceIndex_t(i));
                if (controllerRole == 1) {
                    std::cout << "Left controller identified! Left controller idx: " << i << std::endl;
                    this->LEFT_CONTROLLER_INDEX = i;
                }
                else if (controllerRole == 2) {
                    std::cout << "Right controller identified! Right controller idx: " << i << std::endl;
                    this->RIGHT_CONTROLLER_INDEX = i;
                }
                controller_count += 1;
                continue;
            }
            case vr::ETrackedDeviceClass::TrackedDeviceClass_GenericTracker:{
                std::cout << "Tracker " << tracker_count <<" identified! Idx is " << i << std::endl;
                vr::ETrackedControllerRole controllerRole = VRSystem->GetControllerRoleForTrackedDeviceIndex(vr::TrackedDeviceIndex_t(i));
                vr::VRSystem()->GetStringTrackedDeviceProperty(i, vr::Prop_SerialNumber_String, serialNumber[tracker_count], sizeof(serialNumber));
                printf("Serial Number = %s \n", serialNumber[tracker_count]);
                this->TRACKER_INDEX[tracker_count] = i;
                tracker_count += 1;
                continue;
            }
            case 4:  continue;
            case 5:  continue;
            }
        }

        // Only use HMD
        if (!checkControllers && !checkTrackers && HMD_count == 1) {
            break;
        }
        // Use HMD+controllers
        if (checkControllers && !checkTrackers && (HMD_count == 1 && controller_count == 2)) {
            break;
        }
        // Use HMD+trackers
        if (!checkControllers && checkTrackers && (HMD_count == 1 && tracker_count == trackerNum)) {
            break;
        }
        // Use HMD++controllers+trackers
        if (checkControllers && checkTrackers && (HMD_count == 1 && controller_count == 2 && tracker_count == trackerNum)) {
            break;
        }
    }

    if (!checkControllers && !checkTrackers)
    {
        std::cout << "HMD is identified..." << std::endl;
        std::cout << "All Specified Connection Identified.. Start VR system.." << std::endl;
    }
    else if(checkControllers && !checkTrackers) {
        std::cout << "One HMD and Two controllers are identified..." << std::endl;
        std::cout << "Start VR system and ROS NODE" << std::endl;
    }
    else if(!checkControllers && checkTrackers) {
        std::cout << "One HMD and " << trackerNum << " trackers are identified..." << std::endl;
        std::cout << "Start VR system and ROS NODE" << std::endl;
    }
    else {
        std::cout << "One HMD, Two controllers and "<<  trackerNum << " trackers are identified..." << std::endl;
        std::cout << "Start VR system and ROS NODE" << std::endl;
    }

}




/* Private members IMPLEMENTATIOn*/
_FLOAT HMD::map2array(Mat eigen) {
    // Return 4x4 Eigen matrix to 3x4 c++ matrix
    _FLOAT array = new float[3][4];

    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 4; col++) {
            array[row][col] = eigen(row, col);
        }
    }
    return array;
}

Mat HMD::map2eigen(float array[][4]) {
    // Return 3x4 c++ matrix to 4x4 Eigen matrix
    Mat eigen;
    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 4; col++) {
            eigen(row, col) = array[row][col];
        }
    }
    eigen(3, 0) = 0;
    eigen(3, 1) = 0;
    eigen(3, 2) = 0;
    eigen(3, 3) = 1;

    return eigen;
}

Mat HMD::coordinate_z(Mat array){

    Mat y_r;
    Mat x_r;
    Mat z_r;

    y_r << 0, 0, -1, 0,
        0, 1, 0, 0,
        1, 0, 0, 0,
        0, 0, 0, 1;
    x_r << 1, 0, 0, 0,
        0, 0, -1, 0,
        0, 1, 0, 0,
        0, 0, 0, 1;

    z_r << -1, 0, 0, 0,
        0, -1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    Mat x_r_array = x_r * y_r * array;

    x_r_array.col(0) = x_r_array.col(0) * -1;
    x_r_array.col(1) = x_r_array.col(1) * -1;

    return  x_r_array;
}

Mat HMD::coordinate_robot(Mat array){

    Mat local_coordinate_rotation;

    local_coordinate_rotation << 0, -1, 0, 0,
                0, 0, 1, 0,
                -1, 0, 0, 0,
                0, 0, 0, 1;
    Mat vr_to_robot;
    vr_to_robot << 0, 0, -1, 0,
                -1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 0, 1;

    return  vr_to_robot* array* local_coordinate_rotation;
}

Mat HMD::coordinate(Mat array) {
    Mat y_r;
    Mat x_r;
    Mat z_r;

    y_r << 0, 0, -1, 0,
        0, 1, 0, 0,
        1, 0, 0, 0,
        0, 0, 0, 1;
    x_r << 1, 0, 0, 0,
        0, 0, -1, 0,
        0, 1, 0, 0,
        0, 0, 0, 1;

    return   x_r * y_r * array;
}

VR::matrix_3_4 HMD::makeTrackingmsg(_FLOAT array) {
    VR::matrix_3_4 msg;

    for (int i = 0; i < 4; i++) msg.firstRow.push_back(array[0][i]);
    for (int i = 0; i < 4; i++) msg.secondRow.push_back(array[1][i]);
    for (int i = 0; i < 4; i++) msg.thirdRow.push_back(array[2][i]);

    return msg;
}
void HMD::rosPublish() {
    VRSystem->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseSeated, 0, m_rTrackedDevicePose, vr::k_unMaxTrackedDeviceCount);

    // HMD : Send only rotation parameters(euler or quarternion)
    // controller : Send rotation & translation parameters(w.r.t current HMD Cordinate)

    HMD_curEig = map2eigen(m_rTrackedDevicePose[HMD_INDEX].mDeviceToAbsoluteTracking.m);
    if (checkControllers) 
    {
        LEFTCONTROLLER_curEig = map2eigen(m_rTrackedDevicePose[LEFT_CONTROLLER_INDEX].mDeviceToAbsoluteTracking.m);
        RIGHTCONTROLLER_curEig = map2eigen(m_rTrackedDevicePose[RIGHT_CONTROLLER_INDEX].mDeviceToAbsoluteTracking.m);
        LEFTCONTROLLER = map2array(coordinate_robot(LEFTCONTROLLER_curEig));
        RIGHTCONTROLLER = map2array(coordinate_robot(RIGHTCONTROLLER_curEig));
    }
    if(checkTrackers){
        for (int i=0; i<trackerNum; i++)
        {
            TRACKER_curEig[i] = map2eigen(m_rTrackedDevicePose[TRACKER_INDEX[i]].mDeviceToAbsoluteTracking.m);
            HMD_TRACKER[i] = map2array(coordinate_robot(TRACKER_curEig[i]));
        }
    }
    if (pubPose) {
        hmd_pub.publish(makeTrackingmsg(map2array(coordinate_robot(HMD_curEig))));
        if (checkControllers) {
            leftCon_pub.publish(makeTrackingmsg(LEFTCONTROLLER));
            rightCon_pub.publish(makeTrackingmsg(RIGHTCONTROLLER));
        }
        if (checkTrackers){
            allTrackersFine = true;
            for (int i=0; i<trackerNum; i++)
            {
                allTrackersFine *= m_rTrackedDevicePose[TRACKER_INDEX[i]].bPoseIsValid;
            }
            allTrackersFineData.data = allTrackersFine;
            tracker_status_pub.publish(allTrackersFineData);
            if (allTrackersFine)
            {
                for (int i=0; i<trackerNum; i++)
                {                
                    if (std::string(serialNumber[i]) == "LHR-B979AA9E")
                        tracker_pub[0].publish(makeTrackingmsg(HMD_TRACKER[i]));
                    if (std::string(serialNumber[i]) == "LHR-3F2A7A7B")
                        tracker_pub[1].publish(makeTrackingmsg(HMD_TRACKER[i]));
                    if (std::string(serialNumber[i]) == "LHR-7330E069")
                        tracker_pub[2].publish(makeTrackingmsg(HMD_TRACKER[i]));
                    if (std::string(serialNumber[i]) == "LHR-8C0A4142")
                        tracker_pub[3].publish(makeTrackingmsg(HMD_TRACKER[i]));
                    if (std::string(serialNumber[i]) == "LHR-3C32FE4B")
                        tracker_pub[4].publish(makeTrackingmsg(HMD_TRACKER[i]));
                    if (std::string(serialNumber[i]) == "LHR-5423DE85")
                        tracker_pub[5].publish(makeTrackingmsg(HMD_TRACKER[i]));
                }
            }
        }
    }
}


  Vector3d HMD::rot2Euler(Matrix3f Rot)
  {
    double beta;
    Eigen::Vector3d angle;
    beta = -asin(Rot(2, 0));
    double DEG2RAD = 3.14/180;
    if (abs(beta) < 90 * DEG2RAD)
      beta = beta;
    else
      beta = 180 * DEG2RAD - beta;

    angle(0) = atan2(Rot(2, 1), Rot(2, 2) + 1E-37); //roll
    angle(2) = atan2(Rot(1, 0), Rot(0, 0) + 1E-37); //pitch
    angle(1) = beta;                                //yaw

    return angle;
  }