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
{
    this->argc_arg = arc;
    this->argv_arg = arv;
    checkControllers = false;
    checkTrackers = false;
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

    
    checkConnection();                                        

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
        //RenderFrame();
    }

    t.join();
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