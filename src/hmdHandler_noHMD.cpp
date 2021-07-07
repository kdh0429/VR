#include "hmdHandler_noHMD.h"



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

    eError = vr::VRInitError_None;
    VRSystem = vr::VR_Init(&eError, vr::VRApplication_Background);

    std::cout << "Start Connection Check" << std::endl;   
    checkConnection();                                        
    
    hmd_init_bool = true;

}


void HMD::RunMainLoop()
{   
    ros::Rate r(1000);
    while (ros::ok())
    {
        rosPublish();
        r.sleep();
        ros::spinOnce();
    }
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
                    if (std::string(serialNumber[i]) == "LHR-B979AA9E" || std::string(serialNumber[i]) == "LHR-5567029A") // waist
                        tracker_pub[0].publish(makeTrackingmsg(HMD_TRACKER[i]));
                    else if (std::string(serialNumber[i]) == "LHR-3F2A7A7B" || std::string(serialNumber[i]) == "LHR-D74F7D1A")  // chest
                        tracker_pub[1].publish(makeTrackingmsg(HMD_TRACKER[i]));
                    else if (std::string(serialNumber[i]) == "LHR-7330E069" || std::string(serialNumber[i]) == "LHR-78CF9EE8") // left shoulder
                        tracker_pub[2].publish(makeTrackingmsg(HMD_TRACKER[i]));
                    else if (std::string(serialNumber[i]) == "LHR-8C0A4142" || std::string(serialNumber[i]) == "LHR-CA171B68") // left hand
                        tracker_pub[3].publish(makeTrackingmsg(HMD_TRACKER[i]));
                    else if (std::string(serialNumber[i]) == "LHR-3C32FE4B" || std::string(serialNumber[i]) == "LHR-172B3493") // right shoulder
                        tracker_pub[4].publish(makeTrackingmsg(HMD_TRACKER[i]));
                    else if (std::string(serialNumber[i]) == "LHR-5423DE85" || std::string(serialNumber[i]) == "LHR-88A2CD57")  // right hand
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