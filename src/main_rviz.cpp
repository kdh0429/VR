#pragma once

#include "rvizOverlay.h"

using namespace vr;

void check_error(int line, vr::EVRInitError error);


int main(int argc, char* argv[])
{   
    //vr::VREvent_t vrEvent;
    vr::EVRInitError error;
    std::cout << "11" << std::endl;
    VR_Init(&error, vr::VRApplication_Overlay);
    check_error(__LINE__, error);
    std::cout << "22" << std::endl;
    ImageConverter* rvizSystem = new ImageConverter(argc, argv);
    std::cout << "33" << std::endl;
    rvizSystem->init();
    std::cout << ros::ok() << std::endl;
    std::cout << "44" << std::endl;
    rvizSystem->runMainLoop();
    
    return 0;
}

void check_error(int line, vr::EVRInitError error)
{
    if (error != 0) printf("%d: error %s\n", line, VR_GetVRInitErrorAsSymbol(error));
}