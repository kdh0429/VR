#pragma once


#include "tracker.h"
#include <thread>


int main(int argc, char* argv[])
{   
   
    HMD* hmdSystem = new HMD(argc, argv);
    
    hmdSystem->init();
    
    hmdSystem->RunMainLoop();
    
    return 0;
}


