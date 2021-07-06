#pragma once


#include "hmdHandler_noHMD.h"
#include <thread>


int main(int argc, char* argv[])
{   
   
    HMD* hmdSystem = new HMD(argc, argv);
    
    hmdSystem->init();
    
    hmdSystem->RunMainLoop();
    
    return 0;
}


