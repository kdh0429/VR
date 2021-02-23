#include <stdio.h>
#include <openvr.h>
#include <iostream>


using namespace vr;

VROverlayHandle_t overlayHandle;
vr::VREvent_t vrEvent;
EVRInitError error;

vr::HmdMatrix34_t transform = {								//overlay position
	1.0f, 0.0f, 0.0f, 0.12f,								//+: upper axis
	0.0f, 1.0f, 0.0f, 0.08f,								//+: right horizontal axis
	0.0f, 0.0f, 1.0f, -0.3f									//-: forward axis
};

vr::HmdMatrix34_t transform2 = {								//overlay position
	1.0f, 0.0f, 0.0f, 0.12f,								//+: upper axis
	0.0f, 1.0f, 0.0f, -0.08f,								//+: right horizontal axis
	0.0f, 0.0f, 1.0f, -0.3f									//-: forward axis
};

void check_error(int line, EVRInitError error) { if (error != 0) printf("%d: error %s\n", line, VR_GetVRInitErrorAsSymbol(error)); }

void uploadImage(){
	VROverlay()->SetOverlayFromFile(overlayHandle, "C:/Users/Dyros/Desktop/avatar/src/rvizView/rvizView/bin/win64/image/yong.jpg");  
	VROverlay()->SetOverlayAlpha(overlayHandle, 0.5);					//opacity
	VROverlay()->SetOverlayWidthInMeters(overlayHandle, 0.2f);			//overlay size
	VROverlay()->SetOverlayTransformTrackedDeviceRelative(overlayHandle, vr::k_unTrackedDeviceIndex_Hmd, &transform);
	VROverlay()->ShowOverlay(overlayHandle);
}
void uploadImage2(){
	VROverlay()->SetOverlayFromFile(overlayHandle, "C:/Users/Dyros/Desktop/avatar/src/rvizView/rvizView/bin/win64/image/panorama.png");  
	VROverlay()->SetOverlayAlpha(overlayHandle, 0.5);					//opacity
	VROverlay()->SetOverlayWidthInMeters(overlayHandle, 0.2f);			//overlay size
	VROverlay()->SetOverlayTransformTrackedDeviceRelative(overlayHandle, vr::k_unTrackedDeviceIndex_Hmd, &transform2);
	VROverlay()->ShowOverlay(overlayHandle);
}

int main(int argc, char **argv) { (void) argc; (void) argv;
	
	VR_Init(&error, vr::VRApplication_Overlay);
	check_error(__LINE__, error);
	

	VROverlay()->CreateOverlay ("image", "rvizView", &overlayHandle); /* key has to be unique, name doesn't matter */
	while(true){
		uploadImage();
		uploadImage2();
	}	
	

	
	return 0;
}
