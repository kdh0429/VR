#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <mutex>
#include <thread>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include "tocabi_msgs/matrix_3_4.h"
#include <openvr.h>

#ifndef hmdHandler_h
#define hmdHandler_h

// Initiate VR system as a VRApplication_Scene mode
// VRApplication_Scene mode : A 3D Application that will draw an environment.
// IVRSystem : provides primary data such as display configuration data, tracking data, distortion state, controller states, device properties



// IVRSystem uses "tracked Device Index" to identify a "specified device" that attached to current computer. 
// typedef uint32_t TrackedDeviceIndex_t 
// static const uint32_t k_unTrackedDeviceIndexInvalid = 0xFFFFFFFF 
// static const uint32_t k_unMaxTrackedDeviceCount = 64 (Max device kinds..)
// static const uint32_t k_unTrackedDeviceIndex_Hmd = 0  (HMD Headset Device Index)

// TrackedDeviceClass_Invalid - There is no device at this index
// TrackedDeviceClass_HMD - The device at this index is an HMD
// TrackedDeviceClass_Controller - The device is a controller


// Functions, Knowledge to get Absolute Pose of HMd Device, Controller

// eOrigin - Tracking universe that returned poses should be relative to (one of this : TrackingUniverseSeated, TrackingUniverseStanding, TrackingUniverseRaw)
// float fPredictedSecondsToPhotonsFromNow - Number of seconds from now to predict poses for.
// set fPredictedSecondsToPhotonsFromNow to "0"  if you want to know INSTANT pose of HMD or Controller

using namespace Eigen;

typedef Matrix<float, 4, 4> Mat;
typedef float(*_FLOAT)[4];

class HMD {

public: 
	// Constructor
	HMD(int arc, char* arv[]);
	
	// Destructor
	~HMD();

	// Initializer
	void init();

/* HMD, Controllers members */
public:
	void rosPublish();

public:
	int argc_arg;
	char** argv_arg;

	bool checkHMD = false;
	bool checkControllers = false;
	bool checkTrackers = true;
	bool allTrackersFine = true;
	std_msgs::Bool allTrackersFineData;
	static const uint32_t trackerNum = 6;
	char serialNumber[trackerNum][15];
	bool pubPose = true;
	int loop_tick_ = 0;
	ros::Publisher hmd_pub, leftCon_pub, rightCon_pub, tracker_pub[trackerNum], tracker_status_pub;

	//vr is  right-handed system
   // +y is up
   // +x is to the right
   // -z is forward
   // Distance unit is  meters

	_FLOAT map2array(Mat eigen);
	Mat map2eigen(float array[][4]);
	Mat coordinate_z(Mat array);
	Mat coordinate_robot(Mat array);
	Mat coordinate(Mat array);
	Vector3d rot2Euler(Matrix3f Rot);
	tocabi_msgs::matrix_3_4 makeTrackingmsg(_FLOAT array);

	Mat refMat, refMatInv;
	Mat HMD_curEig;
	Mat HMD_init;
	bool hmd_init=false;
	bool hmd_init_bool = false;
	double yaw_angle;
	_FLOAT HMD_world;
	_FLOAT HMD_world_coord_change;
	Mat LEFTCONTROLLER_curEig;
	Mat RIGHTCONTROLLER_curEig;
	Mat TRACKER_curEig[trackerNum];
	_FLOAT LEFTCONTROLLER;
	_FLOAT RIGHTCONTROLLER;
	_FLOAT HMD_TRACKER[trackerNum];
	Mat HMD_worldEigInv;


/* vr component members */
public:

	vr::TrackedDevicePose_t m_rTrackedDevicePose[vr::k_unMaxTrackedDeviceCount];
	vr::TrackedDeviceIndex_t HMD_INDEX, LEFT_CONTROLLER_INDEX, RIGHT_CONTROLLER_INDEX, TRACKER_INDEX[trackerNum];
	vr::EVRInitError eError;
	vr::IVRSystem* VRSystem;

	void checkConnection();
	void RunMainLoop();

	char m_rDevClassChar[vr::k_unMaxTrackedDeviceCount];
	
};





#endif 





































