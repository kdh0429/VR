#include "render.h"

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

enum class CubeFaceName {
	Front,
	Right,
	Back,
	Left,
	Top,
	Bottom,

	NumFaces
};
struct MapCoord {
	cv::Mat* mapx;
	cv::Mat* mapy;
	bool isSet = false;
};

struct Face {
	CubeFaceName name;
	float polarCoords[2];
};
static const Face facesTable[6] = {
	 { CubeFaceName::Front,  {        0.f,        0.f } },
		{ CubeFaceName::Right,  { +M_HALF_PI,        0.f } },
		{ CubeFaceName::Back,   {      +M_PI,        0.f } },
		{ CubeFaceName::Left,   { -M_HALF_PI,        0.f } },
		{ CubeFaceName::Top,    {        0.f, -M_HALF_PI } },
		{ CubeFaceName::Bottom, {        0.f, +M_HALF_PI } },
};








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

	bool checkControllers = false;
	bool checkTrackers = false;
	static const uint32_t trackerNum = 2;
	bool pubPose = false;

	ros::Publisher hmd_pub, leftCon_pub, rightCon_pub, tracker_pub[trackerNum];

	//vr is  right-handed system
   // +y is up
   // +x is to the right
   // -z is forward
   // Distance unit is  meters

	_FLOAT map2array(Mat eigen);
	Mat map2eigen(float array[][4]);
	Mat coordinate_z(Mat array);
	Mat coordinate(Mat array);
	VR::matrix_3_4 makeTrackingmsg(_FLOAT array);

	Mat refMat, refMatInv;
	Mat HMD_curEig;
	_FLOAT HMD_world;
	_FLOAT HMD_world_coord_change;
	Mat LEFTCONTROLLER_curEig;
	Mat RIGHTCONTROLLER_curEig;
	Mat TRACKER_curEig[trackerNum];
	_FLOAT HMD_LEFTCONTROLLER;
	_FLOAT HMD_RIGHTCONTROLLER;
	_FLOAT HMD_TRACKER[trackerNum];
	Mat HMD_worldEigInv;

/* streaming members */
private:

	ricohRos streamObj;
	ros::Subscriber ricoh_sub;
public:
	int width = -1;
	int height = -1;
	AVFrame leftDat, rightDat;
	cv::Mat leftCvEquirect, rightCvEquirect; // Equirectangular Image from ricoh theta camera
	enum AVPixelFormat left_Pixfmt, right_Pixfmt; // ricoh theta pixel format
	bool streamParamInit = false;
	
	
	std::string faceNameToString(CubeFaceName faceName);
	int faceNameToInteger(CubeFaceName faceName);
	bool createCubeMapFace(const cv::Mat& in, cv::Mat& face, CubeFaceName faceName, int stereo);
	cv::Mat LeftcubeFront, LeftcubeBack, LeftcubeLeft, LeftcubeRight, LeftcubeTop, LeftcubeBottom;
	cv::Mat RightcubeFront, RightcubeBack, RightcubeLeft, RightcubeRight, RightcubeTop, RightcubeBottom;

	int targetDim = 512;
	MapCoord LEFT_MAP_COORDS[CubeFaceName::NumFaces];
	MapCoord RIGHT_MAP_COORDS[CubeFaceName::NumFaces];

/* rviz stream */
private:
	ros::Subscriber rviz_sub;
public:
	cv::Mat RvizScreen;
	int Rviz_targetDim = 256;


/* vr component members */
public:

	vr::TrackedDevicePose_t m_rTrackedDevicePose[vr::k_unMaxTrackedDeviceCount];
	Matrix4 m_rmat4DevicePose[vr::k_unMaxTrackedDeviceCount];

	vr::TrackedDeviceIndex_t HMD_INDEX, LEFT_CONTROLLER_INDEX, RIGHT_CONTROLLER_INDEX, TRACKER_INDEX[trackerNum];
	vr::EVRInitError eError;
	vr::IVRSystem* VRSystem;
	vr::COpenVRContext vrContext;
	vr::IVRCompositor* vrCompositor; 
	
	
	void checkConnection();
	bool BInitCompositor();
	void ShutDown();

	void RunMainLoop();
	bool HandleInput();
	void ProcessVREvent(const vr::VREvent_t& event);
	

/* vr Rendering members (OpenGL + OpenVR) */
	bool BInitGL();
	void RenderFrame();
	void RenderControllerAxes();

	bool SetupTexturemaps();

	void SetupScene();
	void AddCubeToScene(Matrix4 mat, std::vector<float>& vertdata, int flag);
	void AddCubeVertex(float fl0, float fl1, float fl2, float fl3, float fl4, std::vector<float>& vertdata);
	void AddScreenToScene(Matrix4 mat, std::vector<float>& vertdata);

	bool SetupStereoRenderTargets();
	void SetupCompanionWindow();
	void SetupCameras();
	void RenderScene(vr::Hmd_Eye nEye);
	void RenderStereoTargets();

	void RenderCompanionWindow();
	

	Matrix4 GetHMDMatrixProjectionEye(vr::Hmd_Eye nEye);
	Matrix4 GetHMDMatrixPoseEye(vr::Hmd_Eye nEye);
	Matrix4 GetCurrentViewProjectionMatrix(vr::Hmd_Eye nEye);
	void UpdateHMDMatrixPose();

	Matrix4 ConvertSteamVRMatrixToMatrix4(const vr::HmdMatrix34_t& matPose);
	
	GLuint CompileGLShader(const char* pchShaderName, const char* pchVertexShader, const char* pchFragmentShader);
	bool CreateAllShaders();

	RenderModel* FindOrLoadRenderModel(const char* pchRenderModelName);

private:
	bool m_bDebugOpenGL;
	bool m_bVerbose;
	bool m_bPerf;
	bool m_bVblank;
	bool m_bGlFinishHack;

	std::string m_strDriver;
	std::string m_strDisplay;

	struct ControllerInfo_t {
		vr::VRInputValueHandle_t m_source = vr::k_ulInvalidInputValueHandle;
		vr::VRActionHandle_t m_actionPose = vr::k_ulInvalidActionHandle;
		vr::VRActionHandle_t m_actionHaptic = vr::k_ulInvalidActionHandle;
		Matrix4 m_rmat4Pose;
		RenderModel* m_pRenderModel = nullptr;
		std::string m_sRenderModelName;
		bool m_bShowController;
	};

	enum EHand {
		Left = 0,
		Right = 1
	};
	ControllerInfo_t m_rHand[2];


private: // SDL book keeping
	SDL_Window* m_pCompanionWindow;
	uint32_t m_nCompanionWindowWidth;
	uint32_t m_nCompanionWindowHeight;

	SDL_GLContext m_pContext;

public: // OpenGL book keeping
	int m_iTrackedControllerCount;
	int m_iTrackedControllerCount_Last;
	int m_iValidPoseCount;
	int m_iValidPoseCount_Last;
	bool m_bShowCubes;
	Vector2 m_vAnalogValue;

	std::string m_strPoseClasses;
	char m_rDevClassChar[vr::k_unMaxTrackedDeviceCount];
	
	int m_iSceneVolumeWidth;
	int m_iSceneVolumeHeight;
	int m_iSceneVolumeDepth;

	float m_fScaleSpacing;
	float m_fScale;

	int m_iSceneVolumeInit;

	float m_fNearClip;
	float m_fFarClip;

	GLuint m_Texture[7];
	

	unsigned int m_uiVertcount[7];
	

	GLuint m_glSceneVertBuffer[7];

	GLuint m_unSceneVAO[7];



	GLuint m_unCompanionWindowVAO;
	GLuint m_glCompanionWindowIDVertBuffer;
	GLuint m_glCompanionWindowIDIndexBuffer;
	unsigned int m_uiCompanionWindowIndexSize;

	GLuint m_glControllerVertBuffer;
	GLuint m_unControllerVAO;
	unsigned int m_uiControllerVertcount;

	Matrix4 m_mat4HMDPose;
	Matrix4 m_mat4eyePosLeft;
	Matrix4 m_mat4eyePosRight;

	Matrix4 m_mat4ProjectionCenter;
	Matrix4 m_mat4ProjectionLeft;
	Matrix4 m_mat4ProjectionRight;

	struct VertexDataScene {
		Vector3 position;
		Vector2 texCoord;

	};

	struct VertexDataWindow {
		Vector2 position;
		Vector2 texCoord;
		VertexDataWindow(const Vector2& pos, const Vector2 tex) : position(pos), texCoord(tex) {}
	};
	GLuint m_unSceneProgramID;
	GLuint m_unCompanionWindowProgramID;
	GLuint m_unControllerTransformProgramID;
	GLuint m_unRenderModelProgramID;

	GLuint m_nSceneMatrixLocation;
	GLuint m_nControllerMatrixLocation;
	GLuint m_nRenderModelMatrixLocation;

	struct FramebufferDesc {
		GLuint m_nDepthBufferId;
		GLuint m_nRenderTextureId;
		GLuint m_nRenderFramebufferId;
		GLuint m_nResolveTextureId;
		GLuint m_nResolveFramebufferId;
	};

	FramebufferDesc leftEyeDesc;
	FramebufferDesc rightEyeDesc;

	bool CreateFrameBuffer(int nWidth, int nHeight, FramebufferDesc& framebufferDesc);

	uint32_t m_nRenderWidth;
	uint32_t m_nRenderHeight;

	std::vector<RenderModel*> m_vecRenderModels;

	vr::VRActionHandle_t m_actionHideCubes = vr::k_ulInvalidActionHandle;
	vr::VRActionHandle_t m_actionHideThisController = vr::k_ulInvalidActionHandle;
	vr::VRActionHandle_t m_actionTriggerHaptic = vr::k_ulInvalidActionHandle;
	vr::VRActionHandle_t m_actionAnalongInput = vr::k_ulInvalidActionHandle;

	vr::VRActionSetHandle_t m_actionsetDemo = vr::k_ulInvalidActionSetHandle;


};





#endif 





































