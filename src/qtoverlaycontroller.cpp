//====== Copyright Valve Corporation, All rights reserved. =======


#include "qtoverlaycontroller.h"


#include <QOpenGLFramebufferObjectFormat>
#include <QOpenGLPaintDevice>
#include <QPainter>
#include <QtWidgets/QWidget>
#include <QMouseEvent>
#include <QtWidgets/QGraphicsSceneMouseEvent>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGraphicsEllipseItem>
#include <QCursor>

#include "ui_widget.h"
#include <speechapi_cxx.h>
using namespace Microsoft::CognitiveServices::Speech;
using namespace vr;

vr::HmdMatrix34_t transform = {								//overlay position
	1.0f, 0.0f, 0.0f, 0.2f,									//+: right horizontal axis
	0.0f, 1.0f, 0.0f, 0.0f,									//+: upper axis
	0.0f, 0.0f, 1.0f, -0.4f									//-: forward axis
};

OverlayWidget::OverlayWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::OverlayWidget)
{
	
    ui->setupUi(this);
    timer = new QTimer(this);

	std::cout << "22" << std::endl;

    // ros::init(argc, argv, "image_rviz");
    // ros::NodeHandle nh;
    // cv::namedWindow("rviz_image");

    // image_transport::ImageTransport it(nh);
    // image_transport::Subscriber sub = it.subscribe("/rviz1/camera1/image", 1,
    //     &OverlayWidget::update_rviz, this);
    

    // connect(timer, SIGNAL(timeout()), this, SLOT(update_window()));
    // timer->start(20);
}

OverlayWidget::~OverlayWidget()
{
    delete ui;
}

void OverlayWidget::WInit()
{
	
	ros::init(argc, argv, "image_rviz");
    ros::NodeHandle nh;
    cv::namedWindow("rviz_image");

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/rviz1/camera1/image", 1,
        &OverlayWidget::update_rviz, this);
	
	ros::Subscriber command_sub = nh.subscribe("overlay_bool", 10,
		&OverlayWidget::commandCallback, this);
	cv::waitKey(1000);
	OverlayController::SharedInstance()->OnSceneUpdate();
    ros::spin();
	std::cout << "88" << std::endl;
}

void OverlayWidget::commandCallback(const std_msgs::Bool::ConstPtr& msg)
{
	switch(msg->data)
	{
		case false:
			std::cout << "close overlay" << std::endl;
			OverlayController::SharedInstance()->HideRviz();
			break;
		case true:
			std::cout << "open overlay" << std::endl;
			OverlayController::SharedInstance()->ShowRviz();
			break;
	}
	
}

void OverlayWidget::update_rviz(const sensor_msgs::ImageConstPtr& msg)
{

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	  std::cout << "OK1" << std::endl;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Update GUI Window
    cv::imshow("rviz_image", cv_ptr->image);
    cv::waitKey(3);

	std::cout << "OK" << std::endl;

    qt_image = QImage((const unsigned char*) (cv_ptr->image.data), cv_ptr->image.cols, cv_ptr->image.rows, QImage::Format_RGB888);

	std::cout << cv_ptr->image.cols << std::endl;
	std::cout << cv_ptr->image.rows << std::endl;

    ui->label->setPixmap(QPixmap::fromImage(qt_image));

    ui->label->resize(ui->label->pixmap()->size());
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
OverlayController *s_pSharedVRController = NULL;

OverlayController *OverlayController::SharedInstance()
{
	if ( !s_pSharedVRController )
	{
        s_pSharedVRController = new OverlayController();
	}
	return s_pSharedVRController;
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
OverlayController::OverlayController()
	: BaseClass()
	, m_strVRDriver( "No Driver" )
	, m_strVRDisplay( "No Display" )
    , m_eLastHmdError( vr::VRInitError_None )
    , m_eCompositorError( vr::VRInitError_None )
    , m_eOverlayError( vr::VRInitError_None )
	, m_ulOverlayHandle( vr::k_ulOverlayHandleInvalid )
	, m_pOpenGLContext( NULL )
	, m_pScene( NULL )
	, m_pFbo( NULL )
	, m_pOffscreenSurface ( NULL )
	, m_pWidget( NULL )
	, voicetimer( NULL )
{
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
OverlayController::~OverlayController()
{
}


//-----------------------------------------------------------------------------
// Purpose: Helper to get a string from a tracked device property and turn it
//			into a QString
//-----------------------------------------------------------------------------
QString GetTrackedDeviceString( vr::IVRSystem *pHmd, vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prop )
{
	char buf[128];
	vr::TrackedPropertyError err;
	pHmd->GetStringTrackedDeviceProperty( unDevice, prop, buf, sizeof( buf ), &err );
	if( err != vr::TrackedProp_Success )
	{
		return QString( "Error Getting String: " ) + pHmd->GetPropErrorNameFromEnum( err );
	}
	else
	{
		return buf;
	}
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
bool OverlayController::Init()
{
	bool bSuccess = true;

    m_strName = "systemoverlay";

	QStringList arguments = qApp->arguments();

	int nNameArg = arguments.indexOf( "-name" );
	if( nNameArg != -1 && nNameArg + 2 <= arguments.size() )
	{
		m_strName = arguments.at( nNameArg + 1 );
	}

	QSurfaceFormat format;

	//set opengl version (min,max)
	format.setMajorVersion( 4 );
	format.setMinorVersion( 1 );
	format.setProfile( QSurfaceFormat::CompatibilityProfile );


	//Sets the format the OpenGL context
	m_pOpenGLContext = new QOpenGLContext();
	m_pOpenGLContext->setFormat( format );
	bSuccess = m_pOpenGLContext->create();
	if( !bSuccess )
		return false;

	// create an offscreen surface to attach the context and FBO to
	m_pOffscreenSurface = new QOffscreenSurface();
	m_pOffscreenSurface->create();
	m_pOpenGLContext->makeCurrent( m_pOffscreenSurface );

	
	m_pScene = new QGraphicsScene();
	connect( m_pScene, SIGNAL(changed(const QList<QRectF>&)), this, SLOT( OnSceneChanged(const QList<QRectF>&)) );

	// Loading the OpenVR Runtime
	bSuccess = ConnectToVRRuntime();

    bSuccess = bSuccess && vr::VRCompositor() != NULL;

    if( vr::VROverlay() )
	{
        std::string sKey = std::string( "sample." ) + m_strName.toStdString();
        vr::VROverlayError overlayError = vr::VROverlay()->CreateOverlay( sKey.c_str(), m_strName.toStdString().c_str(), &m_ulOverlayHandle);
		bSuccess = bSuccess && overlayError == vr::VROverlayError_None;
	}

	if( bSuccess )
	{
        vr::VROverlay()->SetOverlayWidthInMeters( m_ulOverlayHandle, 0.2f );
        vr::VROverlay()->SetOverlayTransformTrackedDeviceRelative(m_ulOverlayHandle, vr::k_unTrackedDeviceIndex_Hmd, &transform);
		vr::VROverlay()->SetOverlayAlpha(m_ulOverlayHandle,0.9);
		vr::VROverlay()->ShowOverlay(m_ulOverlayHandle);
	}
	
	// voicetimer = new QTimer(this);
	// connect(voicetimer, SIGNAL(timeout()),this,SLOT(recognizeSpeech()));
	// voicetimer -> setInterval(20);
	// voicetimer -> start();
	std::cout << "44" << std::endl;
	return true;
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void OverlayController::Shutdown()
{
	DisconnectFromVRRuntime();

	delete m_pScene;
	delete m_pFbo;
	delete m_pOffscreenSurface;

	if( m_pOpenGLContext )
	{
//		m_pOpenGLContext->destroy();
		delete m_pOpenGLContext;
		m_pOpenGLContext = NULL;
	}
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void OverlayController::OnSceneChanged( const QList<QRectF>& )
{
	// skip rendering if the overlay isn't visible
    if( ( m_ulOverlayHandle == k_ulOverlayHandleInvalid ) || !vr::VROverlay() ||
        ( !vr::VROverlay()->IsOverlayVisible( m_ulOverlayHandle ) ))
        return;

	m_pOpenGLContext->makeCurrent( m_pOffscreenSurface );
	m_pFbo->bind();
	
	QOpenGLPaintDevice device( m_pFbo->size() );
	QPainter painter( &device );

	m_pScene->render( &painter );

	m_pFbo->release();

	GLuint unTexture = m_pFbo->texture();
	if( unTexture != 0 )
	{
        vr::Texture_t texture = {(void*)(uintptr_t)unTexture, vr::TextureType_OpenGL, vr::ColorSpace_Auto };
        vr::VROverlay()->SetOverlayTexture( m_ulOverlayHandle, &texture );
	}
	std::cout << "00" << std::endl;
}
//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------

void OverlayController::OnSceneUpdate()
{
	// skip rendering if the overlay isn't visible
    if( ( m_ulOverlayHandle == k_ulOverlayHandleInvalid ) || !vr::VROverlay() ||
        ( !vr::VROverlay()->IsOverlayVisible( m_ulOverlayHandle ) ))
        return;

	m_pOpenGLContext->makeCurrent( m_pOffscreenSurface );
	m_pFbo->bind();
		
	QOpenGLPaintDevice device( m_pFbo->size() );
	QPainter painter( &device );

	m_pScene->render( &painter );

	m_pFbo->release();

	GLuint unTexture = m_pFbo->texture();
	if( unTexture != 0 )
	{
        vr::Texture_t texture = {(void*)(uintptr_t)unTexture, vr::TextureType_OpenGL, vr::ColorSpace_Auto };
        vr::VROverlay()->SetOverlayTexture( m_ulOverlayHandle, &texture );
	}
	std::cout << "888" << std::endl;
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void OverlayController::SetWidget( QWidget *pWidget )
{
	if( m_pScene )
	{
		// all of the mouse handling stuff requires that the widget be at 0,0
		pWidget->move( 0, 0 );
		m_pScene->addWidget( pWidget );
	}
	m_pWidget = pWidget;

	m_pFbo = new QOpenGLFramebufferObject( pWidget->width(), pWidget->height(), GL_TEXTURE_2D );

    if( vr::VROverlay() )
    {
        vr::HmdVector2_t vecWindowSize =
        {
            (float)pWidget->width(),
            (float)pWidget->height()
        };
        vr::VROverlay()->SetOverlayMouseScale( m_ulOverlayHandle, &vecWindowSize );
    }
	std::cout << "66" << std::endl;
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
bool OverlayController::ConnectToVRRuntime()
{
    m_eLastHmdError = vr::VRInitError_None;
    vr::IVRSystem *pVRSystem = vr::VR_Init( &m_eLastHmdError, vr::VRApplication_Overlay );

    if ( m_eLastHmdError != vr::VRInitError_None )
	{
		m_strVRDriver = "No Driver";
		m_strVRDisplay = "No Display";
		return false;
	}

    m_strVRDriver = GetTrackedDeviceString(pVRSystem, vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_TrackingSystemName_String);
    m_strVRDisplay = GetTrackedDeviceString(pVRSystem, vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_SerialNumber_String);

	return true;
}

void OverlayController::ShowRviz()
{
	vr::VROverlay()->SetOverlayAlpha(m_ulOverlayHandle,0.5);
}
void OverlayController::HideRviz()
{
	vr::VROverlay()->SetOverlayAlpha(m_ulOverlayHandle,0);
}

void OverlayController::recognizeSpeech()
{
    // Creates an instance of a speech config with specified subscription key and service region.
    // Replace with your own subscription key and service region (e.g., "westus").
    auto config = SpeechConfig::FromSubscription("3a2dc20079054eae8f486f2dda519630", "koreacentral");

    // Creates a speech recognizer.
    auto recognizer = SpeechRecognizer::FromConfig(config);
    std::cout << "Say something...\n";

    // Starts speech recognition, and returns after a single utterance is recognized. The end of a
    // single utterance is determined by listening for silence at the end or until a maximum of 15
    // seconds of audio is processed.  The task returns the recognition text as result. 
    // Note: Since RecognizeOnceAsync() returns only a single utterance, it is suitable only for single
    // shot recognition like command or query. 
    // For long-running multi-utterance recognition, use StartContinuousRecognitionAsync() instead.
    auto result = recognizer->RecognizeOnceAsync().get();

    // Checks result.
    if (result->Reason == ResultReason::RecognizedSpeech)
    {
        std::cout << "We recognized: " << result->Text << std::endl;
		if ( result->Text == "Close.")
		{
			HideRviz();
		}
		else if ( result->Text == "Open.")
		{
			ShowRviz();
		}
    }
    else if (result->Reason == ResultReason::NoMatch)
    {
        std::cout << "NOMATCH: Speech could not be recognized." << std::endl;
    }
    else if (result->Reason == ResultReason::Canceled)
    {
        auto cancellation = CancellationDetails::FromResult(result);
        std::cout << "CANCELED: Reason=" << (int)cancellation->Reason << std::endl;

        if (cancellation->Reason == CancellationReason::Error) 
        {
            std::cout << "CANCELED: ErrorCode= " << (int)cancellation->ErrorCode << std::endl;
            std::cout << "CANCELED: ErrorDetails=" << cancellation->ErrorDetails << std::endl;
            std::cout << "CANCELED: Did you update the subscription info?" << std::endl;
        }
    }
}

void OverlayController::DisconnectFromVRRuntime()
{
	vr::VR_Shutdown();
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
QString OverlayController::GetVRDriverString()
{
	return m_strVRDriver;
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
QString OverlayController::GetVRDisplayString()
{
	return m_strVRDisplay;
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
bool OverlayController::BHMDAvailable()
{
    return vr::VRSystem() != NULL;
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------

vr::HmdError OverlayController::GetLastHmdError()
{
	return m_eLastHmdError;
}


