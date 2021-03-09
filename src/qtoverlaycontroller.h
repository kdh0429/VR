//====== Copyright Valve Corporation, All rights reserved. =======

#ifndef OPENVROVERLAYCONTROLLER_H
#define OPENVROVERLAYCONTROLLER_H

#ifdef _WIN32
#pragma once
#endif

#include "openvr.h"

#include <QtCore/QtCore>
// because of incompatibilities with QtOpenGL and GLEW we need to cherry pick includes
#include <QtGui/QVector2D>
#include <QtGui/QMatrix4x4>
#include <QtCore/QVector>
#include <QtGui/QVector2D>
#include <QtGui/QVector3D>
#include <QtGui/QOpenGLContext>
#include <QtGui/QOpenGLFramebufferObject>
#include <QtWidgets/QGraphicsScene>
#include <QtGui/QOffscreenSurface>

#include <iostream>
#include <stdio.h>

#include <QtWidgets/QWidget>
#include <QMainWindow>
#include <QTimer>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>



namespace Ui {
class OverlayWidget;
}

class OverlayWidget : public QWidget
{
    Q_OBJECT

public:
    explicit OverlayWidget(QWidget *parent = 0);
    ~OverlayWidget();
    void OverlayWidget::update_rviz(const sensor_msgs::ImageConstPtr& msg);
	void OverlayWidget::WInit();
	void OverlayWidget::commandCallback(const std_msgs::Bool::ConstPtr& msg);
//private slots:


private:
	
	int argc;
	char **argv;
    Ui::OverlayWidget *ui;

    QTimer *timer;
    
    cv::Mat frame;
    QImage qt_image;
};


class OverlayController : public QObject
{
	Q_OBJECT
	typedef QObject BaseClass;

public:
    static OverlayController *SharedInstance();

public:
    OverlayController();
    virtual ~OverlayController();

	bool Init();
	void Shutdown();
	void EnableRestart();

	bool BHMDAvailable();
    vr::IVRSystem *GetVRSystem();
	vr::HmdError GetLastHmdError();

	QString GetVRDriverString();
	QString GetVRDisplayString();
	QString GetName() { return m_strName; }

	void SetWidget( QWidget *pWidget );
	void OnSceneUpdate();

	void OverlayController::ShowRviz();
	void OverlayController::HideRviz();
	void OverlayController::recognizeSpeech();


public slots:
	void OnSceneChanged( const QList<QRectF>& );
	
	
protected:

private:
	bool ConnectToVRRuntime();
	void DisconnectFromVRRuntime();

	vr::TrackedDevicePose_t m_rTrackedDevicePose[ vr::k_unMaxTrackedDeviceCount ];
	QString m_strVRDriver;
	QString m_strVRDisplay;
	QString m_strName;

	vr::HmdError m_eLastHmdError;

private:

	
	vr::HmdError m_eCompositorError;
	vr::HmdError m_eOverlayError;
	vr::VROverlayHandle_t m_ulOverlayHandle;
    vr::VROverlayHandle_t m_ulOverlayThumbnailHandle;

	QOpenGLContext *m_pOpenGLContext;
	QGraphicsScene *m_pScene;
	QOpenGLFramebufferObject *m_pFbo;
	QOffscreenSurface *m_pOffscreenSurface;

	// the widget we're drawing into the texture
	QWidget *m_pWidget;

	QTimer *voicetimer;
	
};


#endif // OPENVROVERLAYCONTROLLER_H
