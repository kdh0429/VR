// #include "widget.h"
#include "qtoverlaycontroller.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "image_rviz");
    QApplication a(argc, argv);
    std::cout << "ros initialized" << std::endl;
    OverlayWidget *pOverlayWidget = new OverlayWidget;

    OverlayController::SharedInstance()->Init();

    OverlayController::SharedInstance()->SetWidget( pOverlayWidget );

    pOverlayWidget -> WInit();

        
    // OverlayController::SharedInstance()->recognizeSpeech();

    
    //ros::spin();
    // don't show widgets that you're going display in an overlay
    //w.show();

    return a.exec();
}
