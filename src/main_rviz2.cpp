// #include "widget.h"
#include "qtoverlaycontroller.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "image_rviz");
    QApplication a(argc, argv);
    std::cout << "11" << std::endl;
    OverlayWidget *pOverlayWidget = new OverlayWidget;

    std::cout << "33" << std::endl;

    OverlayController::SharedInstance()->Init();

    std::cout << "55" << std::endl;

    OverlayController::SharedInstance()->SetWidget( pOverlayWidget );

    std::cout << "77" << std::endl;

    pOverlayWidget -> WInit();

        
    // OverlayController::SharedInstance()->recognizeSpeech();

    
    //ros::spin();
    // don't show widgets that you're going display in an overlay
    //w.show();

    return a.exec();
}
