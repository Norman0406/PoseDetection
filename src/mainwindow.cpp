#include "mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent)
{
    m_camera = new DepthCameraKinectSDK2();
    m_camera->open();

    do {
        m_camera->getNextImage();
    } while (true);
}

MainWindow::~MainWindow()
{
}
