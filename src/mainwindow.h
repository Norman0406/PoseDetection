#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "depthcamerakinectsdk.h"
#include "depthcamerakinectsdk2.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    DepthCamera* m_camera;
};

#endif // MAINWINDOW_H
