#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_QtFishClient.h"
#include <QtNetwork/qtcpsocket.h>
#include <QTimer>  
#include <opencv.hpp>
#include <ui_QtFishClient.h>
#include <qthread.h>
#include "CameraWorker.h"
#include <opencv.hpp>
class QtFishClient : public QMainWindow
{
    Q_OBJECT

public:
    QtFishClient(QWidget *parent = Q_NULLPTR);
    QTcpSocket* tcpClient;
    ~QtFishClient();

private:
    Ui::QtFishClientClass ui;
    QTimer* timer;
    QImage* image;
    cv::VideoCapture capture;
    int frame_number; //total number of frame of the video
    int current_frame = 0;
    QThread cameraThread;
    CameraWorker* camworker;

    cv::Point3d fishPos;
    cv::Point3d targetPos;
    double fishYaw = 0;
    bool _fishConnected = false;

    void SendActionToFish(int direction, int speed);


public slots:
    void OnBtnConnectClick();
    void OnBtnSendClick();
    void OnBtnSend_0x67Click();

    void ReadTCPData();

    void ReadFrame();
    void OnBtnStartVideoClick();

    void ShowCameraFrame(QImage image, double x, double y, double z);
    void StartCameraThread();
    void StopCameraThread();
    void CalculateAction();
    void CalibrateCamera();
    
    void TestCaliEmit() { emit TestCaliSignal(); }

         
signals:
    void FishPositionObtained();
    void TestCaliSignal();
};
