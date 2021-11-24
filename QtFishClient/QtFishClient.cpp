#include "QtFishClient.h"
#include "stdafx.h"
#include <iostream>
#include "CameraWorker.h"
using namespace cv;
QtFishClient::QtFishClient(QWidget* parent)
    : QMainWindow(parent), camworker(),fishPos(),targetPos(),image()
{
    ui.setupUi(this);

    tcpClient = new QTcpSocket(this);   //实例化tcpClient
    tcpClient->abort();                 //取消原有连接
    connect(tcpClient, SIGNAL(readyRead()), this, SLOT(ReadTCPData()));
    //connect(tcpClient, SIGNAL(error(QAbstractSocket::SocketError)), this, SLOT(ReadError(QAbstractSocket::SocketError)));

    // vedio part
    timer = new QTimer(this);
    connect(ui.startVideoBtn, SIGNAL(clicked()), this, SLOT(OnBtnStartVideoClick()));
    connect(timer, SIGNAL(timeout()), this, SLOT(ReadFrame()));
    connect(this, &QtFishClient::FishPositionObtained, this, &QtFishClient::CalculateAction);

    targetPos = Point3f(0, 0, 0);


}



void QtFishClient::OnBtnConnectClick() {
    QString ip = QString("192.168.8.1");
    //QString port = QString("2017");

    tcpClient->connectToHost(ip, 2017);

    if (tcpClient->waitForConnected(1000))  // 连接成功则进入if{}
    {
        //ui.tcpTextBrowser->setText("connect sucess");
        ui.tcpTextBrowser->append("connect sucess\n");
        std::cout << "connect sucess" << std::endl;
        _fishConnected = true;
        //ui->btnConnect->setText("断开");
        //ui->btnSend->setEnabled(true);
    }
    else {
        ui.tcpTextBrowser->append("connect failed\n");
        std::cout << "connect failed" << std::endl;
    }
}


void QtFishClient::OnBtnSendClick() {
    float num1 = ui.lineEdit->text().toFloat();
    float num2 = ui.lineEdit_2->text().toFloat();
    float num3 = ui.lineEdit_3->text().toFloat();

    SendActionToFish(num1, num2);

    //Json::Value JData;
    //Json::FastWriter writer;
    //std::string strJdata;

    //JData["cmd"] = 0x81;
    //JData["direction"] = num1;
    //JData["speed"] = num2;
    //JData["num3"] = num3;
    //strJdata = writer.write(JData);
    //tcpClient->write(strJdata.c_str(), strJdata.length());
}

void QtFishClient::SendActionToFish(int direction, int speed) {
    if (!_fishConnected) return;
    Json::Value JData;
    Json::FastWriter writer;
    std::string strJdata;

    JData["cmd"] = 0x81;
    JData["direction"] = direction;
    JData["speed"] = speed;
    JData["num3"] = direction;
    strJdata = writer.write(JData);
    tcpClient->write(strJdata.c_str(), strJdata.length());
}

void QtFishClient::OnBtnSend_0x67Click() {
    float num1 = ui.lineEdit->text().toFloat();
    float num2 = ui.lineEdit_2->text().toFloat();
    float num3 = ui.lineEdit_3->text().toFloat();
    
    Json::Value JData;
    Json::FastWriter writer;
    std::string strJdata;
    JData["cmd"] = 0x67;
    JData["speed"] = num2;
    strJdata = writer.write(JData);
    tcpClient->write(strJdata.c_str(), strJdata.length());

}


void QtFishClient::ReadTCPData() {
    QByteArray buffer = tcpClient->readAll();
    char* bufferdata = buffer.data();
    Json::Reader reader;
    Json::Value value;

    if (reader.parse(bufferdata, bufferdata + buffer.length(), value)) {
        if (value.isMember("accelx")) {
            float accelx = value["accelx"].asFloat();
            float accely = value["accely"].asFloat();
            float accelz = value["accelz"].asFloat();

            float pitch = value["pitch"].asFloat();
            float roll = value["roll"].asFloat();
            float yaw = value["yaw"].asFloat();
            float direction = value["direction"].asFloat();
            float speed = value["speed"].asFloat();

            QString msg = "accelx=" + QString::number(accelx) + " ,accely=" + QString::number(accely) + " ,accelz=" + QString::number(accelz); 
            ui.tcpTextBrowser->append(msg);
            ui.textBrowserAccelx->setText(QString::number(accelx));
            ui.textBrowserAccely->setText(QString::number(accely));
            ui.textBrowserAccelz->setText(QString::number(accelz));

            ui.textBrowserPitch->setText(QString::number(pitch));
            ui.textBrowserRoll->setText(QString::number(roll));
            ui.textBrowserYaw->setText(QString::number(yaw));

            ui.textBrowserDirection->setText(QString::number(direction));
            ui.textBrowserSpeed->setText(QString::number(speed));
            fishYaw = yaw;
            //ui.textBrowserAccelx->set
            //ui.tcpTextBrowser->append(buffer);
        }
    }

    //ui.tcpTextBrowser->append(buffer);

    //if (!reader.parse(bufferdata, value)) ;

    //int user1 = value["objn"].asInt();

    //ui.tcpTextBrowser->append(QString::number(user1));
    ui.tcpTextBrowser->append(buffer);

}

void getColor(const cv::Mat& srcimage, cv::Mat& dstimage) {
    Mat hsv = srcimage.clone();
    cvtColor(srcimage, hsv, COLOR_BGR2HSV);

    double hmin = 20, smin = 43, vmin = 46;
    double hmax = 40, smax = 255, vmax = 255;
    inRange(hsv, Scalar(hmin, smin, vmin), Scalar(hmax, smax, vmax), dstimage);
}

void GetContourCenter(const std::vector<Point>& contour, cv::Point2f& p)
{
    //重心法抓中心点
    Moments mu;
    mu = moments(contour, true);
    p = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
}


void QtFishClient::ReadFrame() {
    if (!capture.isOpened())
    {
        std::cout << "open video error";
        return;
        
    }
    Mat frame; //Mat对象  其实就是图像对象
    //capture.set(CAP_PROP_POS_FRAMES, current_frame);//从此时的帧数开始获取帧
            //char image_name[20];
    if (!capture.read(frame))
    {
        std::cout << "读取视频失败" << std::endl;
    }

    cv::GaussianBlur(frame, frame, cv::Size(5, 5), 1.0, 1.0, 4);
    Mat mask(frame.size(), CV_8UC1);       ;
    getColor(frame, mask);
    std::vector<Vec4i> hierarchy;
    std::vector<std::vector<Point>> contours;
    cv::findContours(mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point());

    //std::vector<Point2f> centers(contours.size());
    double maxm00 = 0.0;
    int mainContour = 0;
    for (int i = 0; i < contours.size(); i++)
    {
        Moments mu = moments(contours[i], true);
        if (mu.m00 > maxm00) {
            maxm00 = mu.m00;
            mainContour = i;
        }
    }
    if (contours.size() > 0) {
        //contours[i]代表的是第i个轮廓，contours[i].size()代表的是第i个轮廓上所有的像素点数  
        for (int j = 0; j < contours[mainContour].size(); j++)
        {
            //绘制出contours向量内所有的像素点  
            Point P = Point(contours[mainContour][j].x, contours[mainContour][j].y);
            frame.at<uchar>(P) = 255;
        }
        Point2f center;
        GetContourCenter(contours[mainContour], center); //寻找轮廓中心
        //绘制轮廓  
        drawContours(frame, contours, mainContour, Scalar(255), 1, 8, hierarchy);
        cv::circle(frame, center, 25, { 0,0,255 }, 10, 8, 0);
    }


    //QImage image((const uchar*)frame.data, frame.cols, frame.rows, QImage::Format_RGB888);
    QImage image = QImage((const uchar*)frame.data, frame.cols, frame.rows, QImage::Format_RGB888).rgbSwapped().scaled(ui.videoLabel->size());
    ui.videoLabel->setPixmap(QPixmap::fromImage(image));
    //imshow("che", frame);//显示
    //current_frame += 10;

}

void QtFishClient::OnBtnStartVideoClick() {
    capture.open("D:/Homework/Y4-1/2021-10-08-机器鱼/OpenCVTest/编队巡游项目.mp4");
    timer->start(100);
    //int frame_width = (int)capture.get(CAP_PROP_FRAME_WIDTH);
    //int frame_height = (int)capture.get(CAP_PROP_FRAME_HEIGHT);
    frame_number = capture.get(CAP_PROP_FRAME_COUNT);
    current_frame = 0;
}


QtFishClient::~QtFishClient() {

}

void QtFishClient::StopCameraThread() {
    if (cameraThread.isRunning()) {
        camworker->StopCamera();
        cameraThread.quit();
        cameraThread.wait();
    }

}



void QtFishClient::StartCameraThread() {
    if (cameraThread.isRunning()) {
        qDebug() << "camera thread is already running!" << endl;
        return;
    }
    
    camworker = new CameraWorker();
    camworker->moveToThread(&cameraThread);

    //QThread* tempthread = new QThread();
    //camworker->moveToThread(tempthread);

    //connect(tempthread, &QThread::started, camworker, &CameraWorker::DoWork);
    //connect(camworker, &CameraWorker::WorkFinished, tempthread, &QThread::quit);

    //connect(camworker, &CameraWorker::WorkFinished, camworker, &CameraWorker::deleteLater);
    //connect(tempthread, &QThread::finished, tempthread, &QThread::deleteLater);
    //tempthread->start();

    connect(&cameraThread, &QThread::started, camworker, &CameraWorker::StartCamera);
    connect(camworker, &CameraWorker::WorkFinished, this, &QtFishClient::StopCameraThread);

    connect(camworker, &CameraWorker::WorkFinished, camworker, &CameraWorker::deleteLater);
    connect(ui.btnStopCamera, &QPushButton::clicked, this, &QtFishClient::StopCameraThread);
    //connect(&cameraThread, &QThread::finished, &cameraThread, &QThread::deleteLater);
    connect(camworker, &CameraWorker::ImageReady, this, &QtFishClient::ShowCameraFrame);

    connect(ui.btnCalibrate, &QPushButton::clicked, this, &QtFishClient::CalibrateCamera);



    cameraThread.start();

}

void QtFishClient::ShowCameraFrame(QImage image, double x, double y, double z) {
    /*QImage image = QImage((const uchar*)frame.data, frame.cols, frame.rows, QImage::Format_RGB888).rgbSwapped().scaled(ui.videoLabel->size());*/
    
    ui.videoLabel->setPixmap(QPixmap::fromImage(image.rgbSwapped().scaled(ui.videoLabel->size())));
    ui.textBrowserPosx->setText(QString::number(x));
    ui.textBrowserPosy->setText(QString::number(y));
    ui.textBrowserPosz->setText(QString::number(z));
    fishPos.x = x;
    fishPos.y = y;
    fishPos.z = z;
    emit FishPositionObtained();
}

void QtFishClient::CalculateAction() {
    cv::Point2f targetvec = Point2f(targetPos.x - fishPos.x, targetPos.y - fishPos.y);
    float dist = sqrt(targetvec.dot(targetvec));
    float dir = atan2(targetvec.y, targetvec.x);
    float fishAngle = fishYaw / 180 * M_PI;
    float temp = dir - fishAngle + 2 * M_PI + M_PI;
    float relaAngle = fmod(temp, 2 * M_PI) - M_PI; // range (-pi, pi)
    int dirAction = 5;
    if (relaAngle > M_PI_4) {
        // 目标在鱼左方
        dirAction = 7;
    }
    else if (relaAngle < -M_PI / 4) {
        //目标在鱼右方
        dirAction = 3;
    }
    else {
        // 鱼在前方
        dirAction = 5;
    }
    qDebug() << "fishAngle = " << fishAngle << "dir = " << dir << endl;
    qDebug() << "relaAngle = " << relaAngle << "dirAction = " << dirAction << endl;
    SendActionToFish(dirAction, 2);
    return; 
}

void QtFishClient::CalibrateCamera() {
    camworker->StartCalibrate();
}