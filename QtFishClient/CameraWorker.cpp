#include "CameraWorker.h"
#include <qdebug.h>
#include <opencv.hpp>
//#include <opencv2/imgproc/types_c.h>
#include "CameraPose.h"
#include "ImageProcess.h"
#include <algorithm>
#include <iostream>
#include <fstream>
#include <qimage.h>
#include "SolveLine.h"
using namespace std;
using namespace cv;


void CameraWorker::StartCamera() {

    double timeLimit = 200; // in millisecond;
    double startTime = 0;
    double elaspTime = timeLimit;
    double seconds = 0;


    double fps = 1000.0 / timeLimit;
    bool iset = SetCamera(fps);

    startTime = (double)cv::getCPUTickCount();
    int count = 0;

    bool _isCalibrate;
    string filename = "traj2.txt";

    std::ofstream os(filename, std::ios_base::out, ios::trunc);

    if (!os.is_open())
    {
        cout << "未成功打开文件" << endl;
        return;
    }
    os.close();

	while (1) {
        {
            QMutexLocker locker(&m_Mutex);
            _isCalibrate = this->isCalibrate;
            //_isCalibrate = false;
            if (iReadyToExit) {
                // ready to exit
                StopProcess();
                emit WorkFinished();
                return;
            }
        }
        if (elaspTime < timeLimit) {
            elaspTime = ((double)cv::getTickCount() - startTime) / cv::getTickFrequency() * 1000;
            cv::waitKey(1);
            continue;
        }
        else {
            startTime = (double)cv::getCPUTickCount();
            cout << elaspTime << endl;
            qDebug() << elaspTime << endl;
            seconds += elaspTime / 1000.0;
            elaspTime = ((double)cv::getTickCount() - startTime) / cv::getTickFrequency() * 1000;

        }
        if (!cp1.Capture() || !cp2.Capture()) {
            qDebug() << "capture failed" << endl;
            continue;
        }
        cp1.PrepareFrame();
        cp2.PrepareFrame();
        imshow("img1", cp1.image);

        cout << cp1.image.row(2) << endl;
        //cv::waitKey(0);
        if (_isCalibrate) {

            int status1 = cp1.ChessboardCalibration(5, { 5,14 });
            int status2 = cp2.ChessboardCalibration(5, { 5,14 });

            if ((status1 == 1) && (status2 == 1)) {
                // calibration done
                cp1.chessboardCorner_cali.clear();
                cp2.chessboardCorner_cali.clear();
                   
                QMutexLocker locker(&m_Mutex);
                isCalibrate = false; //reset calibrate flag;
            }

        }
        else {
            cp1.DrawAxis();
            cp2.DrawAxis();
            DetectTarget(seconds);
        }
        


        Mat draw;
        ManyImages({ cp1.drawImage,cp2.drawImage }, draw, 1, 2);
        //imshow("draw", draw);
        QImage qimage = QImage((const uchar*)draw.data, draw.cols, draw.rows, QImage::Format_RGB888).copy();
        emit ImageReady(qimage, targetPos.x, targetPos.y, targetPos.z);

	}

    // if the loop is accidently broken
    StopProcess();
    emit WorkFinished();
    return;

}

void CameraWorker::StopCamera() {
	QMutexLocker locker(&m_Mutex);
	iReadyToExit = true;
}

void CameraWorker::StartCalibrate() {
    QMutexLocker locker(&m_Mutex);
    isCalibrate = true;
}

bool CameraWorker::SetCamera(double fps) {
    //cp1.capture.open(0);
    //cp1.capture.open("D:/Homework/Y4-1/2021-10-08-机器鱼/camera/CameraTest/exp/out1 - 副本.mp4");
    cp1.capture.open("D:/Homework/Y4-1/2021-10-08-机器鱼/camera/CameraTest/exp/out1 - 副本 (2).mp4");
    cp1.capture.set(CAP_PROP_POS_FRAMES, 600);
    long totalFrameNumber = cp1.capture.get(CAP_PROP_FRAME_COUNT);
    cout << totalFrameNumber;
    cp1.SetResolution(800, 600);
    cp1.Capture(); //test capture

    cp1.SetCameraMatrix(504.987011848488, 505.184041377166, 395.456624166183, 311.677680606481); //cam1
    cp1.SetDistCoeff(0.0479365624820876, -0.0641466641816116, 9.827257251616929e-05, 2.279449281466257e-04);//cam1




    //cp2.capture.open("D:/Homework/Y4-1/2021-10-08-机器鱼/camera/CameraTest/exp/out2 - 副本.mp4");
    cp2.capture.open("D:/Homework/Y4-1/2021-10-08-机器鱼/camera/CameraTest/exp/out2 - 副本 (2).mp4");
    cp2.capture.set(CAP_PROP_POS_FRAMES, 600);
    //cp2.capture.open(1);
    cp2.SetResolution(800, 600);
    cp2.Capture(); //test capture

    cp2.SetCameraMatrix(503.338034504919, 503.320684719379, 392.545795889937, 297.777367039825); //cam2
    cp2.SetDistCoeff(0.0468113299023762, -0.0638374315580488, 5.136699316467411e-04, 2.101064394121617e-05);//cam2

    // set writer 

    /*int fps = 1000 / timeLimit;*/
    //创建视频文件
    cp1.writer.open("post/out1.mp4", VideoWriter::fourcc('M', 'P', '4', '2'), fps, Size(800, 600));
    cp2.writer.open("post/out2.mp4", VideoWriter::fourcc('M', 'P', '4', '2'), fps, Size(800, 600));

    if (!cp1.writer.isOpened() || !cp2.writer.isOpened())
    {
        qDebug() << "VideoWriter open failed!" << endl;
        StopProcess();
        return false;
    }
    qDebug() << "VideoWriter open success!" << endl;

    return true;
}

void CameraWorker::StopProcess() {
    cp1.capture.release();
    cp2.capture.release();
    cp1.writer.release();
    cp2.writer.release();
    cv::destroyAllWindows();
    emit WorkFinished();
}


// detect the world position of target
void CameraWorker::DetectTarget(double seconds) {
    //bool icheckSucess = (cp1.UseChessboard() && cp2.UseChessboard());
    bool icheckSucess = true;

    //if (!icheckSucess) continue;

    if (icheckSucess) {

        Point2f center1;
        Point2f center2;
        bool ilocatedTarget = (cp1.LocateTarget(center1) && cp2.LocateTarget(center2));
        if (ilocatedTarget) {
            targetPos = CameraPose::SolveTargetPosition(cp1, center1, cp2, center2);
            std::ofstream os("traj2.txt", std::ios::app);

            if (!os.is_open())
            {
                cout << "未成功打开文件" << endl;
            }
            if (!isnan(targetPos.x)) {
                os << seconds << "    " << targetPos.x << " " << targetPos.y << " " << targetPos.z << endl;
                os.close();
            }
        }

    }
}