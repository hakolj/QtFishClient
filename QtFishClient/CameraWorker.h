#pragma once
#include "stdafx.h"
#include <qobject.h>
#include <opencv.hpp>
#include "CameraPose.h"


class CameraWorker:public QObject
{
	Q_OBJECT
public:
	CameraWorker() {};
	CameraPose cp1;
	CameraPose cp2;

	bool isCalibrate = true;


private:
	QMutex m_Mutex;
	bool iReadyToExit = false;
	cv::Point3f targetPos;

	bool SetCamera(double fps);
	void StopProcess(); // close camera and files safely before stopped

	void DetectTarget(double seconds);

public slots:
	void StartCamera();
	void StopCamera();
	void StartCalibrate();
	
signals:
	void WorkFinished();
	void ImageReady(QImage image, double x, double y, double z);
};

