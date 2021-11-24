#pragma once
#include <opencv.hpp>


class CameraPose {

public:
    cv::VideoCapture capture;
    cv::VideoWriter writer;
    cv::Mat cameraMatrix;
    cv::Mat image;
    cv::Mat drawImage;
    std::vector<double> distCoeff;
    bool idraw = true;
    bool iprojTest = true; // whether test projection?
    cv::Mat rvec;
    cv::Mat tvec;
    cv::Mat RoteM;

    std::vector<cv::Point2f> chessboardCorner_cali;

public:
    CameraPose();



    void SetCameraMatrix(double fx, double fy, double cx, double cy) {
        cameraMatrix = cv::Mat(3, 3, CV_64FC1, cv::Scalar::all(0));
        cameraMatrix.ptr<double>(0)[0] = fx;
        cameraMatrix.ptr<double>(0)[2] = cx;
        cameraMatrix.ptr<double>(1)[1] = fy;
        cameraMatrix.ptr<double>(1)[2] = cy;
        cameraMatrix.ptr<double>(2)[2] = 1.0;
    }

    void SetDistCoeff(double k1, double k2, double p1, double p2);

    void SetResolution(int camwidth, int camheight) {
        capture.set(cv::CAP_PROP_FRAME_WIDTH, camwidth);
        capture.set(cv::CAP_PROP_FRAME_HEIGHT, camheight);
    }

    bool Capture() { return capture.read(image); } //capture a frame to image

    bool UseChessboard();
    bool UseAnchorPoints();

    //use chessboard to calibrate the pose of camera.
    // reqFrames: use how many frames to calibrate
    // patternSize: the size of chessboard corner points
    int ChessboardCalibration(int reqFrames, cv::Size patternSize);
    bool FindAnchorPoints(std::vector<cv::Point2f>& anchors);

    static cv::Point3f ImageFrame2CameraFrame(cv::Point2f p, double F, const cv::Mat& camera_matrix);
    cv::Point3f ImageFrame2CameraFrame(cv::Point2f p, double F);
    cv::Point3f ImageFrame2WorldFrameDir(cv::Point2f p, double F);

    cv::Point3f CameraCenter2WorldFrame();

    void PrepareFrame();
    bool LocateTarget(cv::Point2f& center);


    static cv::Point3f SolveTargetPosition(CameraPose cp1, cv::Point2f point1, CameraPose cp2, cv::Point2f point2); // solve the world p

    void DrawAxis();

};