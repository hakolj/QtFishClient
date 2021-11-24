#include "CameraPose.h"
#include <opencv.hpp>
#include "ImageProcess.h"
#include "SolveLine.h"
#include <qtextstream.h>
#include <qdebug.h>

using namespace cv;
using namespace std;

CameraPose::CameraPose() :
    cameraMatrix(Mat(3, 3, CV_64FC1, cv::Scalar::all(0))),
    distCoeff(vector<double>(4, 0.0)),
    RoteM(Mat(3, 3, CV_64FC1)),
    chessboardCorner_cali(0)
     {}


void CameraPose::SetDistCoeff(double k1, double k2, double p1, double p2) {
    distCoeff = vector<double>({ k1,k2,p1,p2 }); //cam2
}

// obtain real time camera pose at each frame
bool CameraPose::UseChessboard() {
    Mat grey;
    cvtColor(image, grey, COLOR_RGB2GRAY);
    vector<Point2f> corners;
    Size patternsize(5, 14);
    bool isuccess = findChessboardCornersSB(grey, patternsize, corners);
    if (!isuccess) return false;
    int cornerCount = corners.size();

    if (idraw) {        
        drawChessboardCorners(drawImage, patternsize, corners, true);
    }

    // calculate corners points in world frame of reference
    vector<Point3f> objPoints(patternsize.width*patternsize.height);
    int n = 0;
    double gridsize = 120.0;
    for (int j = 0; j < patternsize.height; j++) {
        for (int i = 0; i < patternsize.width; i++) {
            objPoints[n].x = i * gridsize;
            objPoints[n].y = j * gridsize;
            objPoints[n].z = 0.0;
            n++;
        }
    }


    cv::solvePnPRansac(objPoints, corners, cameraMatrix, distCoeff, rvec, tvec, cv::noArray());
    //cout << "rvec = " << rvec << endl;
    //cout << "tvec = " << tvec << endl;
    Rodrigues(rvec, RoteM);
    //cout << "RoteM = " << RoteM << endl;
    //uchar* p;
    //p = rvec.ptr<uchar>();
    qDebug()  << rvec.ptr<double>(0)[0] << " " << rvec.ptr<double>(1)[0] << " " << rvec.ptr<double>(2)[0] << endl;

    if (iprojTest) {
        //reprojection test
        vector<Point2f> projectedPoints;
        cv::projectPoints(objPoints, rvec, tvec, cameraMatrix, distCoeff, projectedPoints);

        for (int i = 0; i < patternsize.width * patternsize.height; i++) {
            cv::circle(drawImage, projectedPoints[i], 10, (255, 255, 255));
        }

        vector<Point3f> axisWorldPoint(4);
        axisWorldPoint[0] = Point3d(0, 0, 0);
        axisWorldPoint[1] = Point3d(100, 0, 0);
        axisWorldPoint[2] = Point3d(0, 100, 0);
        axisWorldPoint[3] = Point3d(0, 0, 100);

        vector<Point2f> axisPicPoint(4);
        cv::projectPoints(axisWorldPoint, rvec, tvec, cameraMatrix, distCoeff, axisPicPoint);
        Scalar red(0, 0, 255, 1);
        Scalar blue(255, 0, 0, 1);
        Scalar green(0, 255, 0, 1);
        Scalar cl = Scalar(0, 255, 0);
        cv::line(drawImage, axisPicPoint[0], axisPicPoint[1], blue, 10);
        cv::line(drawImage, axisPicPoint[0], axisPicPoint[2], green, 10);
        cv::line(drawImage, axisPicPoint[0], axisPicPoint[3], red, 10);
    }
    return true;
}


void CameraPose::DrawAxis() {
    if ((rvec.cols == 0) || (tvec.cols == 0)) return;
    vector<Point3f> axisWorldPoint(4);
    axisWorldPoint[0] = Point3d(0, 0, 0);
    axisWorldPoint[1] = Point3d(100, 0, 0);
    axisWorldPoint[2] = Point3d(0, 100, 0);
    axisWorldPoint[3] = Point3d(0, 0, 100);

    vector<Point2f> axisPicPoint(4);
    cv::projectPoints(axisWorldPoint, rvec, tvec, cameraMatrix, distCoeff, axisPicPoint);
    Scalar red(0, 0, 255, 1);
    Scalar blue(255, 0, 0, 1);
    Scalar green(0, 255, 0, 1);
    Scalar cl = Scalar(0, 255, 0);
    cv::line(drawImage, axisPicPoint[0], axisPicPoint[1], blue, 10);
    cv::line(drawImage, axisPicPoint[0], axisPicPoint[2], green, 10);

}

bool CameraPose::UseAnchorPoints() {

    cv::GaussianBlur(image, image, cv::Size(5, 5), 1.0, 1.0, 4);

    vector<Point2f> anchorPoints(4); //o,x,y,diagonal
    bool isuccess = FindAnchorPoints(anchorPoints);
    if (!isuccess) return false;

    vector<Point3f> objPoints(4);

    objPoints[0] = Point3f{ 0,0,0 }; //o 
    objPoints[1] = Point3f{ 200,0,0 }; //x
    objPoints[2] = Point3f{ 0,150,0 }; //y
    objPoints[3] = Point3f{ 200,150,0 };


    cv::solvePnP(objPoints, anchorPoints, cameraMatrix, distCoeff, rvec, tvec);
    //cout << "rvec = " << rvec << endl;
    //cout << "tvec = " << tvec << endl;
    Rodrigues(rvec, RoteM);
    //cout << "RoteM = " << RoteM << endl;

     //reprojection test
    vector<Point2f> projectedPoints;
    cv::projectPoints(objPoints, rvec, tvec, cameraMatrix, distCoeff, projectedPoints);

    //for (int i = 0; i < 88; i++) {
    //    cv::circle(cp1.drawImage, projectedPoints[i], 10, (255, 255, 255));
    //}

    vector<Point3f> axisWorldPoint(4);
    axisWorldPoint[0] = Point3d(0, 0, 0);
    axisWorldPoint[1] = Point3d(100, 0, 0);
    axisWorldPoint[2] = Point3d(0, 100, 0);
    axisWorldPoint[3] = Point3d(0, 0, 100);

    vector<Point2f> axisPicPoint(4);
    cv::projectPoints(axisWorldPoint, rvec, tvec, cameraMatrix, distCoeff, axisPicPoint);
    Scalar red(0, 0, 255, 1);
    Scalar blue(255, 0, 0, 1);
    Scalar green(0, 255, 0, 1);
    Scalar cl = Scalar(0, 255, 0);
    cv::line(drawImage, axisPicPoint[0], axisPicPoint[1], blue, 10);
    cv::line(drawImage, axisPicPoint[0], axisPicPoint[2], green, 10);
    cv::line(drawImage, axisPicPoint[0], axisPicPoint[3], red, 10);

    return true;
}




cv::Point3f CameraPose::ImageFrame2CameraFrame(cv::Point2f p, double F, const cv::Mat& camMat)
{
    // 未消除畸变误差
    double fx;
    double fy;
    double u0;
    double v0;

    fx = camMat.ptr<double>(0)[0];
    u0 = camMat.ptr<double>(0)[2];
    fy = camMat.ptr<double>(1)[1];
    v0 = camMat.ptr<double>(1)[2];
    double zc = F;
    double xc = (p.x - u0) * F / fx;
    double yc = (p.y - v0) * F / fy;
    return cv::Point3f(xc, yc, zc);
}

cv::Point3f CameraPose::ImageFrame2CameraFrame(cv::Point2f p, double F)
{
    return ImageFrame2CameraFrame(p, F, cameraMatrix);
}

cv::Point3f CameraPose::ImageFrame2WorldFrameDir(cv::Point2f p, double F) {

    Point3f objCenterC = ImageFrame2CameraFrame(p, F); //also the vector direction
    Mat objCenterCMat(3, 1, CV_64FC1);
    objCenterCMat.ptr<double>(0)[0] = objCenterC.x;
    objCenterCMat.ptr<double>(1)[0] = objCenterC.y;
    objCenterCMat.ptr<double>(2)[0] = objCenterC.z;

    Mat objCenterWMat(3, 1, CV_64FC1);
    Mat Rinv(3, 3, CV_64FC1);
    invert(RoteM, Rinv);
    objCenterWMat = Rinv * (objCenterCMat - tvec);

    return Point3f(objCenterWMat.ptr<double>(0)[0], objCenterWMat.ptr<double>(1)[0], objCenterWMat.ptr<double>(2)[0]);
}

cv::Point3f CameraPose::CameraCenter2WorldFrame() {


    Mat Rinv(3, 3, CV_64FC1);
    invert(RoteM, Rinv);
    Mat camCenterWMat(3, 1, CV_64FC1, cv::Scalar::all(0));

    camCenterWMat = Rinv * (-tvec);

    return Point3f(camCenterWMat.ptr<double>(0)[0], camCenterWMat.ptr<double>(1)[0], camCenterWMat.ptr<double>(2)[0]);

}

bool CameraPose::FindAnchorPoints(std::vector<cv::Point2f>& anchors) {

    Mat maskAnchor(image.size(), CV_8UC1); ;
    getColor(image, maskAnchor, "red");

    imshow("mask", maskAnchor);

    std::vector<Vec4i> hierarchy_anc;
    std::vector<std::vector<Point>> contours_anc;
    cv::findContours(maskAnchor, contours_anc, hierarchy_anc, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point());

    if (contours_anc.size() < 4) return false;
    std::vector<int> maxidx;
    maxContours(contours_anc, maxidx, 4);

    vector<Point2f> anchorCenters(4);

    drawImage = image.clone();
    for (int i = 0; i < 4; i++) {
        cout << maxidx[i] << " ";
    }
    cout << endl;

    //contours[i]代表的是第i个轮廓，contours[i].size()代表的是第i个轮廓上所有的像素点数  
    for (int i = 0; i < 4; i++) {


        GetContourCenter(contours_anc[maxidx[i]], anchorCenters[i]); //寻找轮廓中心
        if (isnan(anchorCenters[i].x) || isnan(anchorCenters[i].y)) {
            cout << "NAN is encountered when solving anchorCenters" << endl;
            return false;
        }
        //绘制轮廓  
//drawContours(cp1.drawImage, contours, mainContour1, Scalar(255), 1, 8, hierarchy1);
        cv::circle(drawImage, anchorCenters[i], 25, { 0,0,255 }, 10, 8, 0);

    }


    //区分四个锚点的相对位置
    vector<Point2f> imageCorners(3); //lu, ld, ru
    imageCorners[0] = Point2f(0, 0);
    imageCorners[1] = Point2f(0, maskAnchor.rows);
    imageCorners[2] = Point2f(maskAnchor.cols, 0);
    vector<int> idxCorners(3);
    vector<double> distToLU(4);
    vector<double> distToLD(3);
    vector<double> distToRU(2);
    vector<int> candidate = vector<int>{ 0,1,2,3 };

    // 左上
    for (int i = 0; i < candidate.size(); i++) {
        distToLU[i] = Distance(imageCorners[0], anchorCenters[candidate[i]]);
    }
    vector<double>::iterator miniter = std::min_element(distToLU.begin(), distToLU.end());
    int minloc = distance(distToLU.begin(), miniter);
    int idxPLU = candidate[minloc];
    candidate.erase(candidate.begin() + minloc);

    //左下
    for (int i = 0; i < candidate.size(); i++) {
        distToLD[i] = Distance(imageCorners[1], anchorCenters[candidate[i]]);
    }
    miniter = std::min_element(distToLD.begin(), distToLD.end());
    minloc = distance(distToLD.begin(), miniter);
    int idxPLD = candidate[minloc];
    candidate.erase(candidate.begin() + minloc);

    //右上
    for (int i = 0; i < candidate.size(); i++) {
        distToRU[i] = Distance(imageCorners[2], anchorCenters[candidate[i]]);
    }
    miniter = std::min_element(distToRU.begin(), distToRU.end());
    minloc = distance(distToRU.begin(), miniter);
    int idxPRU = candidate[minloc];
    candidate.erase(candidate.begin() + minloc);

    //右下
    int idxPRD = 6 - idxPLU - idxPLD - idxPRU;

    anchors = vector<Point2f>(4);
    anchors[0] = anchorCenters[idxPLU]; //o，左上
    anchors[1] = anchorCenters[idxPRU]; //x，右上
    anchors[2] = anchorCenters[idxPLD]; //y，左下
    anchors[3] = anchorCenters[idxPRD];

    return true;

}

void CameraPose::PrepareFrame() {
    drawImage = image.clone();
    writer.write(image);
}

bool CameraPose::LocateTarget(cv:: Point2f& center) {
    Mat blurImg;
    cv::GaussianBlur(image, blurImg, cv::Size(5, 5), 1.0, 1.0, 4);
    Mat mask(image.size(), CV_8UC1); ;
    getColor(blurImg, mask, "yellow");

    
    std::vector<Vec4i> hierarchy1;
    std::vector<std::vector<Point>> contours;
    cv::findContours(mask, contours, hierarchy1, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point());



    int mainContour = maxContour(contours);
    if (contours.size() > 0) {
        //contours[i]代表的是第i个轮廓，contours[i].size()代表的是第i个轮廓上所有的像素点数  

        GetContourCenter(contours[mainContour], center); //寻找轮廓中心
        //绘制轮廓  
        //drawContours(cp1.drawImage, contours, mainContour1, Scalar(255), 1, 8, hierarchy1);
        cv::circle(drawImage, center, 25, { 0,0,255 }, 10, 8, 0);
        return true;
    }
    else {
        return false; // target not found
    }

       

}


// solve the world position of the target. 
//cp1: first CameraPose, point1: target position in Image frame of first camera
//cp2: second CameraPose, point1: target position in Image frame of second camera
cv::Point3f CameraPose::SolveTargetPosition(CameraPose cp1, cv::Point2f point1, CameraPose cp2, cv::Point2f point2) {
    Point3f objCenterWPoint1 = cp1.ImageFrame2WorldFrameDir(point1, 100);
    Point3f objCenterWPoint2 = cp2.ImageFrame2WorldFrameDir(point2, 100);

    Point3f camCenterWPoint1 = cp1.CameraCenter2WorldFrame();
    Point3f camCenterWPoint2 = cp2.CameraCenter2WorldFrame();



    // solve real pos
    GetDistanceOf2linesIn3D psolver;
    psolver.SetLineAB(camCenterWPoint1.x, camCenterWPoint1.y, camCenterWPoint1.z,
        objCenterWPoint1.x, objCenterWPoint1.y, objCenterWPoint1.z);
    psolver.SetLineCD(camCenterWPoint2.x, camCenterWPoint2.y, camCenterWPoint2.z,
        objCenterWPoint2.x, objCenterWPoint2.y, objCenterWPoint2.z);

    psolver.GetDistance();

    Point3f targetPos = Point3f(0.5 * (psolver.PABx + psolver.PCDx), 0.5 * (psolver.PABy + psolver.PCDy), 0.5 * (psolver.PABz + psolver.PCDz));
    cout << targetPos << endl;

    // reprojection
    vector<Point3f> targetPosList(1, targetPos);
    vector<Point2f> targetProjectPoint1(1);
    cv::projectPoints(targetPosList, cp1.rvec, cp1.tvec, cp1.cameraMatrix, cp1.distCoeff, targetProjectPoint1);
    cv::circle(cp1.drawImage, targetProjectPoint1[0], 12, { 0,255,0 });

    vector<Point2f> targetProjectPoint2(1);
    cv::projectPoints(targetPosList, cp2.rvec, cp2.tvec, cp2.cameraMatrix, cp2.distCoeff, targetProjectPoint2);
    cv::circle(cp2.drawImage, targetProjectPoint2[0], 12, { 0,255,0 });

    return targetPos;
}




int CameraPose::ChessboardCalibration(int reqFrames, cv::Size patternSize) {
    /*Size patternSize(5, 14);*/
    int numpoints = chessboardCorner_cali.size();
    int cornerCount = patternSize.width * patternSize.height;
    if (numpoints < patternSize.width * patternSize.height * reqFrames) {
        // points not enough
        Mat grey;
        cvtColor(image, grey, COLOR_RGB2GRAY);
        vector<Point2f> corners;

        bool isuccess = findChessboardCornersSB(grey, patternSize, corners);
        if (!isuccess) return -1; // cannot find chess board
       

        if (idraw) {
            drawChessboardCorners(drawImage, patternSize, corners, true);
        }
        //add new corners to the corner vector
        chessboardCorner_cali.reserve(cornerCount * reqFrames);
        chessboardCorner_cali.insert(chessboardCorner_cali.begin(), corners.begin(), corners.end());
        return 0; //not ready
    }
    else {
        //point enough
        // calculate corners points in world frame of reference
        vector<Point3f> objPoints(cornerCount);
        int n = 0;
        double gridsize = 120.0;
        for (int j = 0; j < patternSize.height; j++) {
            for (int i = 0; i < patternSize.width; i++) {
                objPoints[n].x = i * gridsize;
                objPoints[n].y = j * gridsize;
                objPoints[n].z = 0.0;
                n++;
            }
        }
        vector<Point3f> allobjPoints(0);
        allobjPoints.reserve(reqFrames * cornerCount);
        for (int i = 0; i < reqFrames; i++) {
            //copy world points of corners to the same vector
            allobjPoints.insert(allobjPoints.end(), objPoints.begin(), objPoints.end());
        }

        cv::solvePnPRansac(allobjPoints, chessboardCorner_cali, cameraMatrix, distCoeff, rvec, tvec, cv::noArray());
        Rodrigues(rvec, RoteM);
        //cout << "rvec = " << rvec << endl;
        //cout << "tvec = " << tvec << endl;

        //cout << "RoteM = " << RoteM << endl;

        
        return 1; //success
    }
   

    //if (iprojTest) {
    //    //reprojection test
    //    vector<Point2f> projectedPoints;
    //    cv::projectPoints(objPoints, rvec, tvec, cameraMatrix, distCoeff, projectedPoints);

    //    for (int i = 0; i < patternSize.width * patternSize.height; i++) {
    //        cv::circle(drawImage, projectedPoints[i], 10, (255, 255, 255));
    //    }
    //}
    return true;
}