#include "ImageProcess.h"

using namespace cv;
using namespace std;




void getColor(const cv::Mat& srcimage, cv::Mat& dstimage, const std::string& color) {
    cv::Mat hsv;
    cvtColor(srcimage, hsv, COLOR_BGR2HSV);

    double hmin = 0, smin = 0, vmin = 0;
    double hmax = 0, smax = 0, vmax = 0;

    if (color == "yellow") {
        hmin = 25;
        hmax = 35;
        smin = 120;
        smax = 255;
        vmin = 150;
        vmax = 255;
        //hmin = 25;
        //hmax = 30;
        //smin = 150; 
        //smax = 255;
        //vmin = 150;    
        //vmax = 255;    
        inRange(hsv, Scalar(hmin, smin, vmin), Scalar(hmax, smax, vmax), dstimage);
    }
    else if (color == "yellow_exp") {
        hmin = 30;
        hmax = 50;
        smin = 120;
        smax = 255;
        vmin = 150;
        vmax = 255;
        //hmin = 25;
        //hmax = 30;
        //smin = 150; 
        //smax = 255;
        //vmin = 150;    
        //vmax = 255;    
        inRange(hsv, Scalar(hmin, smin, vmin), Scalar(hmax, smax, vmax), dstimage);
    }
    else if (color == "red") {
        hmin = 0;
        hmax = 8;
        smin = 100;
        smax = 255;
        vmin = 100;
        vmax = 255;

        Mat dst1, dst2;
        inRange(hsv, Scalar(hmin, smin, vmin), Scalar(hmax, smax, vmax), dst1);
        inRange(hsv, Scalar(175, smin, vmin), Scalar(180, smax, vmax), dst2);

        bitwise_or(dst1, dst2, dstimage);
    }
    else {
        cout << "undefined color" << endl;
        return;
    }


}

double EuclidDistance(const cv::Point2f& p1, const cv::Point2f& p2) {
    return abs(p1.x - p2.x) + abs(p1.y - p2.y);
}

double Distance(const cv::Point2f& p1, const cv::Point2f& p2) {
    double d1 = p1.x - p2.x;
    double d2 = p1.y - p2.y;
    return sqrt(d1 * d1 + d2 * d2);
}


int maxContour(const std::vector<std::vector<Point>>& contours) {
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
    return mainContour;
}

void maxContours(const std::vector<std::vector<Point>>& contours, std::vector<int>& maxindices, int maxn) {
    if (contours.size() < maxn) {
        cout << "contour does not have " << maxn << "elements" << endl;
        return;
    }

    maxindices.resize(maxn);
    vector<double> maxvals(contours.size(), DBL_MIN);
    vector<int> idc(contours.size(), 0);
    for (int i = 0; i < contours.size(); i++)
    {
        Moments mu = moments(contours[i], true);
        maxvals[i] = mu.m00;

    }
    sort_indexes<double>(idc, maxvals);
    for (int i = 0; i < maxn; i++) {
        maxindices[i] = idc[idc.size() - 1 - i];
    }

    return;
}

void GetContourCenter(vector<Point> contour, cv::Point2f& p)
{
    //重心法抓中心点
    Moments mu;
    mu = moments(contour, true);
    p = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
}


void ManyImages(vector<Mat> Images, Mat& dst, int imgRows, int imgCols)
{
    int Num = Images.size();//得到Vector容器中图片个数
    if (Num != imgRows * imgCols) {
        std::cout << " imgRows and Cols does not match the number of Images in ManyImages" << std::endl;
        return;
    }
    Mat Window(600 * imgRows, 800 * imgCols, CV_8UC3, Scalar(0, 0, 0));
    //设定包含这些图片的窗口大小，这里都是BGR3通道，如果做灰度单通道，稍微改一下下面这行代码就可以
    //Mat Window(800 * ((Num - 1) / imgRows + 1), 600 * imgRows, CV_8UC3, Scalar(0, 0, 0));
    Mat Std_Image;//存放标准大小的图片
    Mat imageROI;//图片放置区域
    Size Std_Size = Size(800, 600);//每个图片显示大小300*300
    int x_Begin = 0;
    int y_Begin = 0;
    for (int i = 0; i < Num; i++)
    {
        x_Begin = (i % imgCols) * Std_Size.width;//每张图片起始坐标
        y_Begin = (i % imgRows) * Std_Size.height;

        //x_Begin = (i % imgRows) * Std_Size.width;//每张图片起始坐标
        //y_Begin = (i / imgRows) * Std_Size.height;
        resize(Images[i], Std_Image, Std_Size, 0, 0, INTER_LINEAR);//将图像设为标准大小
        //将其贴在Window上
        imageROI = Window(Rect(x_Begin, y_Begin, Std_Size.width, Std_Size.height));
        Std_Image.copyTo(imageROI);
    }
    dst = Window;
}