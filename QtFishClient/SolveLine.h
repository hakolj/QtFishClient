#pragma once
//用于求解两条空间直线的最近距离，以及他们最近的两点坐标
//author @vshawn
//url:http://www.cnblogs.com/singlex/p/6091659.html
//date:2016-11-22
#include <math.h>
class GetDistanceOf2linesIn3D
{
public:
    //输入直线A的两个点，以便获得A的方程
    void SetLineAB(double ax, double ay, double az, double bx, double by, double bz)
    {
        this->ax = ax;
        this->ay = ay;
        this->az = az;

        this->bx = bx;
        this->by = by;
        this->bz = bz;
    }

    //输入直线B的两个点，以便获得B的方程
    void SetLineCD(double cx, double cy, double cz, double dx, double dy, double dz)
    {
        this->cx = cx;
        this->cy = cy;
        this->cz = cz;

        this->dx = dx;
        this->dy = dy;
        this->dz = dz;
    }

    //用SetLineA、SetLineB输入A、B方程后
    //调用本函数解出结果
    void GetDistance()
    {
        //方法来自：http://blog.csdn.net/pi9nc/article/details/11820545

        double d1_x = bx - ax;
        double d1_y = by - ay;
        double d1_z = bz - az;

        double d2_x = dx - cx;
        double d2_y = dy - cy;
        double d2_z = dz - cz;

        double dpx = cx - ax;
        double dpy = cy - ay;
        double dpz = cz - az;


        double cross_e_d2_x, cross_e_d2_y, cross_e_d2_z;
        cross(dpx, dpy, dpz, d2_x, d2_y, d2_z, cross_e_d2_x, cross_e_d2_y, cross_e_d2_z);
        double cross_e_d1_x, cross_e_d1_y, cross_e_d1_z;
        cross(dpx, dpy, dpz, d1_x, d1_y, d1_z, cross_e_d1_x, cross_e_d1_y, cross_e_d1_z);
        double cross_d1_d2_x, cross_d1_d2_y, cross_d1_d2_z;
        cross(d1_x, d1_y, d1_z, d2_x, d2_y, d2_z, cross_d1_d2_x, cross_d1_d2_y, cross_d1_d2_z);

        double t1, t2;
        t1 = dot(cross_e_d2_x, cross_e_d2_y, cross_e_d2_z, cross_d1_d2_x, cross_d1_d2_y, cross_d1_d2_z);
        t2 = dot(cross_e_d1_x, cross_e_d1_y, cross_e_d1_z, cross_d1_d2_x, cross_d1_d2_y, cross_d1_d2_z);
        double dd = norm(cross_d1_d2_x, cross_d1_d2_y, cross_d1_d2_z);
        t1 /= dd * dd;
        t2 /= dd * dd;

        //得到最近的位置
        PABx = (ax + (bx - ax) * t1);
        PABy = (ay + (by - ay) * t1);
        PABz = (az + (bz - az) * t1);

        PCDx = (cx + (dx - cx) * t2);
        PCDy = (cy + (dy - cy) * t2);
        PCDz = (cz + (dz - cz) * t2);

        distance = norm(PCDx - PABx, PCDy - PABy, PCDz - PABz);
    }



    double PABx;//两直线最近点之AB线上的点的坐标
    double PABy;//
    double PABz;//
    double PCDx;//两直线最近点之CD线上的点的x坐标
    double PCDy;//
    double PCDz;//
    double distance;//两直线距离
private:
    //直线AB的第一个点
    double ax;
    double ay;
    double az;
    //直线AB的第二个点
    double bx;
    double by;
    double bz;

    //直线CD的第一个点
    double cx;
    double cy;
    double cz;

    //直线CD的第二个点
    double dx;
    double dy;
    double dz;


    //点乘
    double dot(double ax, double ay, double az, double bx, double by, double bz) { return ax * bx + ay * by + az * bz; }
    //向量叉乘得到法向量，最后三个参数为输出参数
    void cross(double ax, double ay, double az, double bx, double by, double bz, double& x, double& y, double& z)
    {
        x = ay * bz - az * by;
        y = az * bx - ax * bz;
        z = ax * by - ay * bx;
    }
    //向量取模
    double norm(double ax, double ay, double az) { return sqrt(dot(ax, ay, az, ax, ay, az)); }
};