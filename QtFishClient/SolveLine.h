#pragma once
//������������ռ�ֱ�ߵ�������룬�Լ������������������
//author @vshawn
//url:http://www.cnblogs.com/singlex/p/6091659.html
//date:2016-11-22
#include <math.h>
class GetDistanceOf2linesIn3D
{
public:
    //����ֱ��A�������㣬�Ա���A�ķ���
    void SetLineAB(double ax, double ay, double az, double bx, double by, double bz)
    {
        this->ax = ax;
        this->ay = ay;
        this->az = az;

        this->bx = bx;
        this->by = by;
        this->bz = bz;
    }

    //����ֱ��B�������㣬�Ա���B�ķ���
    void SetLineCD(double cx, double cy, double cz, double dx, double dy, double dz)
    {
        this->cx = cx;
        this->cy = cy;
        this->cz = cz;

        this->dx = dx;
        this->dy = dy;
        this->dz = dz;
    }

    //��SetLineA��SetLineB����A��B���̺�
    //���ñ�����������
    void GetDistance()
    {
        //�������ԣ�http://blog.csdn.net/pi9nc/article/details/11820545

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

        //�õ������λ��
        PABx = (ax + (bx - ax) * t1);
        PABy = (ay + (by - ay) * t1);
        PABz = (az + (bz - az) * t1);

        PCDx = (cx + (dx - cx) * t2);
        PCDy = (cy + (dy - cy) * t2);
        PCDz = (cz + (dz - cz) * t2);

        distance = norm(PCDx - PABx, PCDy - PABy, PCDz - PABz);
    }



    double PABx;//��ֱ�������֮AB���ϵĵ������
    double PABy;//
    double PABz;//
    double PCDx;//��ֱ�������֮CD���ϵĵ��x����
    double PCDy;//
    double PCDz;//
    double distance;//��ֱ�߾���
private:
    //ֱ��AB�ĵ�һ����
    double ax;
    double ay;
    double az;
    //ֱ��AB�ĵڶ�����
    double bx;
    double by;
    double bz;

    //ֱ��CD�ĵ�һ����
    double cx;
    double cy;
    double cz;

    //ֱ��CD�ĵڶ�����
    double dx;
    double dy;
    double dz;


    //���
    double dot(double ax, double ay, double az, double bx, double by, double bz) { return ax * bx + ay * by + az * bz; }
    //������˵õ��������������������Ϊ�������
    void cross(double ax, double ay, double az, double bx, double by, double bz, double& x, double& y, double& z)
    {
        x = ay * bz - az * by;
        y = az * bx - ax * bz;
        z = ax * by - ay * bx;
    }
    //����ȡģ
    double norm(double ax, double ay, double az) { return sqrt(dot(ax, ay, az, ax, ay, az)); }
};