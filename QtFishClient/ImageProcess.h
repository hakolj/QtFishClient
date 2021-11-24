#pragma once
#include <opencv.hpp>
#include <vector>

extern void getColor(const cv::Mat& srcimage, cv::Mat& dstimage, const std::string& color);
extern double EuclidDistance(const cv::Point2f& p1, const cv::Point2f& p2);
extern double Distance(const cv::Point2f& p1, const cv::Point2f& p2);

extern void GetContourCenter(std::vector<cv::Point> contour, cv::Point2f& p);

extern int maxContour(const std::vector<std::vector<cv::Point>>& contours);
extern void maxContours(const std::vector<std::vector<cv::Point>>& contours, std::vector<int>& maxindices, int maxn);


extern void ManyImages(std::vector<cv::Mat> Images, cv::Mat& dst, int imgRows, int imgCols);

template <typename T>
T sort_indexes(std::vector<int>& idx, std::vector<T>& v)
{
    class node
    {
    public:
        int value;
        int index;


        static bool cmp(struct node a, struct node b)
        {
            if (a.value < b.value)
            {
                return true;
            }
            return false;
        }

    };



    node* a = new node[v.size()];
    for (int i = 0; i < v.size(); i++)
    {
        a[i].value = v[i];
        a[i].index = i;
    }

    std::sort(a, a + v.size(), node::cmp);
    for (int i = 0; i < v.size(); i++)
    {
        idx.push_back(a[i].index);
    }
    delete[] a;

    return 0;
};
