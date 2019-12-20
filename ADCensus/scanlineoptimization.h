#ifndef SCANLINEOPTIMIZATION_H
#define SCANLINEOPTIMIZATION_H

#include <opencv2/opencv.hpp>
#include <omp.h>
#include "common.h"

// using namespace cv;
using namespace std;

// 扫描线优化

class ScanlineOptimization
{
public:
    ScanlineOptimization(const cv::Mat &leftImage, const cv::Mat &rightImage, int dMin, int dMax,
                         uint colorDifference, float pi1, float pi2);
    void optimization(vector<cv::Mat> *costMaps, bool rightFirst);
private:
    cv::Mat images[2];
    cv::Size imgSize;
    int dMin;
    int dMax;
    uint colorDifference;
    float pi1;
    float pi2;

    void verticalComputation(int height, int direction, vector<cv::Mat> *costMaps, bool rightFirst);
    void verticalOptimization(int height1, int height2, vector<cv::Mat> *costMaps, bool rightFirst);

    void horizontalComputation(int width, int direction, vector<cv::Mat> *costMaps, bool rightFirst);
    void horizontalOptimization(int width1, int width2, vector<cv::Mat> *costMaps, bool rightFirst);

    void partialOptimization(int height1, int height2, int width1, int width2, vector<cv::Mat> *costMaps, bool rightFirst);

    void computeP1P2(int height1, int height2, int width1, int width2, int disparity, float &p1, float &p2, bool rightFirst);

    int colorDiff(const cv::Vec3b &p1, const cv::Vec3b &p2) const;
};

#endif // SCANLINEOPTIMIZATION_H
