#ifndef AGGREGATION_H
#define AGGREGATION_H
#include <opencv2/opencv.hpp>
#include <omp.h>
#include "common.h"

// using namespace cv;
using namespace std;

// æ€∫œ¿‡

class Aggregation
{
public:
    Aggregation(const cv::Mat &leftImage, const cv::Mat &rightImage, uint colorThreshold1, uint colorThreshold2,
                uint maxLength1, uint maxLength2);
    void aggregation2D(cv::Mat &costMap, bool horizontalFirst, uchar imageNo);
    void getLimits(vector<cv::Mat> &upLimits, vector<cv::Mat> &downLimits, vector<cv::Mat> &leftLimits, vector<cv::Mat> &rightLimits) const;

private:
    cv::Mat images[2];
    cv::Size imgSize;
    uint colorThreshold1, colorThreshold2;
    uint maxLength1, maxLength2;
    vector<cv::Mat> upLimits;
    vector<cv::Mat> downLimits;
    vector<cv::Mat> leftLimits;
    vector<cv::Mat> rightLimits;

    int colorDiff(const cv::Vec3b &p1, const cv::Vec3b &p2);
    int computeLimit(int height, int width, int directionH, int directionW, uchar imageNo);
    cv::Mat computeLimits(int directionH, int directionW, int imageNo);

    cv::Mat aggregation1D(const cv::Mat &costMap, int directionH, int directionW, cv::Mat &windowSizes, uchar imageNo);
};

#endif // AGGREGATION_H
