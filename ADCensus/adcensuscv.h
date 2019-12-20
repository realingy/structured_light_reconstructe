#ifndef ADCENSUSCV_H
#define ADCENSUSCV_H

#include <opencv2/opencv.hpp>

// using namespace cv;

// AD-Census匹配代价计算

class ADCensusCV
{
public:
    ADCensusCV(const cv::Mat &leftImage, const cv::Mat &rightImage, cv::Size censusWin, float lambdaAD, float lambdaCensus);
    float ad(int wL, int hL, int wR, int hR) const; //ad匹配代价
    float census(int wL, int hL, int wR, int hR) const; //census匹配代价
    float adCensus(int wL, int hL, int wR, int hR) const; //ad-census匹配代价

private:
    cv::Mat leftImage;
    cv::Mat rightImage;
    cv::Size censusWin;
    float lambdaAD;
    float lambdaCensus;
};

#endif // ADCENSUSCV_H
