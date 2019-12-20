#ifndef IMAGEPROCESSOR_H
#define IMAGEPROCESSOR_H
#include <opencv2/opencv.hpp>
#include "common.h"

//using namespace cv;

// 图像处理

class ImageProcessor
{
public:
    ImageProcessor(float percentageOfDeletion);
    cv::Mat stretchHistogram(cv::Mat image);
    cv::Mat unsharpMasking(cv::Mat image, std::string blurMethod, int kernelSize, float alpha, float beta);
    cv::Mat laplacianSharpening(cv::Mat image, int kernelSize, float alpha, float beta);
private:
    float percentageOfDeletion;
};

#endif // IMAGEPROCESSOR_H
