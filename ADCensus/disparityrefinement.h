#ifndef DISPARITYREFINEMENT_H
#define DISPARITYREFINEMENT_H

#include <opencv2/opencv.hpp>
#include "adcensuscv.h"
#include "common.h"

//视差校正类

// using namespace cv;
using namespace std;

class DisparityRefinement
{
public:
    DisparityRefinement(uint dispTolerance, int dMin, int dMax,
                        uint votingThreshold, float votingRatioThreshold, uint maxSearchDepth,
                        uint blurKernelSize, uint cannyThreshold1, uint cannyThreshold2, uint cannyKernelSize);
    cv::Mat outlierElimination(const cv::Mat &leftDisp, const cv::Mat &rightDisp);
    void regionVoting(cv::Mat &disparity, const vector<cv::Mat> &upLimits, const vector<cv::Mat> &downLimits,
                      const vector<cv::Mat> &leftLimits, const vector<cv::Mat> &rightLimits, bool horizontalFirst);
    void properInterpolation(cv::Mat &disparity, const cv::Mat &leftImage);
    void discontinuityAdjustment(cv::Mat &disparity, const vector<vector<cv::Mat> > &costs);
    cv::Mat subpixelEnhancement(cv::Mat &disparity, const vector<vector<cv::Mat> > &costs);

    static const int DISP_OCCLUSION;
    static const int DISP_MISMATCH;

private:
    int colorDiff(const cv::Vec3b &p1, const cv::Vec3b &p2);
    cv::Mat convertDisp2Gray(const cv::Mat &disparity);

    int occlusionValue;
    int mismatchValue;
    uint dispTolerance;
    int dMin;
    int dMax;
    uint votingThreshold;
    float votingRatioThreshold;
    uint maxSearchDepth;
    uint blurKernelSize;
    uint cannyThreshold1;
    uint cannyThreshold2;
    uint cannyKernelSize;
};

#endif // DISPARITYREFINEMENT_H
