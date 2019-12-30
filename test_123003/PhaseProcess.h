#ifndef __PHASE_PROCESS_H__
#define __PHASE_PROCESS_H__

#include <opencv2/opencv.hpp>
#include <iostream>
#include <time.h>
#include "CameraCalib.h"

cv::Mat CalWrappedPhase(const std::string &Rect_images);

void UnwrappedPhaseGraycode(cv::Mat& src, cv::Mat& dst, const std::string &Rect_images);

#endif //__PHASE_PROCESS_H__