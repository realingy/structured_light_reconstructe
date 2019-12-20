#ifndef __STEREO_RECONSTRUCT_H__
#define __STEREO_RECONSTRUCT_H__

#include <iostream>
#include <opencv2/opencv.hpp>
#include <boost/format.hpp>
#include <vector>

using namespace cv;
using namespace std;

void savePhase(const char* filename, Mat& mat);
void savepnts3D(const char* filename,  Mat& mat);
void find_featurepionts(Mat& leftphase, Mat& rightphase, vector<Point2f>& leftkeypoint, vector<Point2f>& rightkeypoint);
void find_featurepionts_single_match(Mat& leftphase, Mat& rightphase, vector<Point2f>& leftkeypoint, vector<Point2f>& rightkeypoint);
void find_featureSAD(Mat& leftphase, Mat& rightphase);
void find_featureBlock(Mat& leftphase, Mat& rightphase, vector<Point2f>& leftkeypoint, vector<Point2f>& rightkeypoint);

#endif