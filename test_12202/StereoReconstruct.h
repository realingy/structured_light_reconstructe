#ifndef __STEREORESTRUCT_H__
#define __STEREORESTRUCT_H__

#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;
using namespace cv;

void find_featurepionts(Mat& leftphase, Mat& rightphase, vector<Point2f>& leftkeypoint, vector<Point2f>& rightkeypoint);
void find_featurepionts_single_match(Mat& leftphase, Mat& rightphase, vector<Point2f>& leftkeypoint, vector<Point2f>& rightkeypoint);
void find_featureSAD(Mat& leftphase, Mat& rightphase);
void find_featureBlock(Mat& leftphase, Mat& rightphase, vector<Point2f>& leftkeypoint, vector<Point2f>& rightkeypoint);

#endif	// __STEREORESTRUCT_H__