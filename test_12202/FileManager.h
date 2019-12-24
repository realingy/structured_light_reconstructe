#ifndef __FILEMANAGER_H__
#define __FILEMANAGER_H__

#include <string>
#include <opencv2/core.hpp>

using namespace std;

const string storintrinsicsyml  = "./source_data/output/intrinsics.yml";
const string storextrinsicsyml  = "./source_data/output/extrinsics.yml";

const string unwrapped_phase_image_left = "./source_data/output/unwrapped_phase_left.bmp";
const string unwrapped_phase_image_right = "./source_data/output/unwrapped_phase_right.bmp";


void savepnts3D(const char *filename, cv::Mat& mat);
void savePhase(const char* filename, cv::Mat& mat);

#endif	//__FILEMANAGER_H__
