#ifndef  __CAMERACALIB_H
#define  __CAMERACALIB_H

#include <iostream>
#include <opencv2/opencv.hpp>

// 读取文件列表
bool readStringList( const std::string& filename, std::vector<std::string>& l);
// 立体标定
void StereoCalibration(const std::string &imagelistfn, const std::string &storintrinsicsyml, 
		       const std::string &storextrinsicsyml);
// 图片微调
void ImgRectified(const std::string& intrinsic_filename, const std::string& extrinsic_filename, 
		  const std::string& imageListfn, const std::string& RectimageListfn);

// 立体标定加相机位置校正(基础Matlab的标定结果)
void StereoCalibrationByParams(const std::string &storintrinsicsyml, const std::string &storextrinsicsyml);
	
void StereoCalibrationByCorner(const std::string &storintrinsicsyml, const std::string &storextrinsicsyml);

#endif