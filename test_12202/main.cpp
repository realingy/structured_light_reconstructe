#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <time.h>
#include <string>
#include "CameraCalib.h"
#include "CalPhase.h"
#include "PointcloudProcess.h"
#include "StereoReconstruct.h"
#include "FileManager.h"

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
	/*******************************Stereo calibration*****************************************/
#if 0
	cout << "\n======================================================" << endl;
	const string Calibimagelistfn = "../mydata/input/stereo_calib_images.xml";

	cout << ">>>1 Stereo Calibration" << endl;

	//clock_t start=0, end=0;
	//start = clock();  //开始计时     

	//根据标定图像进行相机内外参的计算
	//StereoCalibration(Calibimagelistfn, storintrinsicsyml, storextrinsicsyml);

	//根据Matlab标定得到的内参数据填充内参文件，进行外参的计算（相机位置校正）
	StereoCalibration2(storintrinsicsyml, storextrinsicsyml);

	//end = clock(); //计时结束

	//double elapsed_secs = double(end - start) / CLOCKS_PER_SEC;
	//printf("Done in %.2lf seconds.\n", elapsed_secs);

#endif

#if 0
	cout << "\n======================================================" << endl;
	// images Rectified
	cout << ">>>2 Image Rectify" << endl;
	const string Phaseimageslistfn = "../mydata/input/phase_images.xml";
	const string Rectifiedimageslistfn = "../mydata/input/Rect_images.xml";

	ImgRectified(storintrinsicsyml, storextrinsicsyml, Phaseimageslistfn, Rectifiedimageslistfn);
#endif

	/*******************************Calculate unwrapped phase*****************************************/
#if 1
	// Calculate unwrapped phase
	
	cout << "\n======================================================" << endl;
	cout << ">>>3 Calculate phase" << endl;

	const char* wrapped_phaseleft_txt = "../mydata/output/wrapped_phase_left.txt";
	const char* wrapped_phaseright_txt = "../mydata/output/wrapped_phase_right.txt";
	const char* unwrapped_phaseleft_txt = "../mydata/output/unwrapped_phase_left.txt";
	const char* unwrapped_phaseright_txt = "../mydata/output/unwrapped_phase_right.txt";
	const char* Rect_images_left = "../mydata/input/Rect_images_left.xml";
	const char* Rect_images_right = "../mydata/input/Rect_images_right.xml";

	//Calculate left phase
	cout << "\n[1] Calculate left phase" << endl;

	Mat wrapped_phase_left = CalWrappedPhase(Rect_images_left).clone();

	if (wrapped_phaseleft_txt)
	{
		cout << "storing the wrapped_phaseleft_txt" << endl;
		savePhase(wrapped_phaseleft_txt, wrapped_phase_left);
	}

	Mat unwrapped_phase_left(Size(wrapped_phase_left.cols, wrapped_phase_left.rows), CV_32FC1, Scalar(0.0));
	cout << "Phase unwrapping..." << endl;

	UnwrappedPhaseGraycodeMethod(wrapped_phase_left, unwrapped_phase_left, Rect_images_left);
	// UnwrappedPhaseClassicMethod(wrapped_phase_left, unwrapped_phase_left);

	if (unwrapped_phaseleft_txt)
	{
		cout << "storing the unwrapped_phaseleft_txt" << endl;
		savePhase(unwrapped_phaseleft_txt, unwrapped_phase_left);
	}

	cout << "storing the unwrapped_phase_image_left" << endl;
	imwrite(unwrapped_phase_image_left, unwrapped_phase_left);

	/*************************Calculate right phase***********************************/
	cout << "\n[2] Calculate right phase" << endl;
	Mat wrapped_phase_right = CalWrappedPhase(Rect_images_right).clone();
	Mat unwrapped_phase_right(Size(wrapped_phase_right.cols, wrapped_phase_right.rows), CV_32FC1, Scalar(0.0));  // warning SIZE(cols,rows)!!!

	if (wrapped_phaseright_txt)
	{
		cout << "storing the wrapped_phaseright_txt" << endl;
		savePhase(wrapped_phaseright_txt, wrapped_phase_right);
	}

	cout << "Phase unwrapping..." << endl;

	UnwrappedPhaseGraycodeMethod(wrapped_phase_right, unwrapped_phase_right, Rect_images_right);
	// UnwrappedPhaseClassicMethod(wrapped_phase_right, unwrapped_phase_right);

	if (unwrapped_phaseright_txt)
	{
		cout << "storing the unwrapped_phaseright_txt" << endl;
		savePhase(unwrapped_phaseright_txt, unwrapped_phase_right);
	}

	cout << "storing the unwrapped_phase_image_right" << endl;
	imwrite(unwrapped_phase_image_right, unwrapped_phase_right);

	cout << endl << "Calculate phase successful!" << endl;
#endif

	/*****************************Stereo matching and 3D reconstruction************************************/
#if 1
	vector<Point2f> leftfeaturepoints, rightfeaturepoints;
	cout << "\n======================================================" << endl;
	cout << ">>>4 Stereo match and 3D reconstruct" << endl;
	cout << "\n[1] Calculate feature points" << endl;

	// find_featurepionts(unwrapped_phase_left, unwrapped_phase_right, leftfeaturepoints, rightfeaturepoints);
	find_featurepionts_single_match(unwrapped_phase_left, unwrapped_phase_right, leftfeaturepoints, rightfeaturepoints);

	cout << "the number of feature: " << leftfeaturepoints.size() << endl;

	FileStorage fs(storextrinsicsyml, FileStorage::READ);
	if (!fs.isOpened())
	{
		printf("Failed to open file extrinsics_filename.\n");
		return 0;
	}

	Mat P1, P2;
	fs["P1"] >> P1;
	fs["P2"] >> P2;

	//cout << "\n==> P1:\n" << P1 << endl;
	//cout << "\n==> P2:\n" << P2 << endl;

	Mat pnts3D(4, leftfeaturepoints.size(), CV_64F);

	cout << "\n[2] Calculate points3D" << endl;
	cv::triangulatePoints(P1, P2, leftfeaturepoints, rightfeaturepoints, pnts3D);

    cout << "\n[3] Save points3D" <<endl;
	const char* pnts3D_filename = "../mydata/output/pnts3D.txt";
    savepnts3D( pnts3D_filename, pnts3D);
    savepntsPCD(pnts3D);

	cout << "Stereo match and 3D reconstruct successful!\n";

#endif    

	/*****************************Surface reconstruction************************************/
	cout << "\n======================================================" << endl;
	cout << ">>>5 Surface reconstruction" <<endl;
	// filterpointcloud();
	// poissonreconstruction(); // 泊松曲面重建
    
	cout << endl << ">>>";
	cout << "All Done" <<endl;
    
    return 0;
}






