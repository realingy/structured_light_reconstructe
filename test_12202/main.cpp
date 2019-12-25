#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <time.h>
#include <string>
#include "CameraCalib.h"
#include "PhaseProcess.h"
#include "PointcloudProcess.h"
#include "StereoReconstruct.h"
#include "FileManager.h"

using namespace cv;
using namespace std;

#if 1
int main(int argc, char **argv)
{
	/*******************************Stereo calibration*****************************************/
#if 0
	cout << "\n======================================================" << endl;
	const string Calibimagelistfn = "../input/stereo_calib_images.xml";

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
	const string Phaseimageslistfn = "../input/phase_images.xml";
	const string Rectifiedimageslistfn = "../input/Rect_images.xml";

	ImgRectified(storintrinsicsyml, storextrinsicsyml, Phaseimageslistfn, Rectifiedimageslistfn);
#endif

	/*******************************Calculate unwrapped phase*****************************************/
#if 1
	// Calculate unwrapped phase
	
	cout << "\n======================================================" << endl;
	cout << ">>>3 Calculate phase" << endl;

	const char* wrapped_phaseleft_txt = "../result/wrapped_phase_left.txt";
	const char* wrapped_phaseright_txt = "../result/wrapped_phase_right.txt";
	const char* unwrapped_phaseleft_txt = "../result/unwrapped_phase_left.txt";
	const char* unwrapped_phaseright_txt = "../result/unwrapped_phase_right.txt";
	const char* Rect_images_left = "../input/Rect_images_left.xml";
	const char* Rect_images_right = "../input/Rect_images_right.xml";

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

	UnwrappedPhaseGraycode(wrapped_phase_left, unwrapped_phase_left, Rect_images_left);

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

	UnwrappedPhaseGraycode(wrapped_phase_right, unwrapped_phase_right, Rect_images_right);

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

	//find_featurepionts(unwrapped_phase_left, unwrapped_phase_right, leftfeaturepoints, rightfeaturepoints);
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

	Mat points(4, leftfeaturepoints.size(), CV_64F);

	cout << "\n[2] Calculate points3D" << endl;
	cv::triangulatePoints(P1, P2, leftfeaturepoints, rightfeaturepoints, points);

    cout << "\n[3] Save points3D" <<endl;
	// const char* pnts3d_txt = "../result/points.txt";
    // savepnts3D( pnts3d_txt, points);
    savepointcloud(points);

	cout << "Stereo match and 3D reconstruct successful!\n";
#endif    

	/*****************************Surface reconstruction************************************/
	cout << "\n======================================================" << endl;
	cout << ">>>5 Point cloud filte and Surface reconstruction" <<endl;
	filterpointcloud();
	//poissonreconstruction(); // 泊松曲面重建
    
	cout << "\n======================================================" << endl;
	cout << endl << ">>>";
	cout << "All Done" <<endl;
    
    return 0;
}
#endif

/*
typedef pair<string, int> PAIR;

struct CmpByValue {
	bool operator()(const PAIR& lhs, const PAIR& rhs) {
		return lhs.second < rhs.second;
	}
};

int main() {
	map<string, int> name_score_map;
	name_score_map["LiMin"] = 90;
	name_score_map["ZiLinMi"] = 79;
	name_score_map["BoB"] = 92;
	name_score_map.insert(make_pair("Bing", 99));
	name_score_map.insert(make_pair("Albert", 86));
	//把map中元素转存到vector中   
	vector<PAIR> name_score_vec(name_score_map.begin(), name_score_map.end());
	sort(name_score_vec.begin(), name_score_vec.end(), CmpByValue());
	for (int i = 0; i != name_score_vec.size(); ++i) {
		cout << name_score_vec[i].first << endl;
	}
	return 0;
}
*/

/*
typedef pair<string, int> PAIR;

ostream& operator<<(ostream& out, const PAIR& p) {
	return out << p.first << "\t" << p.second;
}

int main() {
	map<string, int> name_score_map;
	name_score_map["LiMin"] = 90;
	name_score_map["ZiLin"] = 79;
	name_score_map["BoB"] = 92;
	name_score_map.insert(make_pair("Bing", 99));
	name_score_map.insert(make_pair("Albert", 86));

	for (auto& x: name_score_map) {
		std::cout << x.first << ":\t" << x.second << '\n';
	}

#if 0
	for (map<string, int>::iterator iter = name_score_map.begin(); iter != name_score_map.end(); ++iter)
	{
		cout << *iter << endl;
	}
#endif
	return 0;
}
*/



