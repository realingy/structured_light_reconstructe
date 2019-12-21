#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <math.h>
#include <time.h>
#include <string>
#include "CameraCalib.h"
#include "CalPhase.h"
#include "Propointcloud.h"
#include "StereoReconstruct.h"

using namespace cv;
using namespace std;

//void Triangulate(Mat projMatr1, Mat projMatr2, const vector<Point2f> & projPoints1, const vector<Point2f> projPoints2, Mat points4D);
	
const string storintrinsicsyml  = "../data/output/intrinsics.yml";
const string storextrinsicsyml  = "../data/output/extrinsics.yml";

int main(int argc, char **argv) 
{
/***********************Stereo Calibration*****************************************/
#if 0
	const string Calibimagelistfn = "../data/input/stereo_calib_images.xml";  
     
	cout << "Stereo Calibration......" <<endl;
     
	//clock_t start=0, end=0;
	//start = clock(); //开始计时     
     
	StereoCalibration(Calibimagelistfn, storintrinsicsyml, storextrinsicsyml);
     
	//end = clock(); //计时结束
	//double elapsed_secs = double(end - start) / CLOCKS_PER_SEC;
	//printf("Done in %.2lf seconds.\n", elapsed_secs);
#endif

#if 0
	const string Phaseimages = "../data/input/phase_images.xml";
	const string Rectifiedimages = "../data/input/Rect_phase_images.xml";
     
	ImgRectified(storintrinsicsyml, storextrinsicsyml, Phaseimages, Rectifiedimages);
#endif

#if 0
    const char* wrapped_phaseleft_txt = "../data/output/wrapped_phase_left.txt";
    const char* wrapped_phaseright_txt = "../data/output/wrapped_phase_right.txt";
    const char* unwrapped_phaseleft_txt = "../data/output/unwrapped_phase_left.txt";
    const char* unwrapped_phaseright_txt = "../data/output/unwrapped_phase_right.txt";

    const char* Rect_images_left = "../data/input/Rect_images_left.xml";
    const char* Rect_images_right = "../data/input/Rect_images_right.xml";
    
	cout << endl << "Process Left Image: \n";
    Mat wrapped_phase_left = CalWrappedPhase(Rect_images_left).clone();

    if(wrapped_phaseleft_txt)
    {
		printf("storing the wrapped_phaseleft_txt...\n");
		//cv::normalize(wrapped_phase_left, wrapped_phase_left, -128, 127, NORM_MINMAX);
		savePhase(wrapped_phaseleft_txt, wrapped_phase_left); //保存相位
    }

    cout << "Phase unwrapping......" <<endl;
	Mat unwrapped_phase_left(Size(wrapped_phase_left.cols, wrapped_phase_left.rows), CV_32FC1, Scalar(0.0));
	Mat series_phase_left(Size(wrapped_phase_left.cols, wrapped_phase_left.rows), CV_32FC1, Scalar(0.0));
    UnwrappedPhaseGraycodeMethod(wrapped_phase_left, unwrapped_phase_left, Rect_images_left);
	// UnwrappedPhaseClassicMethod(wrapped_phase_left, unwrapped_phase_left);

    if(unwrapped_phaseleft_txt)
    {
		printf("\nstoring the unwrapped_phaseleft_txt...");
		cv::normalize(unwrapped_phase_left, unwrapped_phase_left, 0, 255, NORM_MINMAX);
		savePhase(unwrapped_phaseleft_txt, unwrapped_phase_left);
    }

	// 灰度归一化
	cv::normalize(wrapped_phase_left, wrapped_phase_left, 0, 255, NORM_MINMAX);
	cv::imwrite("../data/output/wrapped_phase_left.jpg", wrapped_phase_left);
	cv::imwrite("../data/output/unwrapped_phase_left.jpg", unwrapped_phase_left);

	cout << endl << endl << "Process Right Image: \n";
    Mat wrapped_phase_right = CalWrappedPhase(Rect_images_right).clone();
    Mat unwrapped_phase_right(Size(wrapped_phase_right.cols, wrapped_phase_right.rows), CV_32FC1, Scalar(0.0));
      
    if(wrapped_phaseright_txt)
    {
		printf("storing the wrapped_phaseright_txt...\n");
		savePhase(wrapped_phaseright_txt, wrapped_phase_right);
    }
    
    cout << "Phase unwrapping......" <<endl;
    UnwrappedPhaseGraycodeMethod(wrapped_phase_right, unwrapped_phase_right, Rect_images_right);
	// UnwrappedPhaseClassicMethod(wrapped_phase_right, unwrapped_phase_right);

    if(unwrapped_phaseright_txt)
    {
		printf("\nstoring the unwrapped_phaseright_txt...\n");
		cv::normalize(unwrapped_phase_right, unwrapped_phase_right, 0, 255, NORM_MINMAX);
		savePhase(unwrapped_phaseright_txt, unwrapped_phase_right);
    }
		
	// 灰度归一化
	cv::normalize(wrapped_phase_right, wrapped_phase_right, 0, 255, NORM_MINMAX);
	cv::imwrite("../data/output/wrapped_phase_right.jpg", wrapped_phase_right);
	cv::imwrite("../data/output/unwrapped_phase_right.jpg", unwrapped_phase_right);
#endif

	// Float, 1-channel gray image
	Mat unwrapped_phase_left  = cv::imread("../images/unpahse0.bmp", IMREAD_GRAYSCALE);
	Mat unwrapped_phase_right = cv::imread("../images/unpahse1.bmp", IMREAD_GRAYSCALE);
	Mat tmp0 = unwrapped_phase_left;
	Mat tmp1 = unwrapped_phase_right;
	unwrapped_phase_left.convertTo(unwrapped_phase_left, CV_32F); // convert to float
	unwrapped_phase_right.convertTo(unwrapped_phase_right, CV_32F); // convert to float

	//cv::Mat_<float> unwrapped_phase_left = imread("../images/unpahse0.bmp", CV_LOAD_IMAGE_GRAYSCALE);
	//cv::Mat_<float> unwrapped_phase_right = imread("../images/unpahse1.bmp", CV_LOAD_IMAGE_GRAYSCALE);

	//cv::Mat_<float> unwrapped_phase_left = imread("../data/output/unwrapped_phase_left.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	//cv::Mat_<float> unwrapped_phase_right = imread("../data/output/unwrapped_phase_right.jpg", CV_LOAD_IMAGE_GRAYSCALE);

	if ( !unwrapped_phase_left.data || !unwrapped_phase_right.data )
		cout << "imread error!\n";

	cout << "=== " << unwrapped_phase_left.channels() << endl;
	cout << "=== " << unwrapped_phase_right.channels() << endl;
	cout << "=== " << unwrapped_phase_left.type() << endl;
	cout << "=== " << unwrapped_phase_right.type() << endl;

	//imshow("unphase0", unwrapped_phase_left);
	//imshow("unphase1", unwrapped_phase_right);

	//cv::waitKey(0);
 
#if 1
	// stereo matching and 3D reconstruction
    
    FileStorage fs(storextrinsicsyml, FileStorage::READ);
    if(!fs.isOpened())
    {
		printf("Failed to open file extrinsics_filename.\n");
		return 0;
    }

    Mat P1, P2;
    fs["P1"] >> P1;
    fs["P2"] >> P2;
    
    vector<Point2f> leftfeaturepoints, rightfeaturepoints; 
    cout << "Calculate feature points......"<<endl;

	// 特征点匹配
    find_featurepionts(unwrapped_phase_left, unwrapped_phase_right, leftfeaturepoints, rightfeaturepoints);
    // find_featurepionts_single_match(unwrapped_phase_left, unwrapped_phase_right, leftfeaturepoints, rightfeaturepoints);
	// find_featureBlock(unwrapped_phase_left, unwrapped_phase_right, leftfeaturepoints, rightfeaturepoints);
	// find_featureSAD(unwrapped_phase_left, unwrapped_phase_right);

	cout << "the number of feature: " << leftfeaturepoints.size() << " <==> " << leftfeaturepoints.size() <<endl;

	vector<KeyPoint> keypoint_left, keypoint_right;
	KeyPoint::convert(leftfeaturepoints, keypoint_left, 1, 1, 0, -1);
	KeyPoint::convert(rightfeaturepoints, keypoint_right, 1, 1, 0, -1);

	Mat image_left, image_right;
	// unwrapped_phase_left.convertTo(unwrapped_phase_left, CV_8U);
	// unwrapped_phase_right.convertTo(unwrapped_phase_right, CV_8U);

	/*
	Mat tmp0;
	tmp0.create(unwrapped_phase_left.rows, unwrapped_phase_left.cols, CV_8UC1);
	//src.copyTo(image);
	int fromTo1[] = { 0, 0, 1, 1, 2, 2 };
	mixChannels(&unwrapped_phase_left, 1, &tmp0, 1, fromTo1, 3);
	*/

	cv::drawKeypoints(tmp0, keypoint_left, image_left);
	cv::drawKeypoints(tmp1, keypoint_right, image_right);

	cv::imwrite("../images/image_left.png", image_left);
	cv::imwrite("../images/image_right.png", image_right);

	vector<DMatch> good_matches;
	Mat image_out;

	/*
	cv::drawMatches();
	cv::drawMatches(tmp0, keypoint_left, tmp1, keypoint_right, good_matches, image_out, Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
	*/

	while (1) {}

	Mat pnts3D(4, leftfeaturepoints.size(), CV_64F);
	// Mat pnts3D(3, leftfeaturepoints.size(), CV_64F);

    cout << "Calculate points3D......"<<endl;

	// 通过三角测量计算三维坐标(基于世界坐标和图像坐标的转换关系)
    // cv::triangulatePoints(T1, T2, leftfeaturepoints, rightfeaturepoints, pnts3D);

	// P1，P2是两个相机相对于校正平面的转换矩阵
    cv::triangulatePoints(P1, P2, leftfeaturepoints, rightfeaturepoints, pnts3D);

    const char* pnts3D_filename = "../data/output/pnts3D.txt";

    cout << "Save points3D......" <<endl;
    savepnts3D(pnts3D_filename, pnts3D);

    savepntsPCD(pnts3D);

#endif    

#if 0
	// surface reconstruction
	cout << "surface reconstruction......" <<endl;
	filterpointcloud();
	//poissonreconstruction(); // 泊松曲面重建
#endif
    
	return 0;
}

#if 0
/*****************************************************************
// projMatr11 相机1的世界坐标和相机坐标转换矩阵 3*4
// projMatr12 相机2的世界坐标和相机坐标转换矩阵 3*4
// projPoints1 相机1的特征点矩阵 n
// projPoints2 相机2的特征点矩阵 n
*****************************************************************/
void Triangulate(Mat m1, Mat m2, const vector<Point2f> & projPoints1, const vector<Point2f> projPoints2, Mat points3D)
{
	int size = projPoints1.size(); // 特征点数
	for (size_t i = 0; i < size; i++)
	{
		// 坐标
		Point2f P1 = projPoints1[i];
		Point2f P2 = projPoints2[i];
		float u1 = (float)P1.x;
		float v1 = (float)P1.y;
		float u2 = (float)P2.x;
		float v2 = (float)P2.y;

		/*
		Mat Tran = (Mat_<float>(4, 3) <<
			u1 * m1.at<float>(2, 0) - m1.at<float>(0, 0), u1 * m1.at<float>(2, 1) - m1.at<float>(0, 1), u1 * m1.at<float>(2, 2) - m1.at<float>(0, 2),
			v1 * m1.at<float>(2, 0) - m1.at<float>(1, 0), v1 * m1.at<float>(2, 1) - m1.at<float>(1, 1), v1 * m1.at<float>(2, 2) - m1.at<float>(1, 2),
			u2 * m2.at<float>(2, 0) - m2.at<float>(0, 0), u2 * m2.at<float>(2, 1) - m2.at<float>(0, 1), u2 * m2.at<float>(2, 2) - m2.at<float>(0, 2),
			v2 * m2.at<float>(2, 0) - m2.at<float>(1, 0), v2 * m2.at<float>(2, 1) - m2.at<float>(1, 1), v2 * m2.at<float>(2, 2) - m2.at<float>(1, 2));
		*/

		Mat A = (Mat_<float>(3, 3) <<
			u1 * m1.at<float>(2, 0) - m1.at<float>(0, 0), u1 * m1.at<float>(2, 1) - m1.at<float>(0, 1), u1 * m1.at<float>(2, 2) - m1.at<float>(0, 2),
			v1 * m1.at<float>(2, 0) - m1.at<float>(1, 0), v1 * m1.at<float>(2, 1) - m1.at<float>(1, 1), v1 * m1.at<float>(2, 2) - m1.at<float>(1, 2),
			u2 * m2.at<float>(2, 0) - m2.at<float>(0, 0), u2 * m2.at<float>(2, 1) - m2.at<float>(0, 1), u2 * m2.at<float>(2, 2) - m2.at<float>(0, 2));

		/*
		Mat RES = (Mat_<float>(4, 1) <<
			m1.at<float>(0, 3) - u1 * m1.at<float>(2, 3),
			m1.at<float>(1, 3) - v1 * m1.at<float>(2, 3),
			m2.at<float>(0, 3) - u2 * m2.at<float>(2, 3),
			m2.at<float>(1, 2) - v2 * m2.at<float>(2, 3));
		*/

		Mat B = (Mat_<float>(3, 1) <<
			m1.at<float>(0, 3) - u1 * m1.at<float>(2, 3),
			m1.at<float>(1, 3) - v1 * m1.at<float>(2, 3),
			m2.at<float>(0, 3) - u2 * m2.at<float>(2, 3));

		cout << "\nA==>\n" << A << endl;
		cout << "\nB==>\n" << B << endl;

		Mat C;

		cv::solve(A, B, C, DECOMP_LU);

		cout << "\nC==>\n" << C << endl;

		while (1) {}

		points3D.at<float>(i, 0) = C.at<float>(0,0);
		points3D.at<float>(i, 1) = C.at<float>(1,0);
		points3D.at<float>(i, 3) = C.at<float>(2,0);
	}

}
#endif


