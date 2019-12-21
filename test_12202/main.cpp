#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <time.h>
#include <string>
#include "CameraCalib.h"
#include "CalPhase.h"
#include "PointcloudProcess.h"
#include "StereoReconstruct.h"

using namespace cv;
using namespace std;

static void savePhase(const char* filename, Mat& mat);
static void savepnts3D(const char* filename,  Mat& mat);

const string storintrinsicsyml  = "../mydata/output/intrinsics.yml";
const string storextrinsicsyml  = "../mydata/output/extrinsics.yml";

int main(int argc, char **argv)
{
	/***********************Stereo calibration*****************************************/
#if 0
	const string Calibimagelistfn = "../mydata/input/stereo_calib_images.xml";

	cout << "Stereo Calibration......" << endl;

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
	// images Rectified......
	const string Phaseimageslistfn = "../mydata/input/phase_images.xml";
	const string Rectifiedimageslistfn = "../mydata/input/Rect_phase_images.xml";

	ImgRectified(storintrinsicsyml, storextrinsicsyml, Phaseimageslistfn, Rectifiedimageslistfn);
#endif

	/*******************************calculate unwrapped phase*****************************************/
#if 1
	// calculate unwrapped phase
	// 计算解包相位
	const char* wrapped_phaseleft_txt = "../mydata/output/wrapped_phase_left.txt";
	const char* wrapped_phaseright_txt = "../mydata/output/wrapped_phase_right.txt";
	const char* unwrapped_phaseleft_txt = "../mydata/output/unwrapped_phase_left.txt";
	const char* unwrapped_phaseright_txt = "../mydata/output/unwrapped_phase_right.txt";
	const char* Rect_images_left = "../mydata/input/Rect_images_left.xml";
	const char* Rect_images_right = "../mydata/input/Rect_images_right.xml";

	// 计算左图的包裹相位
	Mat wrapped_phase_left = CalWrappedPhase(Rect_images_left).clone();

	if (wrapped_phaseleft_txt)
	{
		// 存储左图的包裹相位
		printf("storing the wrapped_phaseleft_txt...\n");
		savePhase(wrapped_phaseleft_txt, wrapped_phase_left);
	}
	cout << "Done!!!" << endl;

	// 计算左图的展开相位
	Mat unwrapped_phase_left(Size(wrapped_phase_left.cols, wrapped_phase_left.rows), CV_32FC1, Scalar(0.0));
	cout << "Phase unwrapping......" << endl;

	// 格雷码解码方式计算展开相位
	UnwrappedPhaseGraycodeMethod(wrapped_phase_left, unwrapped_phase_left, Rect_images_left);
	// 古典算法计算展开相位
	// UnwrappedPhaseClassicMethod(wrapped_phase_left, unwrapped_phase_left);

	cout << "Done!!!" << endl;

	if (unwrapped_phaseleft_txt)
	{
		printf("storing the unwrapped_phaseleft_txt...");
		savePhase(unwrapped_phaseleft_txt, unwrapped_phase_left);
	}
	cout << "Done!!!" << endl;


	/*************************Calculate right phase***********************************/
	Mat wrapped_phase_right = CalWrappedPhase(Rect_images_right).clone();
	Mat unwrapped_phase_right(Size(wrapped_phase_right.cols, wrapped_phase_right.rows), CV_32FC1, Scalar(0.0));  // warning SIZE(cols,rows)!!!

	if (wrapped_phaseright_txt)
	{
		printf("storing the wrapped_phaseright_txt...");
		savePhase(wrapped_phaseright_txt, wrapped_phase_right);
	}
	cout << "Done!!!" << endl;

	cout << "Phase unwrapping......" << endl;

	UnwrappedPhaseGraycodeMethod(wrapped_phase_right, unwrapped_phase_right, Rect_images_right);
	// UnwrappedPhaseClassicMethod(wrapped_phase_right, unwrapped_phase_right);

	cout << "Done!!!" << endl;
	if (unwrapped_phaseright_txt)
	{
		printf("storing the unwrapped_phaseright_txt...");
		savePhase(unwrapped_phaseright_txt, unwrapped_phase_right);
	}
	cout << "Done!!!" << endl;

	//imwrite("../mydata/filterphaseright.jpg", filterphase);
	imwrite("../myimages/unwrapped_phase_left.jpg", unwrapped_phase_left);
	imwrite("../myimages/unwrapped_phase_right.jpg", unwrapped_phase_right);
	//imshow("filter_phase", unwrapped_phase_right);

#endif

	/*****************************stereo matching and 3D reconstruction************************************/
#if 1
	vector<Point2f> leftfeaturepoints, rightfeaturepoints;
	cout << "\n=============================" << endl;
	cout << "Calculate feature points......" << endl;

	// find_featurepionts(unwrapped_phase_left, unwrapped_phase_right, leftfeaturepoints, rightfeaturepoints);
	find_featurepionts_single_match(unwrapped_phase_left, unwrapped_phase_right, leftfeaturepoints, rightfeaturepoints);

	cout << "the number of feature: " << leftfeaturepoints.size() << endl;

	Mat pnts3D(4, leftfeaturepoints.size(), CV_64F);

	FileStorage fs(storextrinsicsyml, FileStorage::READ);
	if (!fs.isOpened())
	{
		printf("Failed to open file extrinsics_filename.\n");
		return 0;
	}

	Mat P1, P2;
	fs["P1"] >> P1;
	fs["P2"] >> P2;

	cout << "\n==> P1:\n" << P1 << endl;
	cout << "\n==> P2:\n" << P2 << endl;

	cout << "\n=============================" << endl;
	cout << "Calculate points3D......" << endl;
	cv::triangulatePoints(P1, P2, leftfeaturepoints, rightfeaturepoints, pnts3D);

    const char* pnts3D_filename = "../mydata/output/pnts3D.txt";

    cout << "Save points3D......" <<endl;
    savepnts3D(pnts3D_filename, pnts3D);
    savepntsPCD(pnts3D);

	cout << "Save poind cloud successfully!\n";

#endif    

	/*****************************surface reconstruction************************************/
#if 0
    cout << "\n=============================" << endl;
	cout << "surface reconstruction......" <<endl;
	// filterpointcloud();
	// poissonreconstruction(); // 泊松曲面重建
    
#endif

	cout << "All Done......" <<endl;
    
    return 0;
}

//保存相位
static void savePhase(const char* filename, Mat& mat)
{
    FILE* fp = fopen(filename, "wt");
    for(int y = 0; y < mat.rows; y++)
    {
        float *pixel_phase_data = mat.ptr<float>(y);
	
        for(int x = 0; x < mat.cols; x++)
        {
            float point = *pixel_phase_data++;
            fprintf(fp, "%f \t", point);
        }
        fprintf(fp, "\n");
    }
    fclose(fp);
}

//保存3D点
static void savepnts3D(const char* filename, Mat& mat)
{
    FILE* fp = fopen(filename, "wt");
    
    float *pnts3D_row1 = mat.ptr<float>(0);	
    float *pnts3D_row2 = mat.ptr<float>(1);
    float *pnts3D_row3 = mat.ptr<float>(2);
    float *pnts3D_row4 = mat.ptr<float>(3);
    int pixelsvel; 
    
    for(int y = 0; y < mat.cols; y++)
    {
		float pnts3D_data4 = *(pnts3D_row4 + y);
      
		float pnts3D_data1 = *(pnts3D_row1 + y) / pnts3D_data4;
		float pnts3D_data2 = *(pnts3D_row2 + y) / pnts3D_data4;
		float pnts3D_data3 = *(pnts3D_row3 + y) / pnts3D_data4;

		fprintf(fp, "%f   %f   %f \n", pnts3D_data1, pnts3D_data2, pnts3D_data3);
      
		pixelsvel = (int)(225*pnts3D_data3 / 1900.00);
    }

    fclose(fp);
}



