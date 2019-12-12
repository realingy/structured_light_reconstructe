#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <time.h>
#include <string>
#include "CameraCalib.h"
#include "CalPhase.h"
#include "Propointcloud.h"

using namespace cv;
using namespace std;

#define PHASE_THRESHOLD  0.01
#define BLOCK_THRESHOLD  0.3

static void savePhase(const char* filename, Mat& mat);
static void savepnts3D(const char* filename,  Mat& mat);
void find_featurepionts(Mat& leftphase, Mat& rightphase, 
			vector<Point2f>& leftkeypoint, vector<Point2f>& rightkeypoint);
void find_featurepionts_single_match(Mat& leftphase, Mat& rightphase, 
			vector<Point2f>& leftkeypoint, vector<Point2f>& rightkeypoint);
void find_featureSAD(Mat& leftphase, Mat& rightphase);
void find_featureBlock(Mat& leftphase, Mat& rightphase, 
                       vector<Point2f>& leftkeypoint, vector<Point2f>& rightkeypoint);
void Triangulate(Mat projMatr1, Mat projMatr2, const vector<Point2f> & projPoints1, const vector<Point2f> projPoints2, Mat points4D);

	
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

#if 1
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
 
#if 1
	// stereo matching and 3D reconstruction

    const char* pnts3D_filename = "../data/output/pnts3D.txt";
    
	/*
    FileStorage fs(storextrinsicsyml, FileStorage::READ);
    if(!fs.isOpened())
    {
		printf("Failed to open file extrinsics_filename.\n");
		return 0;
    }

    Mat P1, P2;
    fs["P1"] >> P1;
    fs["P2"] >> P2;
	*/
    
    vector<Point2f> leftfeaturepoints, rightfeaturepoints; 
    cout << "Calculate feature points......"<<endl;

    // find_featurepionts(unwrapped_phase_left, unwrapped_phase_right, leftfeaturepoints, rightfeaturepoints);
    find_featurepionts_single_match(unwrapped_phase_left, unwrapped_phase_right, leftfeaturepoints, rightfeaturepoints);
	// find_featureBlock(unwrapped_phase_left, unwrapped_phase_right, leftfeaturepoints, rightfeaturepoints);
	// find_featureSAD(unwrapped_phase_left, unwrapped_phase_right);

	cout << "the number of feature: " << leftfeaturepoints.size() << " <==> " << leftfeaturepoints.size() <<endl;

	// Mat pnts3D(4, leftfeaturepoints.size(), CV_64F);
	Mat pnts3D(3, leftfeaturepoints.size(), CV_64F);

    cout << "Calculate points3D......"<<endl;

	Mat R1 = (Mat_<float>(3, 3) <<
		-0.003449992052740, 0.908392369471684,  0.418104533149851,	  
		 0.992268580264290, 0.054980888811595, -0.111266196509893,
		-0.124061122738460, 0.414488124016969, -0.901558890408035);

	Mat T1 = (Mat_<float>(3, 1) <<
		3.688988301573581, -4.927452164451585, 329.276493470459510 );

	Mat R2 = (Mat_<float>(3, 3) <<
		-0.005778730523496, 0.970132888506089, 0.242505226567117,
		 0.992520961272705, 0.035135856240512, -0.116908567010947,
		-0.121937474583672, 0.240015937481406, -0.963080267707255);

	Mat T2 = (Mat_<float>(3, 1) <<
		3.780742082249347, -4.998608845649666, 328.926407599367390);

	Mat R2T;
	cv::transpose(R2, R2T); //转置

	Mat R = R1 * R2T;

	//cout << "\n==> R:\n" << R << endl;

	Mat T = T2 - R * T1;

	//cout << "\n==> T:\n" << T << endl;

	T1 = (Mat_<float>(3, 4) <<
		1.00, 0.00, 0.00, 0.00,
		0.00, 1.00, 0.00, 0.00,
		0.00, 0.00, 1.00, 0.00);

	T2 = (Mat_<float>(3, 4) <<
		R.at<float>(0, 0), R.at<float>(0, 1), R.at<float>(0, 2), T.at<float>(0, 0),
		R.at<float>(1, 0), R.at<float>(1, 1), R.at<float>(1, 2), T.at<float>(1, 0),
		R.at<float>(2, 0), R.at<float>(2, 1), R.at<float>(2, 2), T.at<float>(2, 0));

	//cout << "\n==> T1:\n" << T1 << endl;
	//cout << "\n==> T2:\n" << T2 << endl;

	// 通过三角测量计算三维坐标(基于世界坐标和图像坐标的转换关系)
    cv::triangulatePoints(T1, T2, leftfeaturepoints, rightfeaturepoints, pnts3D);

    // Triangulate(T1, T2, leftfeaturepoints, rightfeaturepoints, pnts3D);

	/*
	Mat R = (Mat_<float>(3, 3) <<
		0.9826737, -0.020387046, -0.1842189,
		0.020622084, 0.99978715, -0.00064015388,
		0.18419276, -0.0031699166, 0.982885);

	Mat T = (Mat_<float>(3, 1) <<
		60.714165, 0.062507629, 4.5903931);

    Mat cameraMatrix[2], distCoeffs[2]; //内参矩阵、畸变向量
    Mat R1, R2, P1, P2, Q;
    Rect validRoi[2];
	Size imageSize;

	cameraMatrix[0] = (Mat_<float>(3, 3) <<
			0.9826737, -0.020387046, -0.1842189,
			0.020622084, 0.99978715, -0.00064015388,
			0.18419276, -0.0031699166, 0.982885);

	distCoeffs[0] = (Mat_<float>(3, 1) << 
			1.00, 1.00, 1.00);

	cameraMatrix[1] = (Mat_<float>(3, 3) <<
			0.9826737, -0.020387046, -0.1842189,
			0.020622084, 0.99978715, -0.00064015388,
			0.18419276, -0.0031699166, 0.982885);

	distCoeffs[1] = (Mat_<float>(3, 1) << 
			1.00, 1.00, 1.00);

	stereoRectify(cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		imageSize, R, T, R1, R2, P1, P2, Q,
		0, -1, imageSize, &validRoi[0], &validRoi[1]);

	// P1，P2是两个相机相对于极线校正平面的转换矩阵，也可以使用不做极线校正的矩阵
    // cv::triangulatePoints(P1, P2, leftfeaturepoints, rightfeaturepoints, pnts3D);
	*/

    cout << "Save points3D......" <<endl;
    savepnts3D(pnts3D_filename, pnts3D);

	// cout << "1111111111111111111111111111111111111\n" << pnts3D.cols << "===>" << pnts3D.rows << endl;

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

void find_featurepionts(Mat& leftphase, Mat& rightphase, 
			vector<Point2f>& leftkeypoint, vector<Point2f>& rightkeypoint)
{
	int nr = leftphase.rows;
	int nc = leftphase.cols;
	int x, y, k1,k2;
	float left, right;
	Point2f fleft, fright;
	float *pre_right_data;
	int pre_k;
  
	for(y = 0; y < nr; y+=1)
	{
		float *left_phase_data = leftphase.ptr<float>(y);
		float *right_phase_data = rightphase.ptr<float>(y);   
		float *left_phase_data2;
		// k = 0;
		// pre_right_data = right_phase_data;
		// pre_k = k;
    
		for(x = 0; x < nc; x++)
		{
			left = *left_phase_data++;
           
			if(left > 2*CV_PI)
			{
				// right_phase_data = pre_right_data;
				// k = pre_k;
				// order constraint	
				right_phase_data = rightphase.ptr<float>(y);        	
				k1 = 0;
	
				while((abs(left - *right_phase_data++) > PHASE_THRESHOLD) && (k1 < nc)) k1++;
				if(k1 < nc)
				{	  	  
					//pre_right_data = right_phase_data;
					//pre_k = k;
					right = *(--right_phase_data);
					left_phase_data2 = leftphase.ptr<float>(y);
					k2=0;
					while((abs(right - *left_phase_data2++) > PHASE_THRESHOLD) && (k2 < nc))
						k2++;
	
					if((k2 < nc) && (abs(k2-x) < 2))
					{
						fleft.x = (x+k2)/2;
						fleft.y = y;
						fright.x = k1;
						fright.y = y;
						leftkeypoint.push_back(fleft);
						rightkeypoint.push_back(fright);
					}
				}
			}
		}
	}
}

void find_featurepionts_single_match(Mat& leftphase, Mat& rightphase, 
			vector<Point2f>& leftkeypoint, vector<Point2f>& rightkeypoint)
{
	int nr = leftphase.rows;
	int nc = leftphase.cols;

	int x, y, k;
	float left;
	Point2f fleft, fright;
  
	for(y = 0; y < nr; y+=1)
	{
		float *left_phase_data = leftphase.ptr<float>(y); 
		float *right_phase_data = rightphase.ptr<float>(y);

		for(x = 0; x < nc; x++)
		{
			left = *left_phase_data++;
      
			if(left > 2*CV_PI)
			{
				right_phase_data = rightphase.ptr<float>(y);       	
				k = 0;
	
				while((abs(left - *right_phase_data++) > PHASE_THRESHOLD) && (k < nc)) k++;
				if(k < nc)
				{
					fleft.x = x;
					fleft.y = y;
					fright.x = k;
					fright.y = y;
					leftkeypoint.push_back(fleft);
					rightkeypoint.push_back(fright);
				}
			}
		}
	}
}

void find_featureBlock(Mat& leftphase, Mat& rightphase, 
                       vector<Point2f>& leftkeypoint, vector<Point2f>& rightkeypoint)
{
	int nr = leftphase.rows;
	int nc = leftphase.cols;
	Size Blocksize = Size(3,3);

	int h = Blocksize.height;
	int w = Blocksize.width;
	int x,y,i,j,k;
  
	Point2f fleft, fright;
  
	float leftdataaver, rightdataaver;
	for(y=0; y < nr-h; y+=h)
	{
		for(x=0; x < nc-w; x+=w)
		{
			leftdataaver = 0;      
			for(j=0; j<h; j++)
			{
				for(i=0; i<w; i++)
					leftdataaver += leftphase.at<float>(y+j,x+i);
			}
			leftdataaver = leftdataaver/(h*w);
      
			if(leftdataaver > 2*CV_PI)
			{
				for(k=0; k < nc-w; k+=w)
				{
					rightdataaver = 0;
					for(j=0; j<h; j++)
					{
						for(i=0; i<w; i++)
							rightdataaver += rightphase.at<float>(y+j,k+i);
					}
					rightdataaver = rightdataaver / (h*w);
					if(abs(rightdataaver-leftdataaver) < BLOCK_THRESHOLD)
					{
						fleft.x = x/2 + 1;
						fleft.y = y/2 + 1;
						fright.x = k/2 + 1;
						fright.y = y/2 + 1;
						leftkeypoint.push_back(fleft);
						rightkeypoint.push_back(fright);
						break;
					}
				}
			}
		}
	}
}

//计算视差
void find_featureSAD(Mat& leftphase, Mat& rightphase)
{
    Mat leftimg8(Size(leftphase.cols, leftphase.rows), CV_8UC1, Scalar(0.0));
    Mat rightimg8(Size(leftphase.cols, leftphase.rows), CV_8UC1, Scalar(0.0));
    convertScaleAbs(leftphase, leftimg8);
    convertScaleAbs(rightphase, rightimg8);
    
//     imshow("leftimg8", leftimg8);
//     imshow("rightimg8", rightimg8);
//     waitKey(0);
//     cvConvertScale(&leftphase, leftimg8, 255.0, 0.0);
//     cvConvertScale(&rightphase, rightimg8, 255.0, 0.0);
    
    Size img_size = leftphase.size();
    Rect roi1, roi2;
    Mat Q;
    
    // reading intrinsic parameters
    FileStorage fs(storintrinsicsyml, FileStorage::READ);
    if(!fs.isOpened())
    {
       printf("Failed to open file intrinsic_filename.\n");
       return ;
    }

    Mat M1, D1, M2, D2;
    fs["M1"] >> M1;
    fs["D1"] >> D1;
    fs["M2"] >> M2;
    fs["D2"] >> D2;

    fs.open(storextrinsicsyml, FileStorage::READ);
    if(!fs.isOpened())
    {
       printf("Failed to open file extrinsic_filename.\n");
       return ;
    }
    
	Mat R, T, R1, P1, R2, P2;
	fs["R"] >> R;
	fs["T"] >> T;
	fs["R1"] >> R1;
	fs["R2"] >> R2;
	fs["P1"] >> P1;
	fs["P2"] >> P2;
   
	stereoRectify(M1, D1, M2, D2,img_size, R, T, R1, R2, P1, P2, Q,
                  0, -1, img_size, &roi1, &roi2);
  
	int numberOfDisparities =((img_size.width/8) + 15) & -16;
   
	Ptr<StereoBM> bm = StereoBM::create(16, 3);
	bm->setROI1(roi1);
	bm->setROI2(roi2);  
	bm->setPreFilterCap(31);
	bm->setBlockSize(5);
	bm->setMinDisparity(0);
	bm->setNumDisparities(numberOfDisparities);
	bm->setTextureThreshold(120);
	bm->setUniquenessRatio(7);
	bm->setSpeckleWindowSize(20);
	bm->setSpeckleRange(64);
	bm->setDisp12MaxDiff(-1);  
  
	Mat disp, disp8;
	bm->compute(leftimg8, rightimg8, disp);
	disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));
	double sf = 640./MAX(disp8.rows, disp8.cols);
	resize(disp8, disp8, Size(), sf, sf);   //调整图像大小640 x 640
	imshow("disparity", disp8);
	waitKey(0);
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
    Mat pnts3Dimg(4000, 4000, CV_8UC1);
    
    float *pnts3D_row1 = mat.ptr<float>(0);	
    float *pnts3D_row2 = mat.ptr<float>(1);
    float *pnts3D_row3 = mat.ptr<float>(2);
    float *pnts3D_row4 = mat.ptr<float>(3);
    int i, j, pixelsvel; 

    for(int y = 0; y < mat.cols; y++)
    {
      float pnts3D_data4 = *(pnts3D_row4 + y);
      
      float pnts3D_data1 = *(pnts3D_row1 + y) / pnts3D_data4;
      float pnts3D_data2 = *(pnts3D_row2 + y) / pnts3D_data4;
      float pnts3D_data3 = *(pnts3D_row3 + y) / pnts3D_data4;

      fprintf(fp, "%f   %f   %f \n", pnts3D_data1, pnts3D_data2, pnts3D_data3);
      
      i = (int)(10*pnts3D_data1) + 1000; // col
      j = (int)(10*pnts3D_data2) + 2000; // row
      pixelsvel = (uchar)(225 * pnts3D_data3 / 1900.00);
     // pixelsvel = pnts3D_data3;
      if( i < pnts3Dimg.cols && j < pnts3Dimg.rows && pixelsvel < 255)
         pnts3Dimg.at<uchar>(j, i) = pixelsvel;
    }
    imwrite("../data/output/pnts3D.jpg", pnts3Dimg);
    fclose(fp);

}

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

/*
void pose_estimation_2d2d(
	const std::vector<KeyPoint>& keypoints_1,
	const std::vector<KeyPoint>& keypoints_2,
	const std::vector< DMatch >& matches,
	Mat& R, Mat& t)
{
	// 相机内参,TUM Freiburg2
	Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

	//-- 把匹配点转换为vector<Point2f>的形式
	vector<Point2f> points1;
	vector<Point2f> points2;

	for (int i = 0; i < (int)matches.size(); i++)
	{
		points1.push_back(keypoints_1[matches[i].queryIdx].pt);
		points2.push_back(keypoints_2[matches[i].trainIdx].pt);
	}

	//-- 计算基础矩阵
	Mat fundamental_matrix;
	fundamental_matrix = findFundamentalMat(points1, points2, CV_FM_8POINT);
	cout << "fundamental_matrix is " << endl << fundamental_matrix << endl;

	//-- 计算本质矩阵
	Point2d principal_point(325.1, 249.7);				//相机主点, TUM dataset标定值
	int focal_length = 521;						//相机焦距, TUM dataset标定值
	Mat essential_matrix;
	essential_matrix = findEssentialMat(points1, points2, focal_length, principal_point);
	cout << "essential_matrix is " << endl << essential_matrix << endl;

	//-- 计算单应矩阵
	Mat homography_matrix;
	homography_matrix = findHomography(points1, points2, RANSAC, 3);
	cout << "homography_matrix is " << endl << homography_matrix << endl;

	//-- 从本质矩阵中恢复旋转和平移信息.
	recoverPose(essential_matrix, points1, points2, R, t, focal_length, principal_point);
	cout << "R is " << endl << R << endl;
	cout << "t is " << endl << t << endl;
}

// 将像素坐标转换至相机坐标
Point2f pixel2cam(const Point2d& p, const Mat& K)
{
	return Point2f
	(
		(p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
		(p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
	);
}

// 三角测量功能函数
void triangulation(
	const vector<KeyPoint> & keypoint_1,
	const vector<KeyPoint> & keypoint_2,
	const std::vector<DMatch> & matches,
	const Mat & R, const Mat & t,
	vector<Point3d>& points)
{
	Mat T1 = (Mat_<float>(3, 4) <<
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0);
	Mat T2 = (Mat_<float>(3, 4) <<
		R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
		R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
		R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0)
		);

	Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
	vector<Point2f> pts_1, pts_2;
	for (DMatch m : matches)
	{
		// 将像素坐标转换至相机坐标
		pts_1.push_back(pixel2cam(keypoint_1[m.queryIdx].pt, K));
		pts_2.push_back(pixel2cam(keypoint_2[m.trainIdx].pt, K));
	}

	Mat pts_4d;
	cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);

	// 转换成非齐次坐标
	for (int i = 0; i < pts_4d.cols; i++)
	{
		Mat x = pts_4d.col(i);
		x /= x.at<float>(3, 0); // 归一化   //金戈大王注：此处的归一化是指从齐次坐标变换到非齐次坐标。而不是变换到归一化平面。
		Point3d p(
			x.at<float>(0, 0),
			x.at<float>(1, 0),
			x.at<float>(2, 0)
		);
		points.push_back(p);
	}
}
*/

