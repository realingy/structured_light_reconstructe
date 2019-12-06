#include <opencv2/opencv.hpp>
#include <math.h>
#include <time.h>
#include <iostream>
#include <string>
#include "CalPhase.h"

using namespace cv;
using namespace std;

/*
#define PHASE_THRESHOLD  0.01  // model20 0.01(singlematch) model1 0.01 //相位阈值
#define BLOCK_THRESHOLD  0.3   // Block阈值

static void savePhase(const char* filename, Mat& mat);
static void savepnts3D(const char* filename,  Mat& mat);
void find_featurepionts(Mat& leftphase, Mat& rightphase, 
			vector<Point2f>& leftkeypoint, vector<Point2f>& rightkeypoint);
void find_featurepionts_single_match(Mat& leftphase, Mat& rightphase, 
			vector<Point2f>& leftkeypoint, vector<Point2f>& rightkeypoint);
void find_featureSAD(Mat& leftphase, Mat& rightphase);
void find_featureBlock(Mat& leftphase, Mat& rightphase, 
                       vector<Point2f>& leftkeypoint, vector<Point2f>& rightkeypoint);

const string storintrinsicsyml  = "../mydata/output/intrinsics.yml";
const string storextrinsicsyml  = "../mydata/output/extrinsics.yml";
*/

// 计算每种频率对应的相位主值
// 输出三种频率的相位主值(包裹相位)，用于相差计算
void CalImageWrappedPhase()
{
	cout << "\n============================================================================" << endl;
	cout << "2 计算每种频率对应的相位主值, 输出三种频率的相位主值(包裹相位)，用于相差计算 " << endl;

	Mat phase1(gHeight, gWidth, CV_32F);
	Mat phase2(gHeight, gWidth, CV_32F);
	Mat phase3(gHeight, gWidth, CV_32F);
	Mat phase4(gHeight, gWidth, CV_32F);

	for (int n = 0; n < 3; n++)
	{
		phase1 = pattern[n][0];
		phase2 = pattern[n][1];
		phase3 = pattern[n][2];
		phase4 = pattern[n][3];

		// 包裹相位图（每个频率有一个对应的包裹相位图）
		imageWrappedPhase[n] = Mat(gHeight, gWidth, CV_32F);

		for (int i = 0; i < gHeight; i++)
		{
			for (int j = 0; j < gWidth; j++)
			{
				float I1 = phase1.at<float>(i, j);
				float I2 = phase2.at<float>(i, j);
				float I3 = phase3.at<float>(i, j);
				float I4 = phase4.at<float>(i, j);

				//(I4-I2)/(I1-I3)
				if (I4 == I2 && I1 > I3) // 四个特殊位置
				{
					imageWrappedPhase[n].at<float>(i, j) = 0;
				}
				else if (I4 == I2 && I1 < I3) // 四个特殊位置
				{
					imageWrappedPhase[n].at<float>(i, j) = CV_PI;
				}
				else if (I4 > I2&& I1 == I3) // 四个特殊位置
				{
					imageWrappedPhase[n].at<float>(i, j) = CV_PI / 2;
				}
				else if (I4 < I2 && I1 == I3) // 四个特殊位置
				{
					imageWrappedPhase[n].at<float>(i, j) = 3 * CV_PI / 2;
				}
				else if (I1 < I3) //第二、三象限
				{
					imageWrappedPhase[n].at<float>(i, j) = atan((I4 - I2) / (I1 - I3)) + CV_PI;
				}
				else if (I1 > I3&& I4 > I2) //第一象限
				{
					imageWrappedPhase[n].at<float>(i, j) = atan((I4 - I2) / (I1 - I3));
				}
				else if (I1 > I3&& I4 < I2) //第四象限
				{
					imageWrappedPhase[n].at<float>(i, j) = atan((I4 - I2) / (I1 - I3)) + 2 * CV_PI;
				}
			}
		}
	}

	/*
	ofstream file("wrapphase1.txt");
	for (int i = 0; i < gHeight; i++)
	{
		for (int j = 0; j < gWidth; j++)
		{
			file << imageWrappedPhase[0].at<float>(i, j) << "\t";
		}
		file << endl;
	}

	ofstream file2("wrapphase2.txt");
	for (int i = 0; i < gHeight; i++)
	{
		for (int j = 0; j < gWidth; j++)
		{
			file2 << imageWrappedPhase[1].at<float>(i, j) << "\t";
		}
		file2 << endl;
	}
	*/
}

int main(int argc, char **argv) 
{


#if 0
	// Calculate unwrapped phase
	// 计算解包相位
    const char* wrapped_phaseleft_txt = "../mydata/output/wrapped_phase_left.txt";
    const char* wrapped_phaseright_txt = "../mydata/output/wrapped_phase_right.txt";
    const char* unwrapped_phaseleft_txt = "../mydata/output/unwrapped_phase_left.txt";
    const char* unwrapped_phaseright_txt = "../mydata/output/unwrapped_phase_right.txt";
    const char* Rect_images_left = "../mydata/input/Rect_images_left.xml";
    const char* Rect_images_right = "../mydata/input/Rect_images_right.xml";
    
	/***********************Calculate left phase*****************************************/
	/***********************计算左相位*****************************************/
	// 计算左图的包裹相位
    Mat wrapped_phase_left = CalWrappedPhase(Rect_images_left).clone();

    if(wrapped_phaseleft_txt)
    {
		// 存储左图的包裹相位
		printf("storing the wrapped_phaseleft_txt...\n");
		savePhase(wrapped_phaseleft_txt, wrapped_phase_left);
    }
    cout << "Done!!!" <<endl;

    // 计算左图的展开相位
	Mat unwrapped_phase_left(Size(wrapped_phase_left.cols, wrapped_phase_left.rows), CV_32FC1, Scalar(0.0));
    cout << "Phase unwrapping......" <<endl;

	// 格雷码解码方式计算展开相位
    UnwrappedPhaseGraycodeMethod(wrapped_phase_left, unwrapped_phase_left, Rect_images_left);
	// 古典算法计算展开相位
	// UnwrappedPhaseClassicMethod(wrapped_phase_left, unwrapped_phase_left);
    
    cout << "Done!!!" <<endl;

    if(unwrapped_phaseleft_txt)
    {
      printf("storing the unwrapped_phaseleft_txt...");
      savePhase(unwrapped_phaseleft_txt, unwrapped_phase_left);
    }
    cout << "Done!!!" <<endl;
    
    
	/*************************Calculate right phase***********************************/
	/***********************计算右相位*****************************************/
    Mat wrapped_phase_right = CalWrappedPhase(Rect_images_right).clone();
    Mat unwrapped_phase_right(Size(wrapped_phase_right.cols, wrapped_phase_right.rows), CV_32FC1, Scalar(0.0));  // warning SIZE(cols,rows)!!!
      
    if(wrapped_phaseright_txt)
    {
      printf("storing the wrapped_phaseright_txt...");
      savePhase(wrapped_phaseright_txt, wrapped_phase_right);
    }
    cout << "Done!!!" <<endl;
    
    cout << "Phase unwrapping......" <<endl;
   
    UnwrappedPhaseGraycodeMethod(wrapped_phase_right, unwrapped_phase_right, Rect_images_right);
 //   UnwrappedPhaseClassicMethod(wrapped_phase_right, unwrapped_phase_right);
    
    cout << "Done!!!" <<endl;
    if(unwrapped_phaseright_txt)
    {
		printf("storing the unwrapped_phaseright_txt...");
		savePhase(unwrapped_phaseright_txt, unwrapped_phase_right);
    }
    cout << "Done!!!" <<endl;

	//imwrite("../mydata/filterphaseright.jpg", filterphase);
	imwrite("../myimages/unwrapped_phase_left.jpg", unwrapped_phase_left);
	imwrite("../myimages/unwrapped_phase_right.jpg", unwrapped_phase_right);
	//imshow("filter_phase", unwrapped_phase_right);
    //waitKey(0);
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
      pixelsvel = (int)(225*pnts3D_data3 / 1900.00);
     // pixelsvel = pnts3D_data3;
      if( i < pnts3Dimg.cols && j < pnts3Dimg.rows && pixelsvel < 255)
         pnts3Dimg.at<uchar>(j, i) = pixelsvel;
    }
    imwrite("../mydata/output/pnts3D.jpg", pnts3Dimg);
    fclose(fp);
}



