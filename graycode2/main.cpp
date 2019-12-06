#include <opencv2/opencv.hpp>
#include <math.h>
#include <time.h>
#include <iostream>
#include <string>

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

const int gWidth = 1280;
const int gHeight = 720;

// 存储编码图像（四步相移）
Mat image[4];

// 生成pattern图（四步相位）
Mat pattern[4];

// 相位主值（包裹相位）图
Mat imageWrappedPhase;

void LoadImage()
{
	cout << "\n============================================================================" << endl;
	cout << "1 根据三个频率生成三种相位移pattern" << endl;
	for (int j = 0; j < 4; j++) //四个相位
	{
		pattern[j] = Mat(gHeight, gWidth, CV_32F);
		// 遍历图像填充数据
		for (int r = 0; r < gHeight; r++) {
			float* ptr = pattern[j].ptr<float>(r);
			for (int l = 0; l < gWidth; l++) {
				ptr[l] = 128.0 + 127.0 * sin(2 * CV_PI * l * 32 / gWidth + j * CV_PI / 2);
			}
		}

		// 保存pattern图像
		stringstream ss;
		string filename;
		ss << "pattern/vPhase_" << j << ".bmp";
		ss >> filename;
		cout << "save pattern: " << filename << endl;
		cv::imwrite(filename, pattern[j]);
		ss.clear();
	}
}

// 计算相位主值(包裹相位)
void CalWrappedPhase()
{
	cout << "\n============================================================================" << endl;

	Mat phase1(gHeight, gWidth, CV_32FC1);
	Mat phase2(gHeight, gWidth, CV_32FC1);
	Mat phase3(gHeight, gWidth, CV_32FC1);
	Mat phase4(gHeight, gWidth, CV_32FC1);

	phase1 = pattern[0];
	phase2 = pattern[1];
	phase3 = pattern[2];
	phase4 = pattern[3];

	/*
	phase1 = cv::imread("pattern/vPhase_0.bmp");
	phase2 = cv::imread("pattern/vPhase_1.bmp");
	phase3 = cv::imread("pattern/vPhase_2.bmp");
	phase4 = cv::imread("pattern/vPhase_3.bmp");
	*/

	// 包裹相位图（每个频率有一个对应的包裹相位图）
	imageWrappedPhase = Mat(gHeight, gWidth, CV_32F);

	for (int i = 0; i < gHeight; i++)
	{
		for (int j = 0; j < gWidth; j++)
		{
			float I1 = phase1.at<float>(i, j);
			float I2 = phase2.at<float>(i, j);
			float I3 = phase3.at<float>(i, j);
			float I4 = phase4.at<float>(i, j);
			
			if (I4 == I2 && I1 > I3) // 四个特殊位置
			{
				imageWrappedPhase.at<float>(i, j) = 0;
			}
			else if (I4 == I2 && I1 < I3) // 四个特殊位置
			{
				imageWrappedPhase.at<float>(i, j) = CV_PI;
			}
			else if (I4 > I2&& I1 == I3) // 四个特殊位置
			{
				imageWrappedPhase.at<float>(i, j) = CV_PI / 2;
			}
			else if (I4 < I2 && I1 == I3) // 四个特殊位置
			{
				imageWrappedPhase.at<float>(i, j) = 3 * CV_PI / 2;
			}
			else if (I1 < I3) //第二、三象限
			{
				imageWrappedPhase.at<float>(i, j) = atan((I4 - I2) / (I1 - I3)) + CV_PI;
			}
			else if (I1 > I3&& I4 > I2) //第一象限
			{
				imageWrappedPhase.at<float>(i, j) = atan((I4 - I2) / (I1 - I3));
			}
			else if (I1 > I3&& I4 < I2) //第四象限
			{
				imageWrappedPhase.at<float>(i, j) = atan((I4 - I2) / (I1 - I3)) + 2 * CV_PI;
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

	// 灰度归一化
	cv::normalize(imageWrappedPhase, imageWrappedPhase, 0, 255, NORM_MINMAX);
	cout << "saving wrapped phase \n";
	cv::imwrite("WrapPhase.bmp", imageWrappedPhase);

}

// 解包裹相位(相位展开)
void CalUnwrappedPhase()
{
	// 读取6张格雷码调制图像
	Mat img1 = imread(, CV_LOAD_IMAGE_GRAYSCALE);
	Mat img2 = imread(, CV_LOAD_IMAGE_GRAYSCALE);
	Mat img3 = imread(, CV_LOAD_IMAGE_GRAYSCALE);
	Mat img4 = imread(, CV_LOAD_IMAGE_GRAYSCALE);
	Mat img5 = imread(, CV_LOAD_IMAGE_GRAYSCALE);
	Mat img6 = imread(, CV_LOAD_IMAGE_GRAYSCALE);

	// 相位序列Mat
	Mat phase_series(Size(img1.cols, img1.rows), CV_8UC1, Scalar(0.0));

	// 二值化阈值
	uchar thresh = 130; // 0-255 model21:200   model1:127

	//threshold(img1, img1, thresh, 1, CV_THRESH_BINARY);
	threshold(img1, img1, thresh, 255, CV_THRESH_BINARY);
	threshold(img2, img2, thresh, 255, CV_THRESH_BINARY);
	threshold(img3, img3, thresh, 255, CV_THRESH_BINARY);
	threshold(img4, img4, thresh, 255, CV_THRESH_BINARY);
	threshold(img5, img5, thresh, 255, CV_THRESH_BINARY);
	threshold(img6, img6, thresh, 255, CV_THRESH_BINARY);

	bitwise_xor(img1, img2, img2);
	bitwise_xor(img2, img3, img3);
	bitwise_xor(img3, img4, img4);
	bitwise_xor(img4, img5, img5);
	bitwise_xor(img5, img6, img6);

	cv::imwrite("../myimages/bin1.bmp", img1);
	cv::imwrite("../myimages/bin2.bmp", img2);
	cv::imwrite("../myimages/bin3.bmp", img3);
	cv::imwrite("../myimages/bin4.bmp", img4);
	cv::imwrite("../myimages/bin5.bmp", img5);
	cv::imwrite("../myimages/bin6.bmp", img6);

	int x, y;
	int width = img1.cols;
	int height = img1.rows;
	uchar pre_series, cur_series;
	float pre_unphase, cur_unphase;
	for (y = 0; y < height; y++)
	{
		uchar *img1_ptr = img1.ptr<uchar>(y);
		uchar *img2_ptr = img2.ptr<uchar>(y);
		uchar *img3_ptr = img3.ptr<uchar>(y);
		uchar *img4_ptr = img4.ptr<uchar>(y);
		uchar *img5_ptr = img5.ptr<uchar>(y);
		uchar *img6_ptr = img6.ptr<uchar>(y);

		for (x = 0; x < width; x++)
		{
			phase_series.at<uchar>(y, x) = ((int)(*img1_ptr++)) * 32 + ((int)(*img2_ptr++)) * 16
				+ ((int)(*img3_ptr++)) * 8 + ((int)(*img4_ptr++)) * 4
				+ ((int)(*img5_ptr++)) * 2 + ((int)(*img6_ptr++)) * 1;
		}
	}

	medianBlur(phase_series, phase_series, 9); //中值滤波

	//cv::imwrite("../myimages/phase_series.bmp", phase_series);

	for (y = 0; y < height; y++)
	{
		for (x = 0; x < width; x++)
		{
			//绝对相位 = 2*PI*k + θ(x,y)（相对相位/相位主体）
			dst.at<float>(y, x) = phase_series.at<uchar>(y, x) * 2 * CV_PI + src.at<float>(y, x);
		}
	}

	/*
	if (series_phase_txt)
	{
		printf("storing the series_phase_txt...");
		FILE* fp = fopen(series_phase_txt, "wt");

		for (int y = 0; y < phase_series.rows; y++)
		{
			uchar *pixel_phase_data = phase_series.ptr<uchar>(y);

			for (int x = 0; x < phase_series.cols; x++)
			{
				uchar point = *pixel_phase_data++;
				fprintf(fp, "%d \t", point);
			}
			fprintf(fp, "\n");
		}
		fclose(fp);
	}
	*/

}

int main(int argc, char **argv) 
{
	LoadImage(); //加载图像
	CalWrappedPhase(); //计算包裹相位

    return 0;
}



