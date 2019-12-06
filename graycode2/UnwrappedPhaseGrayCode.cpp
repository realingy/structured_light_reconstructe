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

#if 1
const int gWidth = 1600;
const int gHeight = 1200;
#else
const int gWidth = 1280;
const int gHeight = 720;
#endif

// 存储编码图像（四步相移）
Mat image[4];

// 生成pattern图（四步相位）
Mat pattern[4];

// 相位主值（包裹相位）图
Mat imageWrappedPhase;

Mat img1;
Mat img2;
Mat img3;
Mat img4;
Mat img5;
Mat img6;

void PhaseShiftPatternGenerate()
{
	/*
	cout << "\n============================================================================" << endl;
	cout << "根据频率生成相位移pattern" << endl;
	for (int j = 0; j < 4; j++) //四个相位
	{
		pattern[j] = Mat(gHeight, gWidth, CV_32F);
		// 遍历图像填充数据
		for (int r = 0; r < gHeight; r++) {
			float* ptr = pattern[j].ptr<float>(r);
			for (int l = 0; l < gWidth; l++) {
				ptr[l] = 128.0 + 127.0 * sin(2 * CV_PI * l * 64 / gWidth + j * CV_PI / 2);
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
	*/
}

void LoadImage()
{
#if 0
	pattern[0] = imread("pattern/vPhase_0.bmp", CV_LOAD_IMAGE_GRAYSCALE);
	pattern[1] = imread("pattern/vPhase_1.bmp", CV_LOAD_IMAGE_GRAYSCALE);
	pattern[2] = imread("pattern/vPhase_2.bmp", CV_LOAD_IMAGE_GRAYSCALE);
	pattern[3] = imread("pattern/vPhase_3.bmp", CV_LOAD_IMAGE_GRAYSCALE);
#else
	pattern[0] = imread("image/Image_8.bmp", CV_LOAD_IMAGE_GRAYSCALE);
	pattern[1] = imread("image/Image_9.bmp", CV_LOAD_IMAGE_GRAYSCALE);
	pattern[2] = imread("image/Image_10.bmp", CV_LOAD_IMAGE_GRAYSCALE);
	pattern[3] = imread("image/Image_11.bmp", CV_LOAD_IMAGE_GRAYSCALE);
#endif
}

// 计算相位主值(包裹相位)
void CalWrappedPhase()
{
	Mat phase1 = pattern[0];
	Mat phase2 = pattern[1];
	Mat phase3 = pattern[2];
	Mat phase4 = pattern[3];

	// 包裹相位图（每个频率有一个对应的包裹相位图）
	imageWrappedPhase = Mat(gHeight, gWidth, CV_32F);

	for (int i = 0; i < gHeight; i++)
	{
		for (int j = 0; j < gWidth; j++)
		{
			float I1 = (float)phase1.at<uchar>(i, j);
			float I2 = (float)phase2.at<uchar>(i, j);
			float I3 = (float)phase3.at<uchar>(i, j);
			float I4 = (float)phase4.at<uchar>(i, j);
			
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

	Mat roi1;
	Mat roi2;
	for (int j = 0; j < gWidth; j++)
	{
		if ((int)imageWrappedPhase.at<float>(0, j) == 0)
		{
			roi1 = imageWrappedPhase(Rect(0, 0, j, gHeight));
			roi2 = imageWrappedPhase(Rect(j, 0, gWidth-j, gHeight));
			break;
		}
	}

	roi2.copyTo(imageWrappedPhase(Rect(0, 0, roi2.cols, gHeight)));

	ofstream file("wrapphase.txt");
	for (int i = 0; i < gHeight; i++)
	{
		for (int j = 0; j < gWidth; j++)
		{
			file << imageWrappedPhase.at<float>(i, j) << "\t";
		}
		file << endl;
	}
}

// 解包裹相位(相位展开)
void CalUnwrappedPhase()
{
#if 0
	img1 = imread("pattern/vGray1.bmp", CV_LOAD_IMAGE_GRAYSCALE);
	img2 = imread("pattern/vGray3.bmp", CV_LOAD_IMAGE_GRAYSCALE);
	img3 = imread("pattern/vGray5.bmp", CV_LOAD_IMAGE_GRAYSCALE);
	img4 = imread("pattern/vGray7.bmp", CV_LOAD_IMAGE_GRAYSCALE);
	img5 = imread("pattern/vGray9.bmp", CV_LOAD_IMAGE_GRAYSCALE);
	img6 = imread("pattern/vGray11.bmp", CV_LOAD_IMAGE_GRAYSCALE);
#else
	img1 = imread("image/Image_2.bmp", CV_LOAD_IMAGE_GRAYSCALE);
	img2 = imread("image/Image_3.bmp", CV_LOAD_IMAGE_GRAYSCALE);
	img3 = imread("image/Image_4.bmp", CV_LOAD_IMAGE_GRAYSCALE);
	img4 = imread("image/Image_5.bmp", CV_LOAD_IMAGE_GRAYSCALE);
	img5 = imread("image/Image_6.bmp", CV_LOAD_IMAGE_GRAYSCALE);
	img6 = imread("image/Image_7.bmp", CV_LOAD_IMAGE_GRAYSCALE);
#endif

	// 二值化阈值
	uchar thresh = 130; // 0-255 model21:200   model1:127

	//threshold(img1, img1, thresh, 1, CV_THRESH_BINARY);
	threshold(img1, img1, thresh, 1, CV_THRESH_BINARY);
	threshold(img2, img2, thresh, 1, CV_THRESH_BINARY);
	threshold(img3, img3, thresh, 1, CV_THRESH_BINARY);
	threshold(img4, img4, thresh, 1, CV_THRESH_BINARY);
	threshold(img5, img5, thresh, 1, CV_THRESH_BINARY);
	threshold(img6, img6, thresh, 1, CV_THRESH_BINARY);

	bitwise_xor(img1, img2, img2);
	bitwise_xor(img2, img3, img3);
	bitwise_xor(img3, img4, img4);
	bitwise_xor(img4, img5, img5);
	bitwise_xor(img5, img6, img6);

	// 相位序列Mat
	// Mat phase_series(gHeight, gWidth, CV_32FC1, Scalar(0.0));
	Mat phase_series(gHeight, gWidth, CV_8UC1, Scalar(0.0));

	uchar pre_series, cur_series;
	float pre_unphase, cur_unphase;
	for (int y = 0; y < gHeight; y++)
	{
		for (int x = 0; x < gWidth; x++)
		{
			phase_series.at<uchar>(y, x) = img1.at<uchar>(y, x) * 32
											+ img2.at<uchar>(y, x) * 16
											+ img3.at<uchar>(y, x) * 8
											+ img4.at<uchar>(y, x) * 4
											+ img5.at<uchar>(y, x) * 2
											+ img6.at<uchar>(y, x) * 1;
		}
	}

	medianBlur(phase_series, phase_series, 9); //中值滤波

	cv::imwrite("phase_series.bmp", phase_series);

	ofstream file("phase_series.txt");
	for (int i = 0; i < gHeight; i++)
	{
		for (int j = 0; j < gWidth; j++)
		{
			file << (int)phase_series.at<uchar>(i, j) << "\t";
		}
		file << endl;
	}

	Mat dst(gHeight, gWidth, CV_32FC1);

	for (int y = 0; y < gHeight; y++)
	{
		for (int x = 0; x < gWidth; x++)
		{
			dst.at<float>(y, x) = phase_series.at<uchar>(y, x) * 2 * CV_PI + imageWrappedPhase.at<float>(y, x);
		}
	}

	cv::imwrite("UnwrappedPhase.bmp", dst);

	// 灰度归一化
	cv::normalize(imageWrappedPhase, imageWrappedPhase, 0, 255, NORM_MINMAX);
	cout << "saving wrapped phase \n";
	cv::imwrite("WrapPhase.bmp", imageWrappedPhase);

	ofstream file2("unwrapphase.txt");
	for (int i = 0; i < gHeight; i++)
	{
		for (int j = 0; j < gWidth; j++)
		{
			file2 << dst.at<float>(i, j) << "\t";
		}
		file2 << endl;
	}
	file2.close();

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
	CalUnwrappedPhase(); //解包裹

    return 0;
}

#if 0
int decodeGrayCodes(Mat * & gray_codes, //编码图
					Mat & decoded_cols, //解码图
					Mat & mask,			//mask
					int& n_cols,		//编码图数
					int& col_shift,
					int sl_thresh )		//阈值
{
	// Extract width and height of images. 由相机拍摄的第一张图像，获取其高和宽
	//int cam_width  = gray_codes[0]->width;
	//int cam_height = gray_codes[0]->height;
	int cam_width  = gHeight;
	int cam_height = gWidth;

	Mat gray_1 = Mat(Size(cam_width, cam_height), CV_8U, 1);    //格雷码图1
	Mat gray_2 = Mat(Size(cam_width, cam_height), CV_8U, 1);
	Mat temp = Mat(Size(cam_width, cam_height), CV_8U, 1);    //图像的临时存储变量

	Mat bit_plane_1 = Mat(Size(cam_width, cam_height), CV_8U, 1);
	Mat bit_plane_2 = Mat(Size(cam_width, cam_height), CV_8U, 1);    //位平面

	//对mask图像中的像素设置值为 0
	mask = Mat::zeros(gHeight, gWidth, CV_32FC1);

	// Decode Gray codes for projector columns.

	decoded_cols = Mat::zeros(gHeight, gWidth, CV_32FC1);
	for(int i=0; i < n_cols; i++){
		// Decode bit-plane and update mask.          解码位平面   更新掩码
		// cvCvtColor(gray_codes[2*(i+1)],   gray_1, CV_RGB2GRAY);         //从拍到的第三张图像开始，将相机拍到的连续两张三通道的图像转换为灰度图
		// cvCvtColor(gray_codes[2*(i+1)+1], gray_2, CV_RGB2GRAY);

		cvtColor(gray_codes[2*(i+1)], gray_1, COLOR_BGR2GRAY);
		cvtColor(gray_codes[2*(i+1) + 1], gray_2, COLOR_BGR2GRAY);

		//将两张灰度图作差，差图的绝对值保存到temp临时变量里
		absdiff(gray_1, gray_2, temp);

		// cvCmpS(out, 100, out, CV_CMP_GE);
		// cvThreshold(out1, out1, 100, 255, CV_THRESH_BINARY);
		//cvCmpS(temp, sl_thresh, temp, CV_CMP_GE);                       //比较temp图像的像素点与 sl_thresh 的大小，结果存放到temp
		threshold(temp, temp, sl_thresh, 255, CV_THRESH_BINARY);

		//cvOr(temp, mask, mask);                                         //temp和mask两个矩阵对应元素做或运行,
		bitwise_xor(temp, mask, mask);

		//cvCmp(gray_1, gray_2, bit_plane_2, CV_CMP_GE);                  //比较两幅相应的图像的像素点,结果存到bit_plane_2中
		bit_plane_2 = gray_1 - gray_2;

		// Convert from gray code to decimal value.   格雷码转换为十进制值
		if (i > 0) {
			//cvXor(bit_plane_1, bit_plane_2, bit_plane_1); //矩阵进行异或操作
			bitwise_xor(bit_plane_1, bit_plane_2, bit_plane_1);
		}
		else {
			//cvCopyImage(bit_plane_2, bit_plane_1); //bit_plane_2复制到了bit_plane_1，cvCopyImage使用时目标矩阵必须提前分配内存
			bit_plane_1 = bit_plane_2;
		}

		// cvAddS(decoded_cols, cvScalar(pow(2.0,n_cols-i-1)), decoded_cols, bit_plane_1);     //图像加常量
		// cvAddS(decoded_cols, cvScalar(pow(2.0,n_cols-i-1)), decoded_cols, bit_plane_1);     //图像加常量

	}

	cvSubS(decoded_cols, cvScalar(col_shift), decoded_cols);            //矩阵和值做减法  decoded_cols = decoded_cols - cvScalar(col_shift)

	/*
	// Decode Gray codes for projector rows.
	cvZero(decoded_rows);
	for(int i=0; i<n_rows; i++){
		// Decode bit-plane and update mask.
		cvCvtColor(gray_codes[2*(i+n_cols+1)],   gray_1, CV_RGB2GRAY);
		cvCvtColor(gray_codes[2*(i+n_cols+1)+1], gray_2, CV_RGB2GRAY);
		cvAbsDiff(gray_1, gray_2, temp);
		cvCmpS(temp, sl_thresh, temp, CV_CMP_GE);
		cvOr(temp, mask, mask);
		cvCmp(gray_1, gray_2, bit_plane_2, CV_CMP_GE);

		// Convert from gray code to decimal value.
		if(i>0)
			cvXor(bit_plane_1, bit_plane_2, bit_plane_1);
		else
			cvCopyImage(bit_plane_2, bit_plane_1);

		cvAddS(decoded_rows, cvScalar(pow(2.0,n_rows-i-1)), decoded_rows, bit_plane_1);
	}

	cvSubS(decoded_rows, cvScalar(row_shift), decoded_rows);
	*/


	// Eliminate invalid column/row estimates.
	// Note: This will exclude pixels if either the column or row is missing or erroneous.
	cvCmpS(decoded_cols, proj_width-1,  temp, CV_CMP_LE);
	cvAnd(temp, mask, mask);
	cvCmpS(decoded_cols, 0,  temp, CV_CMP_GE);
	cvAnd(temp, mask, mask);
	cvCmpS(decoded_rows, proj_height-1, temp, CV_CMP_LE);
	cvAnd(temp, mask, mask);
	cvCmpS(decoded_rows, 0,  temp, CV_CMP_GE);
	cvAnd(temp, mask, mask);
	cvNot(mask, temp);
	cvSet(decoded_cols, cvScalar(NULL), temp);
	cvSet(decoded_rows, cvScalar(NULL), temp);

	// Free allocated resources.
	cvReleaseImage(&gray_1);
	cvReleaseImage(&gray_2);
	cvReleaseImage(&bit_plane_1);
	cvReleaseImage(&bit_plane_2);
	cvReleaseImage(&temp);

	// Return without errors.
	return 0;
}
#endif

