#include "UnwrappedPhaseGrayCode.h"

// #define TEST

#ifndef TEST
const int gWidth = 1280;
const int gHeight = 720;
#else
const int gWidth = 1292;
const int gHeight = 964;
#endif

// 相移编码图像
Mat pattern[4];

// 相移图像
Mat image_phase[4];

// 相位主值
Mat image_wrapped_phase;
Mat phase_series;

// 格雷码图像
Mat gray_codes[7];

void PhaseShiftPatternGenerate(int freq)
{
	cout << "\n============================================================================" << endl;
	cout << "generate the structured light pattern." << endl;
	for (int j = 0; j < 4; j++) //四个相位
	{
		pattern[j] = Mat(gHeight, gWidth, CV_32F);
		// 遍历图像填充数据
		for (int r = 0; r < gHeight; r++) {
			float* ptr = pattern[j].ptr<float>(r);
			for (int l = 0; l < gWidth; l++) {
				//ptr[l] = 128.0 + 127.0 * sin(2 * CV_PI * l * freq / gWidth + j * CV_PI / 2);
				ptr[l] = 127.0 * (sin(2 * CV_PI * l * freq / gWidth + j * CV_PI / 2) + 1);
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

void LoadImage()
{	
	cout << "=====================================================" << endl;
	cout << "load gray code images" << endl;
#ifndef TEST
	gray_codes[0] = imread("pattern/vGray1.bmp", CV_LOAD_IMAGE_GRAYSCALE);
	gray_codes[1] = imread("pattern/vGray3.bmp", CV_LOAD_IMAGE_GRAYSCALE);
	gray_codes[2] = imread("pattern/vGray5.bmp", CV_LOAD_IMAGE_GRAYSCALE);
	gray_codes[3] = imread("pattern/vGray7.bmp", CV_LOAD_IMAGE_GRAYSCALE);
	gray_codes[4] = imread("pattern/vGray9.bmp", CV_LOAD_IMAGE_GRAYSCALE);
	gray_codes[5] = imread("pattern/vGray11.bmp", CV_LOAD_IMAGE_GRAYSCALE);
	gray_codes[6] = imread("pattern/vGray13.bmp", CV_LOAD_IMAGE_GRAYSCALE);
#else
	gray_codes[0] = imread("image/Graycodeleft01.bmp", CV_LOAD_IMAGE_GRAYSCALE);
	gray_codes[1] = imread("image/Graycodeleft01.bmp", CV_LOAD_IMAGE_GRAYSCALE);
	gray_codes[2] = imread("image/Graycodeleft01.bmp", CV_LOAD_IMAGE_GRAYSCALE);
	gray_codes[3] = imread("image/Graycodeleft01.bmp", CV_LOAD_IMAGE_GRAYSCALE);
	gray_codes[4] = imread("image/Graycodeleft01.bmp", CV_LOAD_IMAGE_GRAYSCALE);
	gray_codes[5] = imread("image/Graycodeleft01.bmp", CV_LOAD_IMAGE_GRAYSCALE);
#endif

	cout << "load phase shift images" << endl;
#ifndef TEST 
	image_phase[0] = imread("pattern/vPhase_0.bmp", CV_LOAD_IMAGE_GRAYSCALE);
	image_phase[1] = imread("pattern/vPhase_1.bmp", CV_LOAD_IMAGE_GRAYSCALE);
	image_phase[2] = imread("pattern/vPhase_2.bmp", CV_LOAD_IMAGE_GRAYSCALE);
	image_phase[3] = imread("pattern/vPhase_3.bmp", CV_LOAD_IMAGE_GRAYSCALE);
#else
	image_phase[0] = imread("image/PhaseShiftleft01.bmp", CV_LOAD_IMAGE_GRAYSCALE);
	image_phase[1] = imread("image/PhaseShiftleft01.bmp", CV_LOAD_IMAGE_GRAYSCALE);
	image_phase[2] = imread("image/PhaseShiftleft01.bmp", CV_LOAD_IMAGE_GRAYSCALE);
	image_phase[3] = imread("image/PhaseShiftleft01.bmp", CV_LOAD_IMAGE_GRAYSCALE);
#endif

}

// 计算相位主值(包裹相位)
void CalWrappedPhase()
{
	cout << "\n=====================================================" << endl;
	cout << "Cal Wrapped Phase!" << endl;
	Mat phase1 = image_phase[0];
	Mat phase2 = image_phase[1];
	Mat phase3 = image_phase[2];
	Mat phase4 = image_phase[3];

	// 包裹相位图（每个频率有一个对应的包裹相位图）
	image_wrapped_phase = Mat(gHeight, gWidth, CV_32F);

	for (int i = 0; i < gHeight; i++)
	{
		for (int j = 0; j < gWidth; j++)
		{
			float I1 = (float)phase2.at<uchar>(i, j);
			float I2 = (float)phase3.at<uchar>(i, j);
			float I3 = (float)phase4.at<uchar>(i, j);
			float I4 = (float)phase1.at<uchar>(i, j);
			
			if (I4 == I2 && I1 > I3) // 四个特殊位置
			{
				image_wrapped_phase.at<float>(i, j) = 0;
			}
			else if (I4 == I2 && I1 < I3) // 四个特殊位置
			{
				image_wrapped_phase.at<float>(i, j) = CV_PI;
			}
			else if (I4 > I2&& I1 == I3) // 四个特殊位置
			{
				image_wrapped_phase.at<float>(i, j) = CV_PI / 2;
			}
			else if (I4 < I2 && I1 == I3) // 四个特殊位置
			{
				image_wrapped_phase.at<float>(i, j) = 3 * CV_PI / 2;
			}
			else if (I1 < I3) //第二、三象限
			{
				image_wrapped_phase.at<float>(i, j) = atan((I4 - I2) / (I1 - I3)) + CV_PI;
			}
			else if (I1 > I3&& I4 > I2) //第一象限
			{
				image_wrapped_phase.at<float>(i, j) = atan((I4 - I2) / (I1 - I3));
			}
			else if (I1 > I3&& I4 < I2) //第四象限
			{
				image_wrapped_phase.at<float>(i, j) = atan((I4 - I2) / (I1 - I3)) + 2 * CV_PI;
			}
		}
	}

	ofstream file("data/wrapphase.txt");
	for (int i = 0; i < gHeight; i++)
	{
		for (int j = 0; j < gWidth; j++)
		{
			file << image_wrapped_phase.at<float>(i, j) << "\t";
		}
		file << endl;
	}
}

// 解包裹相位(相位展开)
void CalUnwrappedPhase()
{
	cout << "\n=====================================================" << endl;
	cout << "Cal Unwrapped Phase!" << endl;
	Mat img1 = gray_codes[0];
	Mat img2 = gray_codes[1];
	Mat img3 = gray_codes[2];
	Mat img4 = gray_codes[3];
	Mat img5 = gray_codes[4];
	Mat img6 = gray_codes[5];
	Mat img7 = gray_codes[6];

	// 二值化阈值
	uchar thresh = 130; // 0-255 model21:200   model1:127

	//threshold(img1, img1, thresh, 1, CV_THRESH_BINARY);
	threshold(img1, img1, thresh, 1, CV_THRESH_BINARY);
	threshold(img2, img2, thresh, 1, CV_THRESH_BINARY);
	threshold(img3, img3, thresh, 1, CV_THRESH_BINARY);
	threshold(img4, img4, thresh, 1, CV_THRESH_BINARY);
	threshold(img5, img5, thresh, 1, CV_THRESH_BINARY);
	threshold(img6, img6, thresh, 1, CV_THRESH_BINARY);
	threshold(img7, img7, thresh, 1, CV_THRESH_BINARY);

	bitwise_xor(img1, img2, img2);
	bitwise_xor(img2, img3, img3);
	bitwise_xor(img3, img4, img4);
	bitwise_xor(img4, img5, img5);
	bitwise_xor(img5, img6, img6);
	bitwise_xor(img6, img7, img7);

	// 相位序列Mat
	phase_series = Mat(gHeight, gWidth, CV_8UC1, Scalar(0.0));

	uchar pre_series, cur_series;
	float pre_unphase, cur_unphase;
	for (int y = 0; y < gHeight; y++)
	{
		for (int x = 0; x < gWidth; x++)
		{
			if (6 == gray_bit)
			{
				phase_series.at<uchar>(y, x) = img1.at<uchar>(y, x) * 32
												+ img2.at<uchar>(y, x) * 16
												+ img3.at<uchar>(y, x) * 8
												+ img4.at<uchar>(y, x) * 4
												+ img5.at<uchar>(y, x) * 2
												+ img6.at<uchar>(y, x) * 1;
			}
			if (7 == gray_bit)
			{
				phase_series.at<uchar>(y, x) = img1.at<uchar>(y, x) * 64
												+ img2.at<uchar>(y, x) * 32
												+ img3.at<uchar>(y, x) * 16
												+ img4.at<uchar>(y, x) * 8
												+ img5.at<uchar>(y, x) * 4
												+ img6.at<uchar>(y, x) * 2
												+ img7.at<uchar>(y, x) * 1;
			}
		}
	}

	medianBlur(phase_series, phase_series, 9); //中值滤波

	ofstream file("data/phase_series.txt");
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
			dst.at<float>(y, x) = phase_series.at<uchar>(y, x) * 2 * CV_PI + image_wrapped_phase.at<float>(y, x);
		}
	}

	int count = 0;
	for (int i = 0; i < gHeight; i++)
	{
		for (int j = 1; j < gWidth - 1; j++)
		{
			float dist = dst.at<float>(i, j) - dst.at<float>(i, j - 1);
			if (dist < 0)
			{
				// dst.at<float>(i, j) = (dst.at<float>(i, j - 1) + dst.at<float>(i, j + 1)) / 2;
				count++;
			}
		}
	}

	// 周期错位像素数
	cout << "period dislocation pixels count: " << count << endl;


	cv::normalize(image_wrapped_phase, image_wrapped_phase, 0, 255, NORM_MINMAX);
	cout << "saving wrapped phase \n";
	cv::imwrite("data/WrapPhase.bmp", image_wrapped_phase);

	cv::normalize(phase_series, phase_series, 0, 255, NORM_MINMAX);
	cout << "saving phase series \n";
	cv::imwrite("data/phase_series.bmp", phase_series);

	cv::normalize(dst, dst, 0, 255, NORM_MINMAX);
	cout << "saving unwrapped phase \n";
	cv::imwrite("data/UnwrappedPhase.bmp", dst);

	ofstream file2("data/unwrapphase.txt");
	for (int i = 0; i < gHeight; i++)
	{
		for (int j = 0; j < gWidth; j++)
		{
			file2 << dst.at<float>(i, j) << "\t";
		}
		file2 << endl;
	}
	file2.close();
}


