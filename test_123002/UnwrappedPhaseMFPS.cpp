#include <opencv2/opencv.hpp>
#include <string>
#include <fstream>

using namespace cv;
using namespace std;

#if 1
//const int gHeight = 1280;
//const int gWidth = 768;
//const int gWidth = 1280;
//const int gHeight = 720;
const int gWidth = 1600;
const int gHeight = 1200;

// 三频率, 决定正弦函数的周期
// int freq[] = { 70, 64, 59 };
// int freq[] = { 100, 94.75, 90.5 };
int freq[] = { 100, 94, 89 };

// 存储3组共计12张图(三个频率，四个相位)
Mat image[3][4];

// 生成3组共计12张pattern图(三个频率，四个相位)
Mat pattern[3][4];

// 相位主值（包裹相位）图像
Mat imageWrappedPhase[3];

// 利用余弦函数生成12张pattern图
void PasheShiftPatternGenerator(bool vertical)
{
	cout << "\n============================================================================" << endl;
	cout << "1 根据三个频率生成三种相位移pattern" << endl;
	for (int i = 0; i < 3; i++) //三个频率
	{
		for (int j = 0; j < 4; j++) //四个相位
		{
			pattern[i][j] = Mat(gHeight, gWidth, CV_32F);
			// 遍历图像填充数据
			for (int r = 0; r < gHeight; r++) {
				float* ptr = pattern[i][j].ptr<float>(r);
				for (int l = 0; l < gWidth; l++) {
					// ptr[l] = 128.0 + 127.0 * sin(2 * CV_PI * l * freq[i] / gWidth + j * CV_PI / 2);
					ptr[l] = 127.0 * (sin(2 * CV_PI * l * freq[i] / gWidth + j * CV_PI / 2) + 1);
				}
			}

			// 保存pattern图像
			stringstream ss;
			string filename;
			ss << "pattern/vPhase_" << i << "_" << j << ".bmp";
			ss >> filename;
			cout << "save pattern: " << filename << endl;
			cv::imwrite(filename, pattern[i][j]);
			ss.clear();

			// 灰度归一化
			cv::normalize(pattern[i][j], pattern[i][j]);
		}
	}
}

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
	
	for(int n = 0; n < 3; n++)
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
				float I1 = phase2.at<float>(i, j);
				float I2 = phase3.at<float>(i, j);
				float I3 = phase4.at<float>(i, j);
				float I4 = phase1.at<float>(i, j);

				//(I4-I2)/(I1-I3)
				if (I4 == I2 && I1 > I3 ) // 四个特殊位置
				{
					imageWrappedPhase[n].at<float>(i,j) = 0;
				}
				else if (I4 == I2 && I1 < I3 ) // 四个特殊位置
				{
					imageWrappedPhase[n].at<float>(i,j) = CV_PI;
				}
				else if (I4 > I2 && I1 == I3 ) // 四个特殊位置
				{
					imageWrappedPhase[n].at<float>(i,j) = CV_PI / 2;
				}
				else if (I4 < I2 && I1 == I3 ) // 四个特殊位置
				{
					imageWrappedPhase[n].at<float>(i,j) = 3 * CV_PI / 2;
				}
				else if ( I1 < I3 ) //第二、三象限
				{
					imageWrappedPhase[n].at<float>(i,j) = atan( (I4 - I2) / (I1 - I3) ) + CV_PI;
				}
				else if (I1 > I3 && I4 > I2) //第一象限
				{
					imageWrappedPhase[n].at<float>(i,j) = atan( (I4 - I2) / (I1 - I3) );
				}
				else if (I1 > I3&& I4 < I2) //第四象限
				{
					imageWrappedPhase[n].at<float>(i, j) = atan((I4 - I2) / (I1 - I3)) + 2*CV_PI;
				}
			}
		}
	}

	/*
	int series[3] = { 0 };
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < gWidth; j++) {
			if (imageWrappedPhase[i].at<float>(0, j) == 0.0)
				series[i]++;
		}
	}

	for (int i = 0; i < 3; i++) {
		cout << "第" << i << "个主相位包含" << series[i] << "个级次\n";
	}
	*/

	/*
	ofstream file("output/wrapphase1.txt");
	for (int i = 0; i < gHeight; i++)
	{
		for (int j = 0; j < gWidth; j++)
		{
			file << imageWrappedPhase[0].at<float>(i, j) << "\t";
		}
		file << endl;
	}

	ofstream file2("output/wrapphase2.txt");
	for (int i = 0; i < gHeight; i++)
	{
		for (int j = 0; j < gWidth; j++)
		{
			file2 << imageWrappedPhase[1].at<float>(i, j) << "\t";
		}
		file2 << endl;
	}

	ofstream file3("output/wrapphase3.txt");
	for (int i = 0; i < gHeight; i++)
	{
		for (int j = 0; j < gWidth; j++)
		{
			file3 << imageWrappedPhase[2].at<float>(i, j) << "\t";
		}
		file3 << endl;
	}
	*/
}

// 计算相位差（解包裹相位/相位展开）
void CalPhaseDifference()
{
	cout << "\n============================================================================" << endl;
	cout << "3 根据三种频率的相位主值(包裹相位)，计算相位差（相位展开） " << endl;

	// 初始化相差变量
	// 多频相差
	Mat PH12 = Mat(gHeight, gWidth, CV_32F);
	Mat PH23 = Mat(gHeight, gWidth, CV_32F);
	Mat PH123 = Mat(gHeight, gWidth, CV_32F);

	// 两两相差计算
	for (int i = 0; i < gHeight; i++)
	{
		for (int j = 0; j < gWidth; j++)
		{
			float PH0 = imageWrappedPhase[0].at<float>(i, j);
			float PH1 = imageWrappedPhase[1].at<float>(i, j);
			float PH2 = imageWrappedPhase[2].at<float>(i, j);

			// 计算第一组和第二组的相位差
			if(PH0 > PH1) {
				PH12.at<float>(i, j) = PH0 - PH1;
			} else {
				PH12.at<float>(i, j) = PH0 - PH1 + 2*CV_PI;
			}
			// 计算第二组和第三组的相位差
			if(PH1 > PH2) {
				PH23.at<float>(i, j) = PH1 - PH2;
			} else {
				PH23.at<float>(i, j) = PH1 - PH2 + 2*CV_PI;
			}
		}
	}

	// 计算最终相差
	for (int i = 0; i < gHeight; i++)
	{
		for (int j = 0; j < gWidth; j++)
		{
			if( PH12.at<float>(i,j) > PH23.at<float>(i, j) ) {
				PH123.at<float>(i, j) = PH12.at<float>(i, j) - PH23.at<float>(i, j);
			} else {
				PH123.at<float>(i, j) = PH12.at<float>(i, j) - PH23.at<float>(i, j) + 2*CV_PI;
			}
		}
	}

	cv::normalize(PH12, PH12, 0, 255, NORM_MINMAX);
	cv::normalize(PH23, PH23, 0, 255, NORM_MINMAX);
	cv::normalize(PH123, PH123, 0, 255, NORM_MINMAX);

	cout << "saving phase diff of phase 1 & phase 2\n";
	cv::imwrite("output/PhaseDiff12.bmp", PH12);
	cout << "saving phase diff of phase 2 & phase 3\n";
	cv::imwrite("output/PhaseDiff23.bmp", PH23);
	cout << "saving phase diff of phase 1 & phase 2 & phase 3\n";
	cv::imwrite("output/PhaseDiff123.bmp", PH123);

	// 灰度归一化
	for (size_t i = 0; i < 3; i++)
	{
		cv::normalize(imageWrappedPhase[i], imageWrappedPhase[i], 0, 255, NORM_MINMAX);
	}
	cout << "saving wrapped phase 1\n";
	cv::imwrite("output/WrapPhase1.bmp", imageWrappedPhase[0]);
	cout << "saving wrapped phase 2\n";
	cv::imwrite("output/WrapPhase2.bmp", imageWrappedPhase[1]);
	cout << "saving wrapped phase 3\n";
	cv::imwrite("output/WrapPhase3.bmp", imageWrappedPhase[2]);
}


/*
// 输入：包裹相位
// 输出：绝对相位（展开相位）
void decMultiPhase(Mat *imgShift, Mat &imgAbsPhase)
{
	//获取包裹相位
	float * dPtr  = (float *)imgAbsPhase.data;
	Mat imgPhase[5] ;
	for(int k=0; k<5; k++)
		imgPhase[k] = Mat::zeros(m_nHeight, m_nWidth, CV_32FC1);

	for(int n = 0;n<4;n++)
	{
		float *pha=  (float *)imgPhase[n].data;
		for(int k=0; k < m_nWidth*m_nHeight; k++)
		{
			pha[k] =  (float)sqrt( atan2((double)(I1[k]-I3[k]),(double)(I0[k]-I2[k])) ) ;
		}
	}
 
	Mat imgAbsPhase1= Mat::zeros(m_nHeight, m_nWidth, CV_32FC1);
	Mat imgAbsPhase2= Mat::zeros(m_nHeight, m_nWidth, CV_32FC1);
	Mat imgAbsPhase3= Mat::zeros(m_nHeight, m_nWidth, CV_32FC1);
 
	phaseUnWrap(imgPhase[0],  imgPhase[1], imgAbsPhase1, m_dFreq[0], m_dFreq[1]);
	phaseUnWrap(imgAbsPhase1, imgPhase[2], imgAbsPhase2, m_dFreq[1], m_dFreq[2]);
	phaseUnWrap(imgAbsPhase2, imgPhase[3], imgAbsPhase3, m_dFreq[2], m_dFreq[3]);
	phaseUnWrap(imgAbsPhase3, imgPhase[4], imgAbsPhase,  m_dFreq[3], m_dFreq[4]);
 
//	ImgShowPhase(imgPhase[0],"0");
//	ImgShowPhase(imgPhase[1],"1");    
//	ImgShowPhase(imgPhase[2],"2");
//	ImgShowPhase(imgPhase[3],"3");
//	ImgShowPhase(imgPhase[4],"4");
//  cvWaitKey(1000000);

//  ImgShowAbsPhase(imgPhase[0],PI2,"abs0");
//  ImgShowAbsPhase(imgAbsPhase1,3*PI2,"abs1");
//  ImgShowAbsPhase(imgAbsPhase2,9*PI2,"abs2");
//  ImgShowAbsPhase(imgAbsPhase3,27*PI2,"abs3");
//  ImgShowAbsPhase(imgAbsPhase,81*PI2,"abs4");
//  cvWaitKey(1000000);
}
*/


int main()
{
	PasheShiftPatternGenerator(true); //生成pattern
	CalImageWrappedPhase(); // 包裹相位计算
	CalPhaseDifference(); // 解包裹相位（相位展开）

	return 0;
}
#endif

