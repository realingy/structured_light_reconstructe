#include <opencv2/opencv.hpp>
#include <string>

using namespace cv;
using namespace std;

//const int gHeight = 1280;
//const int gWidth = 768;
const int gWidth = 1280;
const int gHeight = 720;

/*
float I0[gWidth * gHeight];
float I1[gWidth * gHeight];
float I2[gWidth * gHeight];
float I3[gWidth * gHeight];
*/
Mat phase0(gHeight, gWidth, CV_32F);
Mat phase1(gHeight, gWidth, CV_32F);
Mat phase2(gHeight, gWidth, CV_32F);
Mat phase3(gHeight, gWidth, CV_32F);

// 三频率, 决定正弦函数的周期
int freq[] = { 70, 64, 59 };

// 存储3组共计12张图(三个频率，四个相位)
Mat image[3][4];

// 生成3组共计12张pattern图(三个频率，四个相位)
Mat pattern[3][4];

// 相位主值（包裹相位）图像
Mat imageUnwrappedPhase[3];

// 利用余弦函数生成12张pattern图
void PasheShiftPatternGenerator(bool vertical)
{
	for (int i = 0; i < 3; i++) //三个频率
	{
		for (int j = 0; j < 4; j++) //四个相位
		{
			pattern[i][j] = Mat(gHeight, gWidth, CV_32F);
			// 遍历图像填充数据
			for (int r = 0; r < gHeight; r++) {
				float* ptr = pattern[i][j].ptr<float>(r);
				for (int l = 0; l < gWidth; l++) {
					ptr[l] = (float)(128.0 + (float)127.0 * sin(2 * CV_PI * l * freq[i] / gWidth + j * CV_PI / 2));
				}
			}

			// 灰度归一化

			// 保存pattern图像
			stringstream ss;
			string filename;
			ss << "pattern/vPhase_" << i << "_" << j << ".bmp";
			ss >> filename;
			cout << "save pattern: " << filename << endl;
			cv::imwrite(filename, pattern[i][j]);
			ss.clear();
		}
	}
}

/*
% 求取相位差
% 计算每种频率对应的相位主值
% 输出三种频率的相位主值，用于相差计算
*/
void CalWrappedPhase()
{
	// 对于3组中的每一组图片，每一组相同频率的有四张图片
	for(int n = 0; n < 3; n++)
	{
		phase0 = pattern[n][0];
		phase1 = pattern[n][1];
		phase2 = pattern[n][2];
		phase3 = pattern[n][3];

		// 包裹相位图像（每个频率有一个对应的包裹相位图像）
		imageUnwrappedPhase[n] = Mat(gHeight, gWidth, CV_32F);

		for (int i = 0; i < gHeight; i++) {
			for (int j = 0; j < gWidth; j++) {
				
				float & I0 = phase0.at<float>(i, j);
				float & I1 = phase1.at<float>(i, j);
				float & I2 = phase0.at<float>(i, j);
				float & I3 = phase0.at<float>(i, j);

				if (I3.at<float>(i, j) == I1.at<float>(i, j) &&
					I0.at<float>(i, j) > I2.at<float>(i, j)) // 四个特殊位置
				{
					imageUnwrappedPhase[n].at<float>(i,j) = 0;
				}
				else if (I3.at<float>(i, j) == I1.at<float>(i, j) &&
					I0.at<float>(i, j) < I2.at<float>(i, j)) // 四个特殊位置
				{
					imageUnwrappedPhase[n].at<float>(i,j) = CV_PI;
				}
				else if (I3.at<float>(i, j) > I1.at<float>(i, j) &&
					I0.at<float>(i, j) == I2.at<float>(i, j)) // 四个特殊位置
				{
					imageUnwrappedPhase[n].at<float>(i,j) = CV_PI / 2;
				}
				else if (I3.at<float>(i, j) < I1.at<float>(i, j) &&
					I0.at<float>(i, j) == I2.at<float>(i, j)) // 四个特殊位置
				{
					imageUnwrappedPhase[n].at<float>(i,j) = 3 * CV_PI / 2;
				}
				else if (I0.at<float>(i, j) < I2.at<float>(i, j))
				{
					imageUnwrappedPhase[n].at<float>(i,j) = atan((I3(g, k) - I1(g, k)) / (I0(g, k) - I2(g, k))) + CV_PI;
				}

			}
		}

	}
}

/*
// 输入：包裹相位
// 输出：绝对相位（展开相位）
void decMultiPhase5(Mat *imgShift, Mat &imgAbsPhase)
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
		
	return 0;
}

