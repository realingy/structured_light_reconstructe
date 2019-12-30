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
Mat pattern_left[3][4];
Mat pattern_right[3][4];

// 相位主值（包裹相位）图像
Mat wrapped_phase_left[3];
Mat wrapped_phase_right[3];

// 绝对相位（解包裹相位）
Mat unwrapped_phase_left;
Mat unwrapped_phase_right;

#define PHASE_THRESHOLD 0.01

void savepnts3D(const char *filename, cv::Mat& mat);
void find_featurepionts_single_match(Mat& leftphase, Mat& rightphase, vector<Point2f>& leftkeypoint, vector<Point2f>& rightkeypoint);

// 利用余弦函数生成12张pattern图
void PhaseShiftPatternGenerator(bool vertical)
{
	cout << "\n============================================================================" << endl;
	cout << "1 根据三个频率生成三种相位移pattern" << endl;
	for (int i = 0; i < 3; i++) //三个频率
	{
		for (int j = 0; j < 4; j++) //四个相位
		{
			pattern_left[i][j] = Mat(gHeight, gWidth, CV_32F);
			pattern_right[i][j] = Mat(gHeight, gWidth, CV_32F);
			// 遍历图像填充数据
			for (int r = 0; r < gHeight; r++) {
				float* ptr_left = pattern_left[i][j].ptr<float>(r);
				float* ptr_right = pattern_right[i][j].ptr<float>(r);
				for (int l = 0; l < gWidth; l++) {
					// ptr[l] = 128.0 + 127.0 * sin(2 * CV_PI * l * freq[i] / gWidth + j * CV_PI / 2);
					ptr_left[l] = 127.0 * (sin(2 * CV_PI * l * freq[i] / gWidth + j * CV_PI / 2) + 1);
					ptr_right[l] = 127.0 * (sin(2 * CV_PI * l * freq[i] / gWidth + j * CV_PI / 2) + 1);
				}
			}

			// 保存左pattern图像
			stringstream ss1;
			string filename1;
			ss1 << "../pattern/pattern_l/vPhase_" << i << "_" << j << ".bmp";
			ss1 >> filename1;
			cout << "save pattern: " << filename1 << endl;
			cv::imwrite(filename1, pattern_left[i][j]);
			ss1.clear();

			// 保存右pattern图像
			stringstream ss2;
			string filename2;
			ss2 << "../pattern/pattern_r/vPhase_" << i << "_" << j << ".bmp";
			ss2 >> filename2;
			cout << "save pattern: " << filename2 << endl;
			cv::imwrite(filename2, pattern_right[i][j]);
			ss2.clear();

			// 灰度归一化
			//cv::normalize(pattern[i][j], pattern[i][j]);
		}
	}
}

// 计算每种频率对应的相位主值
// 输出三种频率的相位主值(包裹相位)，用于相差计算
void CalWrappedPhaseLeft()
{
	cout << "\n============================================================================" << endl;
	cout << "2 计算每种频率对应的相位主值, 输出三种频率的相位主值(包裹相位)，用于相差计算 " << endl;

	Mat phase1(gHeight, gWidth, CV_32F);
	Mat phase2(gHeight, gWidth, CV_32F);
	Mat phase3(gHeight, gWidth, CV_32F);
	Mat phase4(gHeight, gWidth, CV_32F);
	
	for(int n = 0; n < 3; n++)
	{
		phase1 = pattern_left[n][0];
		phase2 = pattern_left[n][1];
		phase3 = pattern_left[n][2];
		phase4 = pattern_left[n][3];

		// 包裹相位图（每个频率有一个对应的包裹相位图）
		wrapped_phase_left[n] = Mat(gHeight, gWidth, CV_32F);

		for (int i = 0; i < gHeight; i++)
		{
			for (int j = 0; j < gWidth; j++)
			{
				float I1 = phase2.at<float>(i, j);
				float I2 = phase3.at<float>(i, j);
				float I3 = phase4.at<float>(i, j);
				float I4 = phase1.at<float>(i, j);

				if (I4 == I2 && I1 > I3 ) // 四个特殊位置
				{
					wrapped_phase_left[n].at<float>(i, j) = 0;
				}
				else if (I4 == I2 && I1 < I3 ) // 四个特殊位置
				{
					wrapped_phase_left[n].at<float>(i, j) = CV_PI;
				}
				else if (I4 > I2 && I1 == I3 ) // 四个特殊位置
				{
					wrapped_phase_left[n].at<float>(i, j) = CV_PI / 2;
				}
				else if (I4 < I2 && I1 == I3 ) // 四个特殊位置
				{
					wrapped_phase_left[n].at<float>(i, j) = 3*CV_PI / 2;
				}
				else if ( I1 < I3 ) //第二、三象限
				{
					wrapped_phase_left[n].at<float>(i,j) = atan( (I4 - I2) / (I1 - I3) ) + CV_PI;
				}
				else if (I1 > I3 && I4 > I2) //第一象限
				{
					wrapped_phase_left[n].at<float>(i,j) = atan( (I4 - I2) / (I1 - I3) );
				}
				else if (I1 > I3&& I4 < I2) //第四象限
				{
					wrapped_phase_left[n].at<float>(i, j) = atan((I4 - I2) / (I1 - I3)) + 2*CV_PI;
				}
			}
		}
	}
}

void CalWrappedPhaseRight()
{
	cout << "\n============================================================================" << endl;
	cout << "2 计算每种频率对应的相位主值, 输出三种频率的相位主值(包裹相位)，用于相差计算 " << endl;

	Mat phase1(gHeight, gWidth, CV_32F);
	Mat phase2(gHeight, gWidth, CV_32F);
	Mat phase3(gHeight, gWidth, CV_32F);
	Mat phase4(gHeight, gWidth, CV_32F);
	
	for(int n = 0; n < 3; n++)
	{
		phase1 = pattern_right[n][0];
		phase2 = pattern_right[n][1];
		phase3 = pattern_right[n][2];
		phase4 = pattern_right[n][3];

		// 包裹相位图（每个频率有一个对应的包裹相位图）
		wrapped_phase_right[n] = Mat(gHeight, gWidth, CV_32F);

		for (int i = 0; i < gHeight; i++)
		{
			for (int j = 0; j < gWidth; j++)
			{
				float I1 = phase2.at<float>(i, j);
				float I2 = phase3.at<float>(i, j);
				float I3 = phase4.at<float>(i, j);
				float I4 = phase1.at<float>(i, j);

				if (I4 == I2 && I1 > I3 ) // 四个特殊位置
				{
					wrapped_phase_right[n].at<float>(i, j) = 0;
				}
				else if (I4 == I2 && I1 < I3 ) // 四个特殊位置
				{
					wrapped_phase_right[n].at<float>(i, j) = CV_PI;
				}
				else if (I4 > I2 && I1 == I3 ) // 四个特殊位置
				{
					wrapped_phase_right[n].at<float>(i, j) = CV_PI / 2;
				}
				else if (I4 < I2 && I1 == I3 ) // 四个特殊位置
				{
					wrapped_phase_right[n].at<float>(i, j) = 3*CV_PI / 2;
				}
				else if ( I1 < I3 ) //第二、三象限
				{
					wrapped_phase_right[n].at<float>(i,j) = atan( (I4 - I2) / (I1 - I3) ) + CV_PI;
				}
				else if (I1 > I3 && I4 > I2) //第一象限
				{
					wrapped_phase_right[n].at<float>(i,j) = atan( (I4 - I2) / (I1 - I3) );
				}
				else if (I1 > I3&& I4 < I2) //第四象限
				{
					wrapped_phase_right[n].at<float>(i, j) = atan((I4 - I2) / (I1 - I3)) + 2*CV_PI;
				}
			}
		}
	}

}
// 计算相位差（解包裹相位/相位展开）
void CalPhaseDifferenceLeft()
{
	cout << "\n============================================================================" << endl;
	cout << "3 根据三种频率的相位主值(包裹相位)，计算相位差（相位展开） " << endl;

	unwrapped_phase_left = Mat(gHeight, gWidth, CV_32F);

	// 初始化相差变量
	// 多频相差
	Mat PH12 = Mat(gHeight, gWidth, CV_32F);
	Mat PH23 = Mat(gHeight, gWidth, CV_32F);

	// 两两相差计算
	for (int i = 0; i < gHeight; i++)
	{
		for (int j = 0; j < gWidth; j++)
		{
			float PH0 = wrapped_phase_left[0].at<float>(i, j);
			float PH1 = wrapped_phase_left[1].at<float>(i, j);
			float PH2 = wrapped_phase_left[2].at<float>(i, j);

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
				unwrapped_phase_left.at<float>(i, j) = PH12.at<float>(i, j) - PH23.at<float>(i, j);
			} else {
				unwrapped_phase_left.at<float>(i, j) = PH12.at<float>(i, j) - PH23.at<float>(i, j) + 2*CV_PI;
			}
		}
	}

	cv::normalize(PH12, PH12, 0, 255, NORM_MINMAX);
	cv::normalize(PH23, PH23, 0, 255, NORM_MINMAX);
	Mat PH123 = unwrapped_phase_left.clone();
	cv::normalize(unwrapped_phase_left, PH123, 0, 255, NORM_MINMAX);

	/*
	cout << "saving phase diff of phase 1 & phase 2\n";
	cv::imwrite("../output/phase_diff12_left.bmp", PH12);
	cout << "saving phase diff of phase 2 & phase 3\n";
	cv::imwrite("../output/phase_diff23_left.bmp", PH23);
	*/
	cout << "saving phase diff of phase 1 & phase 2 & phase 3\n";
	cv::imwrite("../output/unwrapped_phase_left.bmp", PH123);

	// 灰度归一化
	/*
	for (size_t i = 0; i < 3; i++)
	{
		cv::normalize(wrapped_phase_left[i], wrapped_phase_left[i], 0, 255, NORM_MINMAX);
	}
	cout << "saving wrapped phase 1\n";
	cv::imwrite("../output/wrap_phase1_left.bmp", wrapped_phase_left[0]);
	cout << "saving wrapped phase 2\n";
	cv::imwrite("../output/wrap_phase2_left.bmp", wrapped_phase_left[1]);
	cout << "saving wrapped phase 3\n";
	cv::imwrite("../output/wrap_phase3_left.bmp", wrapped_phase_left[2]);
	*/
}

void CalPhaseDifferenceRight()
{
	cout << "\n============================================================================" << endl;
	cout << "3 根据三种频率的相位主值(包裹相位)，计算相位差（相位展开） " << endl;

	unwrapped_phase_right = Mat(gHeight, gWidth, CV_32F);

	// 初始化相差变量
	// 多频相差
	Mat PH12 = Mat(gHeight, gWidth, CV_32F);
	Mat PH23 = Mat(gHeight, gWidth, CV_32F);

	// 两两相差计算
	for (int i = 0; i < gHeight; i++)
	{
		for (int j = 0; j < gWidth; j++)
		{
			float PH0 = wrapped_phase_right[0].at<float>(i, j);
			float PH1 = wrapped_phase_right[1].at<float>(i, j);
			float PH2 = wrapped_phase_right[2].at<float>(i, j);

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
				unwrapped_phase_right.at<float>(i, j) = PH12.at<float>(i, j) - PH23.at<float>(i, j);
			} else {
				unwrapped_phase_right.at<float>(i, j) = PH12.at<float>(i, j) - PH23.at<float>(i, j) + 2*CV_PI;
			}
		}
	}

	cv::normalize(PH12, PH12, 0, 255, NORM_MINMAX);
	cv::normalize(PH23, PH23, 0, 255, NORM_MINMAX);
	Mat PH123 = unwrapped_phase_right.clone();
	cv::normalize(unwrapped_phase_right, PH123, 0, 255, NORM_MINMAX);

	/*
	cout << "saving phase diff of phase 1 & phase 2\n";
	cv::imwrite("../output/phase_diff12_right.bmp", PH12);
	cout << "saving phase diff of phase 2 & phase 3\n";
	cv::imwrite("../output/phase_diff23_right.bmp", PH23);
	*/
	cout << "saving phase diff of phase 1 & phase 2 & phase 3\n";
	cv::imwrite("../output/unwrapped_phase_right.bmp", PH123);

	// 灰度归一化
	/*
	for (size_t i = 0; i < 3; i++)
	{
		cv::normalize(wrapped_phase_right[i], wrapped_phase_right[i], 0, 255, NORM_MINMAX);
	}
	cout << "saving wrapped phase 1\n";
	cv::imwrite("../output/wrap_phase1_right.bmp", wrapped_phase_right[0]);
	cout << "saving wrapped phase 2\n";
	cv::imwrite("../output/wrap_phase2_rightt.bmp", wrapped_phase_right[1]);
	cout << "saving wrapped phase 3\n";
	cv::imwrite("../output/wrap_phase3_right.bmp", wrapped_phase_right[2]);
	*/
}

int main()
{
	PhaseShiftPatternGenerator(true); //生成pattern

	CalWrappedPhaseLeft(); // 包裹相位(相位主值)计算
	CalWrappedPhaseRight(); // 包裹相位(相位主值)计算

	CalPhaseDifferenceLeft(); // 解包裹相位（相位展开/绝对相位计算）
	CalPhaseDifferenceRight(); // 解包裹相位（相位展开/绝对相位计算）

	vector<Point2f> leftfeaturepoints, rightfeaturepoints;
	cout << "\n======================================================" << endl;
	cout << ">>>4 Stereo match and 3D reconstruct" << endl;
	cout << "\n[1] Calculate feature points" << endl;

	// 三维重建
	//find_featurepionts(unwrapped_phase_left, unwrapped_phase_right, leftfeaturepoints, rightfeaturepoints);
	find_featurepionts_single_match(unwrapped_phase_left, unwrapped_phase_right, leftfeaturepoints, rightfeaturepoints);

	cout << "the number of feature: " << leftfeaturepoints.size() << endl;

	//cout << "\n==> P1:\n" << P1 << endl;
	//cout << "\n==> P2:\n" << P2 << endl;
	////////////////////////////////////////////////////////////////////////////////////////
	Mat R_T_1 = (Mat_<double>(3, 4) <<
		-0.003449992052740, 0.908392369471684, 0.418104533149851, 3.688988301573581,
		0.992268580264290, 0.054980888811595, -0.111266196509893, -4.927452164451585,
		-0.124061122738460, 0.414488124016969, -0.901558890408035, 329.276493470459510);

	Mat R_T_2 = (Mat_<double>(3, 4) <<
		-0.005778730523496, 0.970132888506089, 0.242505226567117, 3.780742082249347,
		0.992520961272705, 0.035135856240512, -0.116908567010947, -4.998608845649666,
		-0.121937474583672, 0.240015937481406, -0.963080267707255, 328.926407599367390);

	Mat cameraMatrix_1 = (Mat_<double>(3, 3) <<
		5004.084968538499200, -0.000186077310987, 796.177176385571330,
		0, 5004.288845428079200, 645.098858869668220,
		0, 0, 1);

	Mat cameraMatrix_2 = (Mat_<double>(3, 3) <<
		4991.369386877208900, 0.000227164222608, 786.153820356970750,
		0, 4991.811878028854200, 648.483429215111640,
		0, 0, 1);

	Mat PP1 = cameraMatrix_1 * R_T_1;
	Mat PP2 = cameraMatrix_2 * R_T_2;
	////////////////////////////////////////////////////////////////////////////////////////

	Mat points(4, leftfeaturepoints.size(), CV_64F);

	cout << "\n[2] Calculate points3D" << endl;
	// cv::triangulatePoints(P1, P2, leftfeaturepoints, rightfeaturepoints, points);
	cv::triangulatePoints(PP1, PP2, leftfeaturepoints, rightfeaturepoints, points);

	cout << "\n[3] Save points3D" << endl;
	const char* pnts3d_txt = "../output/points.txt";
	savepnts3D(pnts3d_txt, points);
	// savepointcloud(points);

	return 0;
}

// 亚像素级相位匹配
void find_featurepionts_single_match(Mat& leftphase, Mat& rightphase, vector<Point2f>& leftkeypoint, vector<Point2f>& rightkeypoint)
{
	int row = leftphase.rows;
	int col = leftphase.cols;
	int k;
	float left;
	Point2f point_left, point_right;
	int area = 10; //在边长40的区域中寻找匹配点

	for (int i = 0; i < row; i++)
	{
		for (int j = 0; j < col; j++)
		{
			left = leftphase.at<float>(i, j);

			if (left > 2 * CV_PI)
			{
				k = 0;

				while ((abs(left - rightphase.at<float>(i, k)) > PHASE_THRESHOLD) && (k < col)) k++;

				///////////////////////////////////////////////////////////////////////////
				//找右图上的相对匹配点
				//在以(i,k)为中心的区域内寻找相位最近的4个点
				std::map<float, Point2i> map_points_right;
				bool area_process = false;
				if (i > area && i<row - area && k>area && k < col - area)
				{
					area_process = true;
					map_points_right.insert(std::pair<float, Point2i>(abs(left - rightphase.at<float>(i - area, k - area)), Point2i(i - area, k - area)));
					map_points_right.insert(std::pair<float, Point2i>(abs(left - rightphase.at<float>(i - area, k - area + 1)), Point2i(i - area, k - area + 1)));
					map_points_right.insert(std::pair<float, Point2i>(abs(left - rightphase.at<float>(i - area, k - area + 2)), Point2i(i - area, k - area + 2)));
					map_points_right.insert(std::pair<float, Point2i>(abs(left - rightphase.at<float>(i - area, k - area + 3)), Point2i(i - area, k - area + 3)));
					//map_points.insert(make_pair( abs(left-rightphase.at<float>(i-area, k-area)), Point2i(i-area, k-area) ) );
					//map_points[1.2] = Point2i(10, 10);

					for (int m = i - area; m < i + area; m++)
					{
						for (int n = k - area; n < k + area; n++)
						{
							std::map<float, Point2i>::reverse_iterator rit = map_points_right.rbegin();
							if (abs(left - rightphase.at<float>(m, n)) < rit->first)
							{
								map_points_right.erase(rit->first);
								map_points_right.insert(std::pair<float, Point2i>(abs(left - rightphase.at<float>(m, n)), Point2i(m, n)));
							}
						}
					}
				}

				vector<Point2i> vec_points_right;
				vec_points_right.push_back(Point2i(i, k));
				if (true == area_process)
				{
					std::map<float, Point2i>::iterator it = map_points_right.begin();
					for (auto & x : map_points_right) {
						// 比(i, k)更合理的点添加到容器中
						if (x.first <= abs(left - rightphase.at<float>(i, k)))
							vec_points_right.push_back(x.second);
					}
				}

				int cor_x_sum = 0;
				int cor_y_sum = 0;
				float phase_right_total = 0.0;
				for (auto point : vec_points_right)
				{
					cor_x_sum += point.y;
					cor_y_sum += point.x;
					phase_right_total += rightphase.at<float>(point.x, point.y);
				}

				float cor_x_right = (float)cor_x_sum / vec_points_right.size();
				float cor_y_right = (float)cor_y_sum / vec_points_right.size();
				float cor_phase_right = (float)phase_right_total / vec_points_right.size();
				// float cor_phase_right = rightphase.at<float>( (int)cor_x_right, (int)cor_y_right );

				// cout << "<=" << cor_phase_right << "=>" << endl;
#if 0
				///////////////////////////////////////////////////////////////////////////
				//找左图上的相对匹配点
				//在以(i,j)为中心的区域内寻找和(cor_x, cor_y)相位最近的4个点
				std::map<float, Point2i> map_points_left;
				area_process = false;
				if (i > area && i < row - area && j > area && j < col - area)
				{
					area_process = true;
					map_points_left.insert(std::pair<float, Point2i>(abs(cor_phase_right - leftphase.at<float>(i - area, j - area)), Point2i(i - area, j - area)));
					map_points_left.insert(std::pair<float, Point2i>(abs(cor_phase_right - leftphase.at<float>(i - area, j - area + 1)), Point2i(i - area, j - area + 1)));
					map_points_left.insert(std::pair<float, Point2i>(abs(cor_phase_right - leftphase.at<float>(i - area, j - area + 2)), Point2i(i - area, j - area + 2)));
					map_points_left.insert(std::pair<float, Point2i>(abs(cor_phase_right - leftphase.at<float>(i - area, j - area + 3)), Point2i(i - area, j - area + 3)));

					for (int m = i - area; m < i + area; m++)
					{
						for (int n = j - area; n < j + area; n++)
						{
							std::map<float, Point2i>::reverse_iterator rit = map_points_left.rbegin();
							if (abs(cor_phase_right - leftphase.at<float>(m, n)) < rit->first)
							{
								map_points_left.erase(rit->first);
								map_points_left.insert(std::pair<float, Point2i>(abs(left - rightphase.at<float>(m, n)), Point2i(m, n)));
							}
						}
					}
				}

				vector<Point2i> vec_points_left;
				vec_points_left.push_back(Point2i(i, j));
				if (true == area_process)
				{
					std::map<float, Point2i>::iterator it = map_points_left.begin();
					for (auto & x : map_points_left) {
						// 比(i, j)更合理的点添加到容器中
						if (x.first <= abs(cor_phase_right - leftphase.at<float>(i, j)))
							vec_points_left.push_back(x.second);
					}
				}

				int cor_x_left_sum = 0;
				int cor_y_left_sum = 0;
				float phase_left_total = 0.0;
				for (auto point : vec_points_left)
				{
					cor_x_left_sum += point.y;
					cor_y_left_sum += point.x;
					phase_left_total += leftphase.at<float>(point.x, point.y);
				}

				float cor_x_left = (float)cor_x_left_sum / vec_points_left.size();
				float cor_y_left = (float)cor_y_left_sum / vec_points_left.size();
				float cor_phase_left = (float)phase_left_total / vec_points_left.size();
				// float cor_phase_left = (float)leftphase.at<float>( (int)cor_x_left, (int)cor_y_left);

				///////////////////////////////////////////////////////////////////////////

				// cout << abs(cor_phase_left - cor_phase_right) << "\t";
#endif
				if (k < col)
				//if (abs(cor_phase_left - cor_phase_right) < 0.001)
				{
					point_left.x = j;
					point_left.y = i;
					//point_right.x = k;
					//point_right.y = i;
					//point_left.x = cor_x_left;
					//point_left.y = cor_y_left;
					point_right.x = cor_x_right;
					point_right.y = cor_y_right;
					leftkeypoint.push_back(point_left);
					rightkeypoint.push_back(point_right);
				}
			}
		}
	}

	cout << "====> " << leftkeypoint.size() << endl;
}

//保存3D点
void savepnts3D(const char *filename, cv::Mat& mat)
{
	FILE* fp = fopen(filename, "wt");

	float *pnts3D_row1 = mat.ptr<float>(0);
	float *pnts3D_row2 = mat.ptr<float>(1);
	float *pnts3D_row3 = mat.ptr<float>(2);
	float *pnts3D_row4 = mat.ptr<float>(3);

	//int pixelsvel;
	for (int y = 0; y < mat.cols; y++)
	{
		float pnts3D_data4 = *(pnts3D_row4 + y);

		float pnts3D_data1 = *(pnts3D_row1 + y) / pnts3D_data4;
		float pnts3D_data2 = *(pnts3D_row2 + y) / pnts3D_data4;
		float pnts3D_data3 = *(pnts3D_row3 + y) / pnts3D_data4;

		/*
		float pnts3D_data1 = *(pnts3D_row1 + y);
		float pnts3D_data2 = *(pnts3D_row2 + y);
		float pnts3D_data3 = *(pnts3D_row3 + y);
		*/

		fprintf(fp, "%f   %f   %f \n", pnts3D_data1, pnts3D_data2, pnts3D_data3);

		//pixelsvel = (int)(225 * pnts3D_data3 / 1900.00);
	}

	fclose(fp);
}

#endif

