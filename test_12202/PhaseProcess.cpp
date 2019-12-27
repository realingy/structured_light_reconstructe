#include "PhaseProcess.h"
#include "FileManager.h"

using namespace cv;
using namespace std;

static bool readString( const string& filename, vector<string>& l)
{
    l.resize(0);
    FileStorage fs(filename, FileStorage::READ);
    if( !fs.isOpened() )
        return false;
    FileNode n = fs.getFirstTopLevelNode();
    if( n.type() != FileNode::SEQ )
        return false;
    FileNodeIterator it = n.begin(), it_end = n.end();
    for( ; it != it_end; ++it )
        l.push_back((string)*it);
    return true;
}

Mat CalWrappedPhase(const std::string &Rect_images)
{
    vector<string> imagelist;
    bool ok = readString(Rect_images, imagelist);
    if(!ok || imagelist.empty())
    {
        cout << "can not open " << Rect_images << " or the string list is empty" << endl;
    }

	// 6张格雷码调制图像+4张相移调制图像
    if(imagelist.size() != 10)
    {
		cout << "the number of images less ten" <<endl;
    }
    
    Mat img1 = imread(imagelist[6], CV_LOAD_IMAGE_GRAYSCALE);
    Mat img2 = imread(imagelist[7], CV_LOAD_IMAGE_GRAYSCALE);
    Mat img3 = imread(imagelist[8], CV_LOAD_IMAGE_GRAYSCALE);
    Mat img4 = imread(imagelist[9], CV_LOAD_IMAGE_GRAYSCALE);
    
    int height = img1.rows;  //行数
    int width = img1.cols;   //列数
    Mat wrapped_phase(Size(width, height), CV_32FC1, Scalar(0.0));
    
	/*
	for(int i=0; i < height; i++)
    {
		uchar *img1_data = img1.ptr<uchar>(i);
		uchar *img2_data = img2.ptr<uchar>(i);
		uchar *img3_data = img3.ptr<uchar>(i);
		uchar *img4_data = img4.ptr<uchar>(i);
		float *wrapped_phase_data = wrapped_phase.ptr<float>(i);
    
		for(int j=0; j < width; j++)
		{
			*wrapped_phase_data++ = atan2(((float)(*img4_data++) - (float)(*img2_data++)),
											((float)(*img1_data++) - (float)(*img3_data++)));
		}
	}
	*/

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			float I1 = (float)img2.at<uchar>(i, j);
			float I2 = (float)img3.at<uchar>(i, j);
			float I3 = (float)img4.at<uchar>(i, j);
			float I4 = (float)img1.at<uchar>(i, j);

			if (I4 == I2 && I1 > I3)
			{
				wrapped_phase.at<float>(i, j) = 0;
			}
			else if (I4 == I2 && I1 < I3)
			{
				wrapped_phase.at<float>(i, j) = CV_PI;
			}
			else if (I4 > I2&& I1 == I3)
			{
				wrapped_phase.at<float>(i, j) = CV_PI / 2;
			}
			else if (I4 < I2 && I1 == I3)
			{
				wrapped_phase.at<float>(i, j) = 3 * CV_PI / 2;
			}
			else if (I1 < I3) //第二、三象限
			{
				wrapped_phase.at<float>(i, j) = atan((I4 - I2) / (I1 - I3)) + CV_PI;
			}
			else if (I1 > I3&& I4 > I2) //第一象限
			{
				wrapped_phase.at<float>(i, j) = atan((I4 - I2) / (I1 - I3));
			}
			else if (I1 > I3&& I4 < I2) //第四象限
			{
				wrapped_phase.at<float>(i, j) = atan((I4 - I2) / (I1 - I3)) + 2 * CV_PI;
			}
		}
	}

	return wrapped_phase;
}

void UnwrappedPhaseGraycode(Mat& src, Mat& dst, const std::string &Rect_images)
{
	const char * phase_series_txt = "../result/phase_series.txt";
  
	vector<string> imagelist;
	bool ok = readString(Rect_images, imagelist);
	if(!ok || imagelist.empty())
	{
		cout << "can not open " << Rect_images << " or the string list is empty" << endl;
	}
  
	// 6张格雷码调制图像和4张相移调制图像
	if(imagelist.size() != 10)
	{
		cout << "the number of images less ten" <<endl;
	}
  
	// 读取6张格雷码调制图像
	Mat img1 = imread(imagelist[0], CV_LOAD_IMAGE_GRAYSCALE);
	Mat img2 = imread(imagelist[1], CV_LOAD_IMAGE_GRAYSCALE);
	Mat img3 = imread(imagelist[2], CV_LOAD_IMAGE_GRAYSCALE);
	Mat img4 = imread(imagelist[3], CV_LOAD_IMAGE_GRAYSCALE);
	Mat img5 = imread(imagelist[4], CV_LOAD_IMAGE_GRAYSCALE);
	Mat img6 = imread(imagelist[5], CV_LOAD_IMAGE_GRAYSCALE);

	/*
	Mat img_1, img_2, img_3, img_4, img_5, img_6;

	uchar thresh2 = 80;

	threshold(img1, img_1, thresh2, 255, CV_THRESH_BINARY);
	threshold(img2, img_2, thresh2, 255, CV_THRESH_BINARY);
	threshold(img3, img_3, thresh2, 255, CV_THRESH_BINARY);
	threshold(img4, img_4, thresh2, 255, CV_THRESH_BINARY);
	threshold(img5, img_5, thresh2, 255, CV_THRESH_BINARY);
	threshold(img6, img_6, thresh2, 255, CV_THRESH_BINARY);

	bitwise_xor(img_1, img_2, img_2);
	bitwise_xor(img_2, img_3, img_3);
	bitwise_xor(img_3, img_4, img_4);
	bitwise_xor(img_4, img_5, img_5);
	bitwise_xor(img_5, img_6, img_6);

	imwrite("../result/bin_1.bmp", img_1);
	imwrite("../result/bin_2.bmp", img_2);
	imwrite("../result/bin_3.bmp", img_3);
	imwrite("../result/bin_4.bmp", img_4);
	imwrite("../result/bin_5.bmp", img_5);
	imwrite("../result/bin_6.bmp", img_6);
	*/
  
	uchar thresh = 80;

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

	Mat phase_series(Size(img1.cols, img1.rows), CV_8UC1, Scalar(0.0)); 

	int x,y;
	int width = img1.cols;
	int height = img1.rows;
	// uchar pre_series,cur_series;
	// float pre_unphase, cur_unphase;
	for(y=0; y < height; y++)
	{
		uchar *img1_ptr = img1.ptr<uchar>(y);
		uchar *img2_ptr = img2.ptr<uchar>(y);
		uchar *img3_ptr = img3.ptr<uchar>(y);
		uchar *img4_ptr = img4.ptr<uchar>(y);
		uchar *img5_ptr = img5.ptr<uchar>(y);
		uchar *img6_ptr = img6.ptr<uchar>(y);
    
		for (x = 0; x < width; x++)
		{
			phase_series.at<uchar>(y, x) = img1.at<uchar>(y, x) * 32
											+ img2.at<uchar>(y, x) * 16
											+ img3.at<uchar>(y, x) * 8
											+ img4.at<uchar>(y, x) * 4
											+ img5.at<uchar>(y, x) * 2
											+ img6.at<uchar>(y, x) * 1;
			/*
			phase_series.at<uchar>(y,x) = ((int)(*img1_ptr++))*32 + ((int)(*img2_ptr++))*16
												+ ((int)(*img3_ptr++))*8 + ((int)(*img4_ptr++))*4
												+ ((int)(*img5_ptr++))*2 + ((int)(*img6_ptr++))*1; 
			*/
		}
	}
  
	//medianBlur(phase_series, phase_series, 9); //中值滤波

#if 0
	int c1 = 0;
	int c2 = 0;
	int c3 = 0;
	// 通过调整相位级次进行周期错位调整 
	for (int i = 0; i < height; i++)
	{
		for (int j = 2; j < width; j++)
		{
			if ( src.at<float>(i,j) > CV_PI && (abs(src.at<float>(i, j) - src.at<float>(i, j - 1)) >= CV_PI) && (phase_series.at<uchar>(i, j) == phase_series.at<uchar>(i, j - 1)) )
			{
				//相位突变但是级次不变
				c1++;
				phase_series.at<uchar>(i, j) = phase_series.at<uchar>(i, j - 1) + 1;
			}
			else if ( src.at<float>(i, j) > CV_PI && (abs(src.at<float>(i, j) - src.at<float>(i, j - 1)) < CV_PI) && (phase_series.at<uchar>(i, j) == phase_series.at<uchar>(i, j - 1) + 1 ) )
			{
				//相位不突变但是级次有变化
				c2++;
				phase_series.at<uchar>(i, j) = phase_series.at<uchar>(i, j - 1);
			}

		}
	}
	//cout << "c1=======> " << c1 << ", c2=======> " << c2 << endl;
	//cout << "c1=======> " << c1 << ", c2=======> " << c2 << ", c3=======>" << c3 << endl;
#endif

	if(phase_series_txt)
	{
		FILE* fp = fopen(phase_series_txt, "wt");
    
		for(int y = 0; y < phase_series.rows; y++)
		{
			uchar *pixel_phase_data = phase_series.ptr<uchar>(y);
	
			for(int x = 0; x < phase_series.cols; x++)
			{
				uchar point = *pixel_phase_data++;
				fprintf(fp, "%d \t", point);
			}
			fprintf(fp, "\n");
		}
		fclose(fp);
	}

	// 保存解包裹相位（绝对相位）
	for (y = 0; y < height; y++)
	{
		for (x = 0; x < width; x++)
		{
			//绝对相位 = 2*PI*k + θ(x,y)（相对相位/相位主体）
			dst.at<float>(y, x) = phase_series.at<uchar>(y, x) * 2 * CV_PI + src.at<float>(y, x);
		}
	}

	int c1 = 0;
	int c2 = 0;
	// 通过调整相位级次进行周期错位调整 
	for (int i = 0; i < height; i++)
	{
		for (int j = 2; j < width; j++)
		{
			//if (src.at<float>(i, j) > CV_PI && (abs(src.at<float>(i, j) - src.at<float>(i, j - 1)) >= CV_PI) && (phase_series.at<uchar>(i, j) == phase_series.at<uchar>(i, j - 1)))
			if ( (dst.at<float>(i, j) - dst.at<float>(i, j - 1) >= CV_PI) && (phase_series.at<uchar>(i, j) == phase_series.at<uchar>(i, j - 1)))
			{
				//相位突变但是级次不变
				c1++;
				phase_series.at<uchar>(i, j) = phase_series.at<uchar>(i, j - 1) + 1;
			}
			//else if (src.at<float>(i, j) > CV_PI && (abs(src.at<float>(i, j) - src.at<float>(i, j - 1)) < CV_PI) && (phase_series.at<uchar>(i, j) == phase_series.at<uchar>(i, j - 1) + 1))
			else if ( (dst.at<float>(i, j) - dst.at<float>(i, j - 1) < CV_PI) && (phase_series.at<uchar>(i, j) == phase_series.at<uchar>(i, j - 1) + 1))
			{
				//相位不突变但是级次有变化
				c2++;
				phase_series.at<uchar>(i, j) = phase_series.at<uchar>(i, j - 1);
			}

		}
	}
	cout << "c1=======> " << c1 << ", c2=======> " << c2 << endl;

	// 重新计算绝对相位
	// 保存解包裹相位（绝对相位）
	for (y = 0; y < height; y++)
	{
		for (x = 0; x < width; x++)
		{
			//绝对相位 = 2*PI*k + θ(x,y)（相对相位/相位主体）
			dst.at<float>(y, x) = phase_series.at<uchar>(y, x) * 2 * CV_PI + src.at<float>(y, x);
		}
	}

	int count = 0;
	for (int i = 0; i < height; i++)
	{
		for (int j = 1; j < width-1; j++)
		{
			float dist = dst.at<float>(i, j) - dst.at<float>(i, j - 1);
			if (dist < 0)
			{
				count++;
			}
		}
	}

	//周期错位像素数
	cout << "period dislocation pixels count: " << count << endl;

}


