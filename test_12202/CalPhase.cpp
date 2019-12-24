#include "CalPhase.h"
#include "FileManager.h"

using namespace cv;
using namespace std;

#define showThreshold false 

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
    
	return wrapped_phase;
}

void UnwrappedPhaseClassicMethod(Mat& src, Mat& dst)
{
	for(int y = 0; y < src.rows; y++)
    {
		int k1 = 0, k2 = 0;
		float *wrapped_phase_data = src.ptr<float>(y);
		float *unwrapped_phase_data = dst.ptr<float>(y);
      
		float fprephase = *wrapped_phase_data;
		*unwrapped_phase_data = fprephase;
      
		for(int x = 1; x < src.cols; x++)
		{
			wrapped_phase_data++;
			unwrapped_phase_data++;
			if(abs((*wrapped_phase_data)-fprephase) <= CV_PI)
				k2 = k1;
			else if(*wrapped_phase_data - fprephase < -CV_PI)
				k2 = k1 + 1;
			else if(*wrapped_phase_data - fprephase > CV_PI)
				k2 = k1 - 1;
	
			fprephase = *wrapped_phase_data;
			*unwrapped_phase_data = 2*CV_PI*k2 + *wrapped_phase_data;	
			k1 = k2;
		}
	}
}

void UnwrappedPhaseGraycodeMethod(Mat& src, Mat& dst, const std::string &Rect_images)
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
  
	Mat phase_series(Size(img1.cols, img1.rows), CV_8UC1, Scalar(0.0)); 
  
	uchar thresh = 130; // model1:127

	//threshold(img1, img1, thresh, 1, CV_THRESH_BINARY);
	threshold(img1, img1, thresh, 255, CV_THRESH_BINARY);
	threshold(img2, img2, thresh, 255, CV_THRESH_BINARY);
	threshold(img3, img3, thresh, 255, CV_THRESH_BINARY);
	threshold(img4, img4, thresh, 255, CV_THRESH_BINARY);
	threshold(img5, img5, thresh, 255, CV_THRESH_BINARY);
	threshold(img6, img6, thresh, 255, CV_THRESH_BINARY);

	if (showThreshold)
	{
		cout << "show threshold" << endl;
		for (int k = 0; k < 6; k++)
		{
			Mat imgshow = imread(imagelist[k], CV_LOAD_IMAGE_GRAYSCALE);
			threshold(imgshow, imgshow, thresh, 255, CV_THRESH_BINARY);
			double sf = 640. / MAX(imgshow.rows, imgshow.cols);
			resize(imgshow, imgshow, Size(), sf, sf); //调整图像大小640 x 640

			// imshow("imgshow", imgshow);
		}
	}

	bitwise_xor(img1, img2, img2);
	bitwise_xor(img2, img3, img3);
	bitwise_xor(img3, img4, img4);
	bitwise_xor(img4, img5, img5);
	bitwise_xor(img5, img6, img6);

#if 1
	int x,y;
	int width = img1.cols;
	int height = img1.rows;
	uchar pre_series,cur_series;
	float pre_unphase, cur_unphase;
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
			phase_series.at<uchar>(y,x) = ((int)(*img1_ptr++))*32 + ((int)(*img2_ptr++))*16
												+ ((int)(*img3_ptr++))*8 + ((int)(*img4_ptr++))*4 
												+ ((int)(*img5_ptr++))*2 + ((int)(*img6_ptr++))*1; 
		}
	}
  
	medianBlur(phase_series, phase_series, 9); //中值滤波

	for(y=0; y < height; y++)
	{
		for(x=0; x < width; x++)
		{
			//绝对相位 = 2*PI*k + θ(x,y)（相对相位/相位主体）
			dst.at<float>(y,x) = phase_series.at<uchar>(y,x)*2*CV_PI + src.at<float>(y,x);
		}
	}
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
}


