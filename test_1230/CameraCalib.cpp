#include "CameraCalib.h"
#include<cxcore.h>
#include<cv.h>
#include<highgui.h>
#include<stdio.h>

using namespace cv;
using namespace std;

// 相机立体标定

#define CORNER_COLS  7
#define CORNER_ROWS  6
bool displayCorners = true;   //可视化角点

bool readStringList( const string& filename, vector<string>& l)
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

static void StereoCalib(const vector<string>& imagelist, Size boardSize, bool useCalibrated,
			bool showRectified, const string &storintrinsicsyml, const string &storextrinsicsyml)
{
    if( imagelist.size() % 2 != 0 )
    {
        cout << "Error: the image list contains odd (non-even) number of elements\n";
        return;
    }

    const int maxScale = 2;      
    const float squareSize = 20.f;  // Set this to your actual square size
    // ARRAY AND VECTOR STORAGE:

    vector<vector<Point2f> > imagePoints[2];  //角点像素坐标
    vector<vector<Point3f> > objectPoints;    //物理坐标
    Size imageSize;

    int i, j, k, nimages = (int)imagelist.size()/2;

    imagePoints[0].resize(nimages);  //根据nimages确定vector的大小
    imagePoints[1].resize(nimages);  //同上
    vector<string> goodImageList;    
   
    for(i = j = 0; i < nimages; i++)
    {
        for( k = 0; k < 2; k++ )
        {
            const string  filename = imagelist[i*2+k];
            Mat img = imread(filename, 0);
            if(img.empty())
                break;
            if(imageSize == Size())
                imageSize = img.size();
            else if( img.size() != imageSize )  //确定所有图像大小一样
            {
                cout << "The image " << filename << " has the size different from the first image size. Skipping the pair\n";
                break;
            }   
            bool found = false;
            vector<Point2f>& corners = imagePoints[k][j];
            for( int scale = 1; scale <= maxScale; scale++ )
            {
                Mat timg;
                if( scale == 1 )
                    timg = img;
                else
                    resize(img, timg, Size(), scale, scale);
		
                found = findChessboardCorners(timg, boardSize, corners,
                    CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);		
                if( found )
                {
                    if( scale > 1 )
                    {
                        Mat cornersMat(corners);
                        cornersMat *= 1./scale;
                    }
                    break;
                }
            }
            if( displayCorners )
            {
                cout << filename << endl;
                Mat cimg, cimg1;
                cvtColor(img, cimg, COLOR_GRAY2BGR);
                drawChessboardCorners(cimg, boardSize, corners, found);  //显示角点
                double sf = 640./MAX(img.rows, img.cols);
                resize(cimg, cimg1, Size(), sf, sf);   //调整图像大小640 x 640
                imshow("corners", cimg1);
                char c = (char)waitKey(500);
                if( c == 27 || c == 'q' || c == 'Q' ) //Allow ESC to quit
                    exit(-1);
            }
            else
                putchar('.');
            if( !found )  // whether scale = 1 or 2  can not calculate conners.
                break;
            cornerSubPix(img, corners, Size(11,11), Size(-1,-1),
                         TermCriteria(TermCriteria::COUNT+TermCriteria::EPS,
                                      30, 0.01));
        }
        if( k == 2 )  //
        {
            goodImageList.push_back(imagelist[i*2]);  // Add data to the end of the %vector
            goodImageList.push_back(imagelist[i*2+1]);
            j++;
        }
    }

    cout << j << " pairs have been successfully detected.\n";
    nimages = j;
    if( nimages < 2 )
    {
        cout << "Error: too little pairs to run the calibration\n";
        return;
    }

    imagePoints[0].resize(nimages);    //根据nimages确定vector的大小
    imagePoints[1].resize(nimages);    //同上
    objectPoints.resize(nimages);      //同上

    for( i = 0; i < nimages; i++ )
    {
        for( j = 0; j < boardSize.height; j++ )
            for( k = 0; k < boardSize.width; k++ )
                objectPoints[i].push_back(Point3f(j*squareSize, k*squareSize, 0));
    }

    cout << "Running stereo calibration ...\n";

    Mat cameraMatrix[2], distCoeffs[2];

    cameraMatrix[0] = Mat::eye(3, 3, CV_64F);  
    cameraMatrix[1] = Mat::eye(3, 3, CV_64F);
    Mat R, T, E, F; //E：本征矩阵， F：基础矩阵
	//R/T: 第一个相机和第二个相机的相对位置 

	// 立体标定，得到相机的内参【内参矩阵(焦距、主点)和畸变系数矩阵】
    double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
			cameraMatrix[0], distCoeffs[0],
			cameraMatrix[1], distCoeffs[1],
			imageSize, R, T, E, F,
			CALIB_FIX_ASPECT_RATIO +
			CALIB_SAME_FOCAL_LENGTH +
			CALIB_ZERO_TANGENT_DIST +
		    CALIB_RATIONAL_MODEL +
            CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,
            TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 100, 1e-5)
			);
    cout << "done with RMS error=" << rms << endl;

    double err = 0;
    int npoints = 0;
    vector<Vec3f> lines[2];
    for( i = 0; i < nimages; i++ )
    {
        int npt = (int)imagePoints[0][i].size();
        Mat imgpt[2];
        for( k = 0; k < 2; k++ )
        {
            imgpt[k] = Mat(imagePoints[k][i]);
            undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
            computeCorrespondEpilines(imgpt[k], k+1, F, lines[k]); //极线校正
        }
        for( j = 0; j < npt; j++ )
        {
            double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
                                imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
                           fabs(imagePoints[1][i][j].x*lines[0][j][0] +
                                imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
            err += errij;
        }
        npoints += npt;
    }
    cout << "average reprojection err = " <<  err/npoints << endl;

    // save intrinsic parameters
    FileStorage fs(storintrinsicsyml, FileStorage::WRITE);
    if( fs.isOpened() )
    {
        fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
            "M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
        fs.release();
    }
    else
        cout << "Error: can not save the intrinsic parameters\n";

	cout << "\n==> M1:\n" << cameraMatrix[0] << endl;
	cout << "\n==> M2:\n" << cameraMatrix[1] << endl;
	cout << "\n==> D1:\n" << distCoeffs[0] << endl;
	cout << "\n==> D2:\n" << distCoeffs[1] << endl;
	cout << "\n==> R:\n" << R << endl;
	cout << "\n==> T:\n" << T << endl;

	cout << "image_size: " << imageSize << endl;

	// 相机校正
    Mat R1, R2, P1, P2, Q;
    Rect validRoi[2];

    stereoRectify(cameraMatrix[0], distCoeffs[0],
                  cameraMatrix[1], distCoeffs[1],
                  imageSize, R, T, R1, R2, P1, P2, Q,
                  0, -1, imageSize, &validRoi[0], &validRoi[1]);    
                 
    bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));
    
	// COMPUTE AND DISPLAY RECTIFICATION
    if( !showRectified )
        return;

    Mat rmap[2][2];
	// IF BY CALIBRATED (BOUGUET'S METHOD)
    if( useCalibrated )
    {
        // we already computed everything
    }
	// OR ELSE HARTLEY'S METHOD
    else
	// use intrinsic parameters of each camera, but
	// compute the rectification transformation directly
	// from the fundamental matrix
    {
        vector<Point2f> allimgpt[2];
        for( k = 0; k < 2; k++ )
        {
            for( i = 0; i < nimages; i++ )
                std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
        }
        F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
        Mat H1, H2;
        stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize, H1, H2, 3);

        R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
        R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
        P1 = cameraMatrix[0];
        P2 = cameraMatrix[1];
    }
    fs.open(storextrinsicsyml, FileStorage::WRITE);
    if( fs.isOpened() )
    {
        fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
        fs.release();
    }
    else
        cout << "Error: can not save the extrinsic parameters\n";

    //Precompute maps for cv::remap()
    initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
    initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

    Mat canvas;
    double sf;
    int w, h;
    if( !isVerticalStereo )
    {
        sf = 600./MAX(imageSize.width, imageSize.height);  //600
        w = cvRound(imageSize.width*sf);
        h = cvRound(imageSize.height*sf);
        canvas.create(h, w*2, CV_8UC3);
    }
    else
    {
        sf = 300./MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width*sf);
        h = cvRound(imageSize.height*sf);
        canvas.create(h*2, w, CV_8UC3);
    }
/*************************************************************************************/  

/***************************************************************************************/
    for( i = 0; i < nimages; i++ )
    {
        for( k = 0; k < 2; k++ )
        {
            Mat img = imread(goodImageList[i*2+k], 0), rimg, cimg;
	   
            remap(img, rimg, rmap[k][0], rmap[k][1], INTER_LINEAR);
            cvtColor(rimg, cimg, COLOR_GRAY2BGR);
            Mat canvasPart = !isVerticalStereo ? canvas(Rect(w*k, 0, w, h)) : canvas(Rect(0, h*k, w, h));
            resize(cimg, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
            if( useCalibrated )
            {
                Rect vroi(cvRound(validRoi[k].x*sf), cvRound(validRoi[k].y*sf),
                          cvRound(validRoi[k].width*sf), cvRound(validRoi[k].height*sf));
                rectangle(canvasPart, vroi, Scalar(0,0,255), 3, 8);
            }
        }

        if( !isVerticalStereo )
            for( j = 0; j < canvas.rows; j += 16 )
                line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
        else
            for( j = 0; j < canvas.cols; j += 16 )
                line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
        imshow("rectified", canvas);
		// imwrite("./rectified.jpg", canvas);
        char c = (char)waitKey();
        if( c == 27 || c == 'q' || c == 'Q' )
            break;
    }
}

// 立体标定(对外公开接口)
void StereoCalibration(const std::string &imagelistfn, const std::string &storintrinsics, 
		       const std::string &storextrinsics)
{
	Size boardSize = Size(CORNER_COLS, CORNER_ROWS);  // 棋格数 corners 
    bool showRectified = false;  

    vector<string> imagelist;
    bool ok = readStringList(imagelistfn, imagelist);
    if(!ok || imagelist.empty())
    {
        cout << "can not open " << imagelistfn << " or the string list is empty." << endl;
    }

    StereoCalib(imagelist, boardSize, true, showRectified, storintrinsics, storextrinsics);
}

void StereoCalibrationByCorner(const std::string &storintrinsicsyml, const std::string &storextrinsicsyml)
{
    vector<vector<Point2f>> imagePoints[2];  //角点像素坐标
    vector<vector<Point3f>> objectPoints;    //物理坐标
    Size imageSize(1440, 1080);

	int nimages = 13;

    imagePoints[0].resize(nimages);    //根据nimages确定vector的大小
    imagePoints[1].resize(nimages);    //同上
    objectPoints.resize(nimages);      //同上

	const char * object_points_txt = "../input/calib/X3D_Test_2.txt";
	const char * points_left_txt = "../input/calib/Left_Test_2.txt";
	const char * points_right_txt = "../input/calib/Righ_Test_2.txt";

	ifstream infile;
	infile.open(object_points_txt);   //将文件流对象与文件连接起来 
	if (!infile.is_open()) {
		//若失败,则输出错误消息,并终止程序运行 
		cout << "open object points txt error!" << endl;
	}

	string s;
	while (getline(infile, s))
	{
		// cout << s << endl;
		float sTmp;
		vector<float> coors;
		istringstream istr(s);
		while (!istr.eof())
		{
			istr >> sTmp; //get a word
			//cout << sTmp << "\t";
			coors.push_back(sTmp);
		}
		//cout << endl;
		for (int i = 0; i < nimages; i++)
		{
			objectPoints[i].push_back(Point3f(coors[0], coors[1], coors[2]));
		}
	}
	infile.close();

	infile.open(points_left_txt); //将文件流对象与文件连接起来 
	if (!infile.is_open()) {
		cout << "open points left txt error!" << endl;
	}

	int line = 0;
	while (getline(infile, s))
	{
		// cout << s << endl;
		float sTmp;
		vector<float> coors;
		istringstream istr(s);
		int i = 0;
		while (!istr.eof() && i < 2)
		{
			istr >> sTmp; //get a word
			//cout << sTmp << "\t";
			coors.push_back(sTmp);
			i++;
		}
		//cout << endl;

		int index = line / 289;
		line++;

		imagePoints[0][index].push_back(Point2f(coors[0], coors[1]));
	}
	infile.close();

	infile.open(points_right_txt); //将文件流对象与文件连接起来 
	if (!infile.is_open()) {
		cout << "open points left txt error!" << endl;
	}

	line = 0;
	while (getline(infile, s))
	{
		// cout << s << endl;
		float sTmp;
		vector<float> coors;
		istringstream istr(s);
		int i = 0;
		while (!istr.eof() && i < 2)
		{
			istr >> sTmp; //get a word
			//cout << sTmp << "\t";
			coors.push_back(sTmp);
			i++;
		}
		//cout << endl;

		int index = line / 289;
		line++;

		imagePoints[1][index].push_back(Point2f(coors[0], coors[1]));
	}
	infile.close();

	// 无效点去除
	for (int i = 0; i < nimages; i++)
	{
		vector<Point2f>::iterator it1 = imagePoints[0][i].begin();
		vector<Point2f>::iterator it2 = imagePoints[1][i].begin();
		vector<Point3f>::iterator it3 = objectPoints[i].begin();
		while( it1 != imagePoints[0][i].end() && it2 != imagePoints[1][i].end() )
		{
			Point2f pnt1 = *it1;
			Point2f pnt2 = *it2;
			if(pnt1.x < 0 || pnt2.x < 0)
			{
				it1 = imagePoints[0][i].erase(it1);
				it2 = imagePoints[1][i].erase(it2);
				it3 = objectPoints[i].erase(it3);
			}
			else
			{
				it1++;
				it2++;
				it3++;
			}
		}
	}

    Mat cameraMatrix[2], distCoeffs[2];

	cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
	cameraMatrix[1] = Mat::eye(3, 3, CV_64F);
	Mat R, T, E, F;
	//E：本征矩阵， F：基础矩阵
	//R/T: 第一个相机和第二个相机的相对位置 

	// 立体标定，得到相机的内参【内参矩阵(焦距、主点)和畸变系数矩阵】
	double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
		cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		imageSize, R, T, E, F,
		CALIB_FIX_ASPECT_RATIO +
		CALIB_SAME_FOCAL_LENGTH +
		CALIB_ZERO_TANGENT_DIST +
		CALIB_RATIONAL_MODEL +
		CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,
		TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5)
	);
	cout << "done with RMS error=" << rms << endl;

#if 1
	double err = 0;
	int npoints = 0;
	vector<Vec3f> lines[2];
	for (int i = 0; i < nimages; i++)
	{
		int npt = (int)imagePoints[0][i].size();
		Mat imgpt[2];
		for (int k = 0; k < 2; k++)
		{
			imgpt[k] = Mat(imagePoints[k][i]);
			undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
			computeCorrespondEpilines(imgpt[k], k + 1, F, lines[k]); //极线校正
		}
		for (int j = 0; j < npt; j++)
		{
			double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
				imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
				fabs(imagePoints[1][i][j].x*lines[0][j][0] +
					imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
			err += errij;
		}
		npoints += npt;
	}
	cout << "average reprojection err = " << err / npoints << endl;

	// save intrinsic parameters
	FileStorage fs(storintrinsicsyml, FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
			"M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
		fs.release();
	}
	else
		cout << "Error: can not save the intrinsic parameters\n";

	cout << "\n==> M1:\n" << cameraMatrix[0] << endl;
	cout << "\n==> M2:\n" << cameraMatrix[1] << endl;
	cout << "\n==> D1:\n" << distCoeffs[0] << endl;
	cout << "\n==> D2:\n" << distCoeffs[1] << endl;
	cout << "\n==> R:\n" << R << endl;
	cout << "\n==> T:\n" << T << endl;

	cout << "image_size: " << imageSize << endl;

	// 相机校正
	Mat R1, R2, P1, P2, Q;
	Rect validRoi[2];

	stereoRectify(cameraMatrix[0], distCoeffs[0],
					cameraMatrix[1], distCoeffs[1],
					imageSize, R, T, R1, R2, P1, P2, Q,
					0, -1, imageSize, &validRoi[0], &validRoi[1]);

	// 保存相机的外部参数
	fs.open(storextrinsicsyml, FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
		fs.release();
	}
	else
		cout << "Error: can not save the extrinsic parameters\n";

	cout << "save extrinsic params successfully!" << endl;

#endif

#if 0
	bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));

	// COMPUTE AND DISPLAY RECTIFICATION
	if (!showRectified)
		return;

	Mat rmap[2][2];
	// IF BY CALIBRATED (BOUGUET'S METHOD)
	if (useCalibrated)
	{
		// we already computed everything
	}
	// OR ELSE HARTLEY'S METHOD
	else
		// use intrinsic parameters of each camera, but
		// compute the rectification transformation directly
		// from the fundamental matrix
	{
		vector<Point2f> allimgpt[2];
		for (k = 0; k < 2; k++)
		{
			for (i = 0; i < nimages; i++)
				std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
		}
		F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
		Mat H1, H2;
		stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize, H1, H2, 3);

		R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
		R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
		P1 = cameraMatrix[0];
		P2 = cameraMatrix[1];
	}
	fs.open(storextrinsicsyml, FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
		fs.release();
	}
	else
		cout << "Error: can not save the extrinsic parameters\n";

	//Precompute maps for cv::remap()
	initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
	initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

	Mat canvas;
	double sf;
	int w, h;
	if (!isVerticalStereo)
	{
		sf = 600. / MAX(imageSize.width, imageSize.height);  //600
		w = cvRound(imageSize.width*sf);
		h = cvRound(imageSize.height*sf);
		canvas.create(h, w * 2, CV_8UC3);
	}
	else
	{
		sf = 300. / MAX(imageSize.width, imageSize.height);
		w = cvRound(imageSize.width*sf);
		h = cvRound(imageSize.height*sf);
		canvas.create(h * 2, w, CV_8UC3);
	}
	/*************************************************************************************/

	/***************************************************************************************/
	for (i = 0; i < nimages; i++)
	{
		for (k = 0; k < 2; k++)
		{
			Mat img = imread(goodImageList[i * 2 + k], 0), rimg, cimg;

			remap(img, rimg, rmap[k][0], rmap[k][1], INTER_LINEAR);
			cvtColor(rimg, cimg, COLOR_GRAY2BGR);
			Mat canvasPart = !isVerticalStereo ? canvas(Rect(w*k, 0, w, h)) : canvas(Rect(0, h*k, w, h));
			resize(cimg, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
			if (useCalibrated)
			{
				Rect vroi(cvRound(validRoi[k].x*sf), cvRound(validRoi[k].y*sf),
					cvRound(validRoi[k].width*sf), cvRound(validRoi[k].height*sf));
				rectangle(canvasPart, vroi, Scalar(0, 0, 255), 3, 8);
			}
		}

		if (!isVerticalStereo)
			for (j = 0; j < canvas.rows; j += 16)
				line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
		else
			for (j = 0; j < canvas.cols; j += 16)
				line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
		imshow("rectified", canvas);
		// imwrite("./rectified.jpg", canvas);
		char c = (char)waitKey();
		if (c == 27 || c == 'q' || c == 'Q')
			break;
	}
#endif
}

// 立体标定加相机位置校正
void StereoCalibrationByParams(const std::string &storintrinsicsyml, const std::string &storextrinsicsyml)
{
    Mat cameraMatrix[2], distCoeffs[2];
	for (size_t i = 0; i < 2; i++)
	{
		// 存放float型数据
		cameraMatrix[i].convertTo(cameraMatrix[i], CV_64F);
		distCoeffs[i].convertTo(cameraMatrix[i], CV_64F);
	}

	cameraMatrix[0] = (Mat_<double>(3, 3) <<
							5004.084968538499200,   -0.000186077310987, 796.177176385571330,
							 0,					  5004.288845428079200, 645.098858869668220,
							 0,						 0,					  1);

	distCoeffs[0] = (Mat_<double>(1, 14) <<
					-0.094291506605149, 0.454520127343520, 0.000390298085887, -0.000167794573001, 3.533934146177145,
					 0.000000000000000, 0.000000000000000, 0.000000000000000,  0.000000000000000, 0.000000000000000,
					 0.000000000000000, 0.000000000000000, 0.000000000000000, 0.000000000000000);
	/*
	distCoeffs[0] = (Mat_<double>(1, 14) <<
					-0.094291506605149, 0.454520127343520, 0.000390298085887,  0.000000000000000, 0.000000000000000,
					 0.000000000000000, 0.000000000000000, 0.000000000000000, -0.000167794573001, 3.533934146177145,
					 0.000000000000000, 0.000000000000000, 0.000000000000000, 0.000000000000000);
	*/

	cameraMatrix[1] = (Mat_<double>(3, 3) <<
							4991.369386877208900,    0.000227164222608, 786.153820356970750,
							 0,					  4991.811878028854200, 648.483429215111640,
							 0,						 0,					  1);

	distCoeffs[1] = (Mat_<double>(1, 14) <<
					-0.097747727862195, 0.605006458599360, 0.000130603752529, 0.000759994410491, -0.765872396801609,
					 0.000000000000000, 0.000000000000000, 0.000000000000000, 0.000000000000000,  0.000000000000000,
					 0.000000000000000, 0.000000000000000, 0.000000000000000, 0.000000000000000);
	/*
	distCoeffs[1] = (Mat_<double>(1, 14) <<
					-0.097747727862195, 0.605006458599360, 0.000130603752529, 0.000000000000000,  0.000000000000000,
					 0.000000000000000, 0.000000000000000, 0.000000000000000, 0.000759994410491, -0.765872396801609,
					 0.000000000000000, 0.000000000000000, 0.000000000000000, 0.000000000000000);
	*/

	// save intrinsic parameters
	FileStorage fs(storintrinsicsyml, FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
			"M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
		fs.release();
	}
	else
		cout << "Error: can not save the intrinsic parameters\n";

	Mat I_R1, I_T1, I_R2, I_T2;
	I_R1.convertTo(I_R1, CV_64F);
	I_T1.convertTo(I_T1, CV_64F);
	I_R2.convertTo(I_R2, CV_64F);
	I_T2.convertTo(I_T2, CV_64F);

	I_R1 = (Mat_<double>(3, 3) <<
		-0.003449992052740, 0.908392369471684, 0.418104533149851,
		0.992268580264290, 0.054980888811595, -0.111266196509893,
		-0.124061122738460, 0.414488124016969, -0.901558890408035);

	I_T1 = (Mat_<double>(3, 1) <<
		3.688988301573581, -4.927452164451585, 329.276493470459510);

	I_R2 = (Mat_<double>(3, 3) <<
		-0.005778730523496, 0.970132888506089, 0.242505226567117,
		0.992520961272705, 0.035135856240512, -0.116908567010947,
		-0.121937474583672, 0.240015937481406, -0.963080267707255);

	I_T2 = (Mat_<double>(3, 1) <<
		3.780742082249347, -4.998608845649666, 328.926407599367390);

	Mat R2T = Mat(Size(3,3), CV_64F);
	cv::transpose(I_R2, R2T); //转置

	Mat R, T; //两个相机的相对位置

	R = I_R1 * R2T;
	T = I_T2 - R * I_T1;

	/*
	Mat P1 = (Mat_<float>(3, 4) <<
		1.00, 0.00, 0.00, 0.00,
		0.00, 1.00, 0.00, 0.00,
		0.00, 0.00, 1.00, 0.00);

	Mat P2 = (Mat_<float>(3, 4) <<
		R.at<float>(0, 0), R.at<float>(0, 1), R.at<float>(0, 2), T.at<float>(0, 0),
		R.at<float>(1, 0), R.at<float>(1, 1), R.at<float>(1, 2), T.at<float>(1, 0),
		R.at<float>(2, 0), R.at<float>(2, 1), R.at<float>(2, 2), T.at<float>(2, 0));
	*/

    Size imageSize = Size(1600, 1200);

	// 相机校正
	Mat R1, R2, P1, P2, Q; //两个相机相对于矫正平面的变换矩阵，P1、P2为相机的投影矩阵
	Rect validRoi[2];

	cout << "\n==> M1:\n" << cameraMatrix[0] << endl;
	cout << "\n==> M2:\n" << cameraMatrix[1] << endl;
	cout << "\n==> D1:\n" << distCoeffs[0] << endl;
	cout << "\n==> D2:\n" << distCoeffs[1] << endl;
	cout << "\n==> R:\n" << R << endl;
	cout << "\n==> T:\n" << T << endl;

	cout << "image_size: " << imageSize << endl;

	stereoRectify(cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		imageSize, R, T, R1, R2, P1, P2, Q,
		0, -1, imageSize, &validRoi[0], &validRoi[1]);

	cout << "\n==> P1:\n" << P1 << endl;
	cout << "\n==> P2:\n" << P2 << endl;

	cout << "stereo rectify successfully!\n";

	// 0, -1, imageSize, &validRoi[0], &validRoi[1]);
	// fs.open(storextrinsicsyml, FileStorage::WRITE);
	// if( fs.isOpened() )
	// {
	//     fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
	//     fs.release();
	// }
	// else
	//     cout << "Error: can not save the extrinsic parameters\n";

	// OpenCV can handle left-right
	// or up-down camera arrangements
	bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));

	bool showRectified = true;

	// COMPUTE AND DISPLAY RECTIFICATION
	if (!showRectified)
		return;

	bool useCalibrated = true;

	/*
	Mat rmap[2][2];
	// IF BY CALIBRATED (BOUGUET'S METHOD)
	if (useCalibrated)
	{
		// we already computed everything
	}
	// OR ELSE HARTLEY'S METHOD
	else
		// use intrinsic parameters of each camera, but
		// compute the rectification transformation directly
		// from the fundamental matrix
	{
		vector<Point2f> allimgpt[2];
		for (k = 0; k < 2; k++)
		{
			for (i = 0; i < nimages; i++)
				std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
		}
		F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
		Mat H1, H2;
		stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize, H1, H2, 3);

		R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
		R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
		P1 = cameraMatrix[0];
		P2 = cameraMatrix[1];
	}
	*/

	// 保存相机的外部参数
	fs.open(storextrinsicsyml, FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
		fs.release();
	}
	else
		cout << "Error: can not save the extrinsic parameters\n";

	cout << "save extrinsic params successfully!" << endl;

#if 0
	//Precompute maps for cv::remap()
	Mat rmap[2][2];
	initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
	initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

	cout << "init undistort rectify map successfully!" << endl;

	Mat canvas;
	double sf;
	int w, h;
	if (!isVerticalStereo)
	{
		sf = 600. / MAX(imageSize.width, imageSize.height);  //600
		w = cvRound(imageSize.width*sf);
		h = cvRound(imageSize.height*sf);
		canvas.create(h, w * 2, CV_8UC3);
	}
	else
	{
		sf = 300. / MAX(imageSize.width, imageSize.height);
		w = cvRound(imageSize.width*sf);
		h = cvRound(imageSize.height*sf);
		canvas.create(h * 2, w, CV_8UC3);
	}
	/*************************************************************************************/


	/***************************************************************************************/
	for (i = 0; i < nimages; i++)
	{
		for (k = 0; k < 2; k++)
		{
			Mat img = imread(goodImageList[i * 2 + k], 0), rimg, cimg;

			remap(img, rimg, rmap[k][0], rmap[k][1], INTER_LINEAR);
			cvtColor(rimg, cimg, COLOR_GRAY2BGR);
			Mat canvasPart = !isVerticalStereo ? canvas(Rect(w*k, 0, w, h)) : canvas(Rect(0, h*k, w, h));
			resize(cimg, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
			if (useCalibrated)
			{
				Rect vroi(cvRound(validRoi[k].x*sf), cvRound(validRoi[k].y*sf),
					cvRound(validRoi[k].width*sf), cvRound(validRoi[k].height*sf));
				rectangle(canvasPart, vroi, Scalar(0, 0, 255), 3, 8);
			}
		}

		if (!isVerticalStereo)
			for (j = 0; j < canvas.rows; j += 16)
				line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
		else
			for (j = 0; j < canvas.cols; j += 16)
				line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
		imshow("rectified", canvas);
		// imwrite("./rectified.jpg", canvas);
		char c = (char)waitKey();
		if (c == 27 || c == 'q' || c == 'Q')
			break;
	}
#endif
}

void ImgRectified(const std::string& intrinsic_filename, const std::string& extrinsic_filename, 
		  const std::string& imageListfn, const std::string& RectimageListfn)
{
	bool  showRect = false;
    vector<string> imagelist;
    vector<string> Rectimagelist;
    bool ok = readStringList(imageListfn, imagelist);
    if(!ok || imagelist.empty())
    {
		cout << "can not open " << imageListfn << " or the string list is empty" << endl;
    }   
    if( imagelist.size() % 2 != 0 )
    {
		cout << "Error: the image list contains odd (non-even) number of elements\n";
		return;
    }
    
    ok = readStringList(RectimageListfn, Rectimagelist);
    if(!ok || Rectimagelist.empty())
    {
		cout << "can not open " << RectimageListfn << " or the string list is empty" << endl;
    }   
    if( Rectimagelist.size() % 2 != 0 )
    {
		cout << "Error: the image list contains odd (non-even) number of elements\n";
		return;
    }
    
    int i, k, nimages = imagelist.size()/2;   	
    string  filename = imagelist[0];
    Mat img = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
    
    Size img_size = img.size();
    Rect roi1, roi2;
    Mat Q;
    
    // reading intrinsic parameters
    FileStorage fs(intrinsic_filename, FileStorage::READ);
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

    fs.open(extrinsic_filename, FileStorage::READ);
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
	//  stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );
     
	Mat map11, map12, map21, map22;
	initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
	initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

	for(i=0; i < nimages; i++)
	{
		filename = imagelist[2*i];
		Mat img1 = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
		filename = imagelist[2*i+1];
		Mat img2 = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
	  
		Mat img1r, img2r;
		remap(img1, img1r, map11, map12, INTER_LINEAR);
		remap(img2, img2r, map21, map22, INTER_LINEAR);
	
		filename  = Rectimagelist[2*i];
		imwrite(filename,img1r);
		filename  = Rectimagelist[2*i+1];
		imwrite(filename,img2r);
		if(showRect)
		{
			Mat canvas;
			double sf;
			int w, h;
			sf = 600./MAX(img_size.width, img_size.height);  //600
			w = cvRound(img_size.width*sf);
			h = cvRound(img_size.height*sf);
			canvas.create(h, w*2, CV_8UC3);
	  
			cvtColor(img1r, img1, COLOR_GRAY2BGR);
			cvtColor(img2r, img2, COLOR_GRAY2BGR);
	  
			Mat canvasPart = canvas(Rect(w*0, 0, w, h));
			resize(img1, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
			canvasPart = canvas(Rect(w*1, 0, w, h));
			resize(img2, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
	  
			for(int j = 0; j < canvas.rows; j += 16 )
				line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
	  
			imshow("rectified", canvas);
			char c = (char)waitKey();
			if( c == 27 || c == 'q' || c == 'Q' )
				break;
		}
	}
}

#if 0
Mat householder(Mat& x, Mat& y);
Mat blkdiag(const Mat& Ht, const CvSize& rect);
Mat blkdiag(const Mat& Ht, const CvSize& rect);

int ypQR_householder(const Mat& A, Mat& Q, Mat& R)
{
	int width = A.cols;
	int height = A.rows;
	R = A.clone();
	Q = Mat(height, height, A.type); // CV_64FC1
	setIdentity(Q); // 单位矩阵

	int end = 0;
	if (width > height)
		end = height;
	else
		end = width;

	for (int i = 0; i < end; i++)
	{
		Mat x = cvCreateMatHeader(height - i, 1, CV_32F); //此处一定要注意
		cvGetSubRect(R, x, cvRect(i, i, 1, height - i));
		Mat y = Mat::zeros(height - i, 1, CV_32F);
		// cvZero(y);
		// cvmSet(y, 0, 0, 1);
		y.at<float>(0, 0);
		Mat Ht = householder(x, y);
		if (NULL == Ht) return -1;
		Mat H = blkdiag(Ht, cvSize(Q->cols, Q->rows));
		if (NULL == H) return -1;
		cvmMul(Q, H, Q);
		cvmMul(H, R, R);
	}
	return 0;
}
Mat householder(Mat& x, Mat& y)
{
	int sign = 1;
	if (cvmGet(x, 0, 0) < 0)
		sign = -1;
	float nx = cvNorm(x);
	float ny = cvNorm(y);
	float rho = -1 * sign * nx / ny;
	cvConvertScale(y, y, rho);
	Mat diff_xy = cvCreateMat(x->rows, x->cols, x->type);
	cvSub(x, y, diff_xy);
	float n_diff_xy = cvNorm(diff_xy);
	cvConvertScale(diff_xy, diff_xy, 1 / n_diff_xy);
	Mat I = cvCreateMat(x->rows, x->rows, x->type);
	cvSetIdentity(I);
	Mat diff_xy_trans = cvCreateMat(diff_xy->cols, diff_xy->rows, diff_xy->type);
	cvTranspose(diff_xy, diff_xy_trans);
	Mat diff_xy_mul = cvCreateMat(diff_xy->rows, diff_xy_trans->cols, diff_xy->type);
	cvmMul(diff_xy, diff_xy_trans, diff_xy_mul);
	cvConvertScale(diff_xy_mul, diff_xy_mul, -2);
	cvAdd(I, diff_xy_mul, diff_xy_mul);
	return diff_xy_mul;
}
Mat blkdiag(const Mat& Ht, const CvSize& rect)
{
	int H_width = rect.width;
	int H_height = rect.height;
	int Ht_width = Ht->cols;
	int Ht_height = Ht->rows;
	if (H_width < Ht_width || H_height < Ht_height)
		return NULL;
	else if (H_width == Ht_width && H_height == Ht_height)
		return Ht;
	else
	{
		Mat H = cvCreateMat(rect.height, rect.width, CV_32F);
		cvSetIdentity(H);
		int start_i = H_height - Ht_height;
		int start_j = H_width - Ht_width;
		for (int i = start_i; i < H_height; i++)
		{
			for (int j = start_j; j < H_width; j++)
			{
				cvmSet(H, i, j, cvmGet(Ht, i - start_i, j - start_j));
			}
		}
		return H;
	}
}
/***********************************test*****************************************/
void test_qr()
{
	float a[] = { -5,7, 6,8,-20,26,5,-5,34,16,9,65,10,13,15 };
	Mat A = cvCreateMatHeader(3, 5, CV_32FC1);
	cvInitMatHeader(A, 3, 5, CV_32FC1, a);
	Mat Q = NULL;
	Mat R = NULL;
	int re = ypQR_householder(A, Q, R);
	if (-1 == re || NULL == Q || NULL == R) return;
	Mat Aq = cvCreateMat(3, 5, CV_32FC1);
	cvmMul(Q, R, Aq);
	printf("A:\n");
	for (int i = 0; i < A->rows; i++)
	{
		for (int j = 0; j < A->cols; j++)
			printf("%f ", cvGetReal2D(A, i, j));
		printf("\n");
	}
	printf("Q:\n");
	for (int i = 0; i < Q->rows; i++)
	{
		for (int j = 0; j < Q->cols; j++)
			printf("%f ", cvGetReal2D(Q, i, j));
		printf("\n");
	}
	printf("R:\n");
	for (int i = 0; i < R->rows; i++)
	{
		for (int j = 0; j < R->cols; j++)
			printf("%f ", cvGetReal2D(R, i, j));
		printf("\n");
	}
	printf("At:\n");
	for (int i = 0; i < Aq->rows; i++)
	{
		printf("\n");
		for (int j = 0; j < Aq->cols; j++)
			printf("%f ", cvGetReal2D(Aq, i, j));
	}
	printf("\n");
	return;
}
int main() {
	test_qr();
}

// 从相机的映射矩阵通过QR分解得到相机的内参矩阵K、旋转矩阵R和平移矩阵T
void KRT_From_P_QR(const Mat & P, Mat & K, Mat & R, Mat & T)
{
	int i, j;

	//第一步
	// P = [H h]
	Mat H = Mat(Size(3, 3), CV_64FC1);
	Mat h = Mat(Size(3, 1), CV_64FC1);

	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			H.at<double>(i, j) = P.at<double>(i, j);
		}
		h.at<double>(i, 0) = P.at<double>(i, 3);
	}

	cout << "\n==> H:\n" << H << endl;
	cout << "\n==> h:\n" << h << endl;

	//第二步
	Mat Hinv = Mat(3, 3, CV_64FC1);
	invert(H, Hinv, CV_SVD); //转置，SVD主值
	int rn = H.rows;
	int cn = H.cols;
	Mat Qd = Mat(Size(rn, cn), CV_64FC1);
	Mat Rd = Mat(Size(rn, rn), CV_64FC1);

	qrHouse(Hinv, Qd, Rd);
	invert(Rd, K, CV_SVD); //
	transpose(Qd, R); //转置

	//第三步：
	double s1[9] = { -1,0,0,0,1,0,0,0,-1 };
	double s2[9] = { -1,0,0,0,-1,0,0,0,1 };
	double s3[9] = { 1,0,0,0,-1,0,0,0,-1 };
	double s4[9] = { 1,0,0,0,1,0,0,0,1 };

	Mat Rn = Mat(3, 3, CV_64FC1);
	Mat Kn = Mat(3, 3, CV_64FC1);

	Mat S1 = Mat(3, 3, CV_64FC1);
	Mat S2 = Mat(3, 3, CV_64FC1);
	Mat S3 = Mat(3, 3, CV_64FC1);
	Mat S4 = Mat(3, 3, CV_64FC1);
	//cvInitMatHeader(S1, 3, 3, CV_64FC1, s1);
	//cvInitMatHeader(S2, 3, 3, CV_64FC1, s2);
	//cvInitMatHeader(S3, 3, 3, CV_64FC1, s3);
	//cvInitMatHeader(S4, 3, 3, CV_64FC1, s4);
	S1 = (Mat_<double>(3, 3) << -1, 0, 0, 0, 1, 0, 0, 0, -1);
	S2 = (Mat_<double>(3, 3) << -1, 0, 0, 0, -1, 0, 0, 0, 1);
	S3 = (Mat_<double>(3, 3) << 1, 0, 0, 0, -1, 0, 0, 0, -1);
	S4 = (Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);

	double a = K.at<double>(0, 0);
	double b = K.at<double>(1, 1); // cvmGet(K, 1, 1);
	double c = K.at<double>(2, 2); // cvmGet(K, 2, 2);

	Mat t0 = Mat(3, 3, CV_64FC1);

	if ((a > 0 && b < 0 && c>0) || (a < 0 && b>0 && c < 0))
	{
		// cvMatMul(K, S1, Kn);
		// cvMatMul(S1, R, Rn);
		// cvMatMul(S1, Rd, t0);
		Kn = K * S1;
		Rn = S1 * R;
		t0 = S1 * Rd;
	}
	else if ((a > 0 && b > 0 && c < 0) || (a < 0 && b < 0 && c>0))
	{
		//cvMatMul(K, S2, Kn);
		//cvMatMul(S2, R, Rn);
		//cvMatMul(S2, Rd, t0);
		Kn = K * S2;
		Rn = S2 * R;
		t0 = S2 * Rd;
	}
	else if ((a > 0 && b < 0 && c < 0) || (a < 0 && b>0 && c > 0))
	{
		//cvMatMul(K, S3, Kn);
		//cvMatMul(S3, R, Rn);
		//cvMatMul(S2, Rd, t0);
		Kn = K * S3;
		Rn = S3 * R;
		t0 = S2 * Rd;
	}
	else if ((a > 0 && b > 0 && c > 0) || (a < 0 && b < 0 && c < 0))
	{
		//cvMatMul(K, S4, Kn);
		//cvMatMul(S4, R, Rn);
		//cvMatMul(S2, Rd, t0);
		Kn = K * S4;
		Rn = S4 * R;
		t0 = S2 * Rd;
	}

	T = t0 * h;

	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			K.at<double>(i, j) = Kn.at<double>(i, j) / Kn.at<double>(2, 2);
		}
	}

	//cvCopy(Rn, R);

	R = Rn.clone();
}
#endif

/*
void main()
{
	//用Matlab仿真的真实图像实验的投影矩阵
	double P0[12] = {
	0.00191854517446,-0.00174998332538,0.00009670477631,-0.63861687292509,
	0.00015184786215,0.00005945815592,0.00252273614700,-0.76951593410321,
	   -0.00000042931672,-0.00000093480944,0.00000027152848,-0.00075721579556
	};

	CvMat* P = cvCreateMat(3, 4, CV_64FC1);
	cvInitMatHeader(P, 3, 4, CV_64FC1, P0);
	CvMat* K = cvCreateMat(3, 3, CV_64FC1);
	CvMat* R = cvCreateMat(3, 3, CV_64FC1);
	CvMat* T = cvCreateMat(3, 1, CV_64FC1);

	KRT_From_P_QR(P, K, R, T);

	//输出求解出的内外参数矩阵
	int i, j;
	cout << "K:" << endl;
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			cout << cvmGet(K, i, j) << " ";
		}
		cout << endl;
	}
	cout << endl;
	cout << "R:" << endl;
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			cout << cvmGet(R, i, j) << " ";
		}
		cout << endl;
	}
	cout << endl;
	cout << "T:" << endl;
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 1; j++)
		{
			cout << cvmGet(T, i, j) << " ";
		}
		cout << endl;
	}
}
*/

/***************************************************************************************************/
//接下来把一般实矩阵的QR分解按函数的形式稍稍改写一下，输入是一般mxn实矩阵A，以及矩阵的行数m列数n，输出是QR形式的正交矩阵和上三角矩阵的乘积，
/*
void Maqr(double ** A, int m, int n)//进行一般实矩阵QR分解的函数
{
	int i, j, k, nn, jj;
	double u, alpha, w, t;
	double** Q = new double*[m];   //动态分配内存空间
	for (i = 0; i < m; i++) Q[i] = new double[m];
	if (m < n)
	{
		cout << "\nQR分解失败！" << endl;
		exit(1);
	}
	//保证列数<=行数
	for (i = 0; i <= m - 1; i++)
		for (j = 0; j <= m - 1; j++)
		{
			Q[i][j] = 0.0;
			if (i == j) Q[i][j] = 1.0;
		}
	//初始的Q矩阵就是一个单位的m阶方阵
	nn = n;
	if (m == n) nn = m - 1;
	for (k = 0; k <= nn - 1; k++)//在大循环k：0~m当中，进行H矩阵的求解，左乘Q，以及左乘A
	{

		u = 0.0;
		for (i = k; i <= m - 1; i++)
		{
			w = fabs(A[i][k]);
			if (w > u) u = w;
		}
		alpha = 0.0;
		for (i = k; i <= m - 1; i++)
		{
			t = A[i][k] / u; alpha = alpha + t * t;
		}
		if (A[k][k] > 0.0) u = -u;
		alpha = u * sqrt(alpha);
		if (fabs(alpha) + 1.0 == 1.0)
		{
			cout << "\nQR分解失败！" << endl;
			exit(1);
		}

		u = sqrt(2.0*alpha*(alpha - A[k][k]));
		if ((u + 1.0) != 1.0)
		{
			A[k][k] = (A[k][k] - alpha) / u;
			for (i = k + 1; i <= m - 1; i++) A[i][k] = A[i][k] / u;

			//以上就是H矩阵的求得，实际上程序并没有设置任何数据结构来存储H矩
			//阵，而是直接将u向量的元素赋值给原A矩阵的原列向量相应的位置，这样做
			//这样做是为了计算左乘矩阵Q和A
			for (j = 0; j <= m - 1; j++)
			{
				t = 0.0;
				for (jj = k; jj <= m - 1; jj++)
					t = t + A[jj][k] * Q[jj][j];
				for (i = k; i <= m - 1; i++)
					Q[i][j] = Q[i][j] - 2.0*t*A[i][k];
			}
			//左乘矩阵Q，循环结束后得到一个矩阵，再将这个矩阵转置一下就得到QR分解中的Q矩阵
			//也就是正交矩阵

			for (j = k + 1; j <= n - 1; j++)
			{
				t = 0.0;
				for (jj = k; jj <= m - 1; jj++)
					t = t + A[jj][k] * A[jj][j];
				for (i = k; i <= m - 1; i++)
					A[i][j] = A[i][j] - 2.0*t*A[i][k];
			}
			//H矩阵左乘A矩阵，循环完成之后，其上三角部分的数据就是上三角矩阵R
			A[k][k] = alpha;
			for (i = k + 1; i <= m - 1; i++)  A[i][k] = 0.0;
		}
	}
	for (i = 0; i <= m - 2; i++)
		for (j = i + 1; j <= m - 1; j++)
		{
			t = Q[i][j]; Q[i][j] = Q[j][i]; Q[j][i] = t;
		}
	//QR分解完毕，然后在函数体里面直接将Q、R矩阵打印出来
	for (i = 0; i < m; i++)
	{
		for (j = 0; j < n; j++)
		{
			cout << "    " << Q[i][j];
		}
		cout << endl;
	}
	cout << endl;
	for (i = 0; i < m; i++)
	{
		for (j = 0; j < n; j++)
		{
			cout << "    " << A[i][j];
		}
		cout << endl;
	}
}
*/

/*
int main()//主函数的调用
{
	int m, n;
	cout << "输入矩阵的行数m，列数n" << endl;
	cin >> m >> n;
	double** A = new double*[m];
	for (int i = 0; i < m; i++)A[i] = new double[n];
	cout << "输入矩阵A的每一个元素" << endl;
	for (int i = 0; i < m; i++)
		for (int j = 0; j < n; j++)
			cin >> A[i][j];
	Maqr(A, m, n);

	system("pause");
	return 0;
}
*/


