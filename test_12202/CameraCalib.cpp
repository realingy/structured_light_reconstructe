#include "CameraCalib.h"

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
    const float squareSize = 20.f;  // Set this to your actual square size  正方形大小
    // ARRAY AND VECTOR STORAGE:

    vector<vector<Point2f> > imagePoints[2];  //角点像素坐标
    vector<vector<Point3f> > objectPoints;    //物理坐标
    Size imageSize;

    int i, j, k, nimages = (int)imagelist.size()/2;  //单侧图像的数量

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
//*************************************************//
//     FileStorage  fs1;
//     fs1.open("./intrinsics_Left.xml", FileStorage::READ);
//     
//     Mat intrinsic_matrix_loaded, distortion_coeffs_loaded;
//     fs1["camera_matrix"] >> intrinsic_matrix_loaded;
//     fs1["distortion_cofficients"] >> distortion_coeffs_loaded;
//     fs1.release();
//     
//     cameraMatrix[0] = intrinsic_matrix_loaded;
//     distCoeffs[0] = distortion_coeffs_loaded;
//    
//     fs1.open("./intrinsics_Right.xml", FileStorage::READ);
//     fs1["camera_matrix"] >> intrinsic_matrix_loaded;
//     fs1["distortion_cofficients"] >> distortion_coeffs_loaded;
//     fs1.release(); 
//    
//     cameraMatrix[1] = intrinsic_matrix_loaded; 
//     distCoeffs[1] = distortion_coeffs_loaded;
    
//******************************************************//  
    cameraMatrix[0] = Mat::eye(3, 3, CV_64F);  
    cameraMatrix[1] = Mat::eye(3, 3, CV_64F);
    Mat R, T, E, F;
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

//                     CALIB_FIX_ASPECT_RATIO +
//                     CALIB_ZERO_TANGENT_DIST +
//                     CALIB_SAME_FOCAL_LENGTH +
//                     CALIB_RATIONAL_MODEL +
//                     CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,       
    
// CALIBRATION QUALITY CHECK
// because the output fundamental matrix implicitly 
// includes all the output information,
// we can check the quality of calibration using the
// epipolar geometry constraint: m2^t*F*m1=0  极线约束检验
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
            computeCorrespondEpilines(imgpt[k], k+1, F, lines[k]);
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
                 
//	   0, -1, imageSize, &validRoi[0], &validRoi[1]);
//     fs.open(storextrinsicsyml, FileStorage::WRITE);
//     if( fs.isOpened() )
//     {
//         fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
//         fs.release();
//     }
//     else
//         cout << "Error: can not save the extrinsic parameters\n";

// OpenCV can handle left-right
// or up-down camera arrangements
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

// 立体标定加相机位置校正
void StereoCalibration2(const std::string &storintrinsicsyml, const std::string &storextrinsicsyml)
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
				 0.000000000000000, 0.000000000000000, 0.000000000000000, 0.000000000000000, 0.000000000000000,
				 0.000000000000000, 0.000000000000000, 0.000000000000000, 0.000000000000000);

	cameraMatrix[1] = (Mat_<double>(3, 3) <<
							4991.369386877208900,    0.000227164222608, 786.153820356970750,
							 0,					  4991.811878028854200, 648.483429215111640,
							 0,						 0,					  1);

	distCoeffs[1] = (Mat_<double>(1, 14) <<
				-0.097747727862195, 0.605006458599360, 0.000130603752529, 0.000759994410491, -0.765872396801609,
				 0.000000000000000, 0.000000000000000, 0.000000000000000, 0.000000000000000,  0.000000000000000,
				 0.000000000000000, 0.000000000000000, 0.000000000000000, 0.000000000000000);

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
	Mat R1, R2, P1, P2, Q; //两个相机相对于矫正平面的变换矩阵
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

	// while (1);

	//	   0, -1, imageSize, &validRoi[0], &validRoi[1]);
	//     fs.open(storextrinsicsyml, FileStorage::WRITE);
	//     if( fs.isOpened() )
	//     {
	//         fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
	//         fs.release();
	//     }
	//     else
	//         cout << "Error: can not save the extrinsic parameters\n";

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
