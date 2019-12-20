#include "StereoReconstruct.h"

#define PHASE_THRESHOLD  0.01
#define BLOCK_THRESHOLD  0.3
const string storintrinsicsyml  = "../data/output/intrinsics.yml";
const string storextrinsicsyml  = "../data/output/extrinsics.yml";

// 特征点计算
// leftphase/rightphase: CV_32F, 1-channel image
void find_featurepionts(Mat& leftphase, Mat& rightphase, vector<Point2f>& leftkeypoint, vector<Point2f>& rightkeypoint)
{
	int row = leftphase.rows;
	int col = leftphase.cols;

	cout << " ========================================= " << row << " == " << col << " == " << row * col << endl;

	int k1, k2;
	float left, right;
	Point2f point_left, point_right;
	float* pre_right_data;
	int pre_k;

	for (int j = 0; j < row; j += 1)
	{
		float * left_phase_data = leftphase.ptr<float>(j);
		float * right_phase_data = rightphase.ptr<float>(j);
		float * left_phase_data2;

		for (int i = 0; i < col; i++)
		{
			left = *left_phase_data++;

			//大于2*PI
			if (left > 2 * CV_PI)
			{
				right_phase_data = rightphase.ptr<float>(j);
				k1 = 0;

				// 寻找匹配点
				while ((abs(left - *right_phase_data++) > PHASE_THRESHOLD) && (k1 < col))
					k1++;

				if (k1 < col)
				{
					right = *(--right_phase_data);
					left_phase_data2 = leftphase.ptr<float>(j);
					k2 = 0;
					while ((abs(right - *left_phase_data2++) > PHASE_THRESHOLD) && (k2 < col))
						k2++;

					if ((k2 < col) && (abs(k2 - i) < 2))
					{
						point_left.x = (i + k2) / 2;
						point_left.y = j;
						point_right.x = k1;
						point_right.y = j;
						leftkeypoint.push_back(point_left);
						rightkeypoint.push_back(point_right);
					}
				}
			}
		}
	}

}

void find_featurepionts_single_match(Mat& leftphase, Mat& rightphase, vector<Point2f>& leftkeypoint, vector<Point2f>& rightkeypoint)
{
	int nr = leftphase.rows;
	int nc = leftphase.cols;

	int x, y, k;
	float left;
	Point2f point_left, point_right;

	for (y = 0; y < nr; y += 1)
	{
		float* left_phase_data = leftphase.ptr<float>(y);
		float* right_phase_data = rightphase.ptr<float>(y);

		for (x = 0; x < nc; x++)
		{
			left = *left_phase_data++;

			if (left > 2 * CV_PI)
			{
				right_phase_data = rightphase.ptr<float>(y);
				k = 0;

				while ((abs(left - *right_phase_data++) > PHASE_THRESHOLD) && (k < nc)) k++;
				if (k < nc)
				{
					point_left.x = x;
					point_left.y = y;
					point_right.x = k;
					point_right.y = y;
					leftkeypoint.push_back(point_left);
					rightkeypoint.push_back(point_right);
				}
			}
		}
	}
}

void find_featureBlock(Mat& leftphase, Mat& rightphase, vector<Point2f>& leftkeypoint, vector<Point2f>& rightkeypoint)
{
	int nr = leftphase.rows;
	int nc = leftphase.cols;
	Size Blocksize = Size(3, 3);

	int h = Blocksize.height;
	int w = Blocksize.width;
	int x, y, i, j, k;

	Point2f point_left, point_right;

	float leftdataaver, rightdataaver;
	for (y = 0; y < nr - h; y += h)
	{
		for (x = 0; x < nc - w; x += w)
		{
			leftdataaver = 0;
			for (j = 0; j < h; j++)
			{
				for (i = 0; i < w; i++)
					leftdataaver += leftphase.at<float>(y + j, x + i);
			}
			leftdataaver = leftdataaver / (h * w);

			if (leftdataaver > 2 * CV_PI)
			{
				for (k = 0; k < nc - w; k += w)
				{
					rightdataaver = 0;
					for (j = 0; j < h; j++)
					{
						for (i = 0; i < w; i++)
							rightdataaver += rightphase.at<float>(y + j, k + i);
					}
					rightdataaver = rightdataaver / (h * w);
					if (abs(rightdataaver - leftdataaver) < BLOCK_THRESHOLD)
					{
						point_left.x = x / 2 + 1;
						point_left.y = y / 2 + 1;
						point_right.x = k / 2 + 1;
						point_right.y = y / 2 + 1;
						leftkeypoint.push_back(point_left);
						rightkeypoint.push_back(point_right);
						break;
					}
				}
			}
		}
	}
}

//计算视差
void find_featureSAD(Mat& leftphase, Mat& rightphase)
{
	Mat leftimg8(Size(leftphase.cols, leftphase.rows), CV_8UC1, Scalar(0.0));
	Mat rightimg8(Size(leftphase.cols, leftphase.rows), CV_8UC1, Scalar(0.0));
	convertScaleAbs(leftphase, leftimg8);
	convertScaleAbs(rightphase, rightimg8);

	//     imshow("leftimg8", leftimg8);
	//     imshow("rightimg8", rightimg8);
	//     waitKey(0);
	//     cvConvertScale(&leftphase, leftimg8, 255.0, 0.0);
	//     cvConvertScale(&rightphase, rightimg8, 255.0, 0.0);

	Size img_size = leftphase.size();
	Rect roi1, roi2;
	Mat Q;

	// reading intrinsic parameters
	FileStorage fs(storintrinsicsyml, FileStorage::READ);
	if (!fs.isOpened())
	{
		printf("Failed to open file intrinsic_filename.\n");
		return;
	}

	Mat M1, D1, M2, D2;
	fs["M1"] >> M1;
	fs["D1"] >> D1;
	fs["M2"] >> M2;
	fs["D2"] >> D2;

	fs.open(storextrinsicsyml, FileStorage::READ);
	if (!fs.isOpened())
	{
		printf("Failed to open file extrinsic_filename.\n");
		return;
	}

	Mat R, T, R1, P1, R2, P2;
	fs["R"] >> R;
	fs["T"] >> T;
	fs["R1"] >> R1;
	fs["R2"] >> R2;
	fs["P1"] >> P1;
	fs["P2"] >> P2;

	stereoRectify(M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q,
		0, -1, img_size, &roi1, &roi2);

	int numberOfDisparities = ((img_size.width / 8) + 15) & -16;

	Ptr<StereoBM> bm = StereoBM::create(16, 3);
	bm->setROI1(roi1);
	bm->setROI2(roi2);
	bm->setPreFilterCap(31);
	bm->setBlockSize(5);
	bm->setMinDisparity(0);
	bm->setNumDisparities(numberOfDisparities);
	bm->setTextureThreshold(120);
	bm->setUniquenessRatio(7);
	bm->setSpeckleWindowSize(20);
	bm->setSpeckleRange(64);
	bm->setDisp12MaxDiff(-1);

	Mat disp, disp8;
	bm->compute(leftimg8, rightimg8, disp);
	disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities * 16.));
	double sf = 640. / MAX(disp8.rows, disp8.cols);
	resize(disp8, disp8, Size(), sf, sf);   //调整图像大小640 x 640
	imshow("disparity", disp8);
	waitKey(0);
}

//保存相位
void savePhase(const char* filename, Mat& mat)
{
	FILE* fp = fopen(filename, "wt");
	for (int y = 0; y < mat.rows; y++)
	{
		float* pixel_phase_data = mat.ptr<float>(y);

		for (int x = 0; x < mat.cols; x++)
		{
			float point = *pixel_phase_data++;
			fprintf(fp, "%f \t", point);
		}
		fprintf(fp, "\n");
	}
	fclose(fp);
}

//保存3D点
void savepnts3D(const char* filename, Mat& mat)
{
	FILE* fp = fopen(filename, "wt");
	Mat pnts3Dimg(4000, 4000, CV_8UC1);

	float* pnts3D_row1 = mat.ptr<float>(0);
	float* pnts3D_row2 = mat.ptr<float>(1);
	float* pnts3D_row3 = mat.ptr<float>(2);
	float* pnts3D_row4 = mat.ptr<float>(3);
	int i, j, pixelsvel;

	for (int y = 0; y < mat.cols; y++)
	{
		float pnts3D_data4 = *(pnts3D_row4 + y);

		float pnts3D_data1 = *(pnts3D_row1 + y) / pnts3D_data4;
		float pnts3D_data2 = *(pnts3D_row2 + y) / pnts3D_data4;
		float pnts3D_data3 = *(pnts3D_row3 + y) / pnts3D_data4;

		fprintf(fp, "%f   %f   %f \n", pnts3D_data1, pnts3D_data2, pnts3D_data3);

		i = (int)(10 * pnts3D_data1) + 1000; // col
		j = (int)(10 * pnts3D_data2) + 2000; // row
		pixelsvel = (uchar)(225 * pnts3D_data3 / 1900.00);
		// pixelsvel = pnts3D_data3;
		if (i < pnts3Dimg.cols && j < pnts3Dimg.rows && pixelsvel < 255)
			pnts3Dimg.at<uchar>(j, i) = pixelsvel;
	}
	cv::imwrite("../data/output/pnts3D.jpg", pnts3Dimg);
	fclose(fp);

}