#include "StereoReconstruct.h"
#include "FileManager.h"

#define PHASE_THRESHOLD  0.01
#define BLOCK_THRESHOLD  0.3

void find_featurepionts(Mat& leftphase, Mat& rightphase,
							vector<Point2f>& leftkeypoint, vector<Point2f>& rightkeypoint)
{
	int nr = leftphase.rows;
	int nc = leftphase.cols;
	int x, y, k1, k2;
	float left, right;
	Point2f fleft, fright;
	float *pre_right_data;
	int pre_k;

	for (y = 0; y < nr; y += 1)
	{
		float *left_phase_data = leftphase.ptr<float>(y);
		float *right_phase_data = rightphase.ptr<float>(y);
		float *left_phase_data2;
		// k = 0;
		// pre_right_data = right_phase_data;
		// pre_k = k;

		for (x = 0; x < nc; x++)
		{
			left = *left_phase_data++;

			if (left > 2 * CV_PI)
			{
				// right_phase_data = pre_right_data;
				// k = pre_k;
				// order constraint	
				right_phase_data = rightphase.ptr<float>(y);
				k1 = 0;

				while ((abs(left - *right_phase_data++) > PHASE_THRESHOLD) && (k1 < nc)) k1++;
				if (k1 < nc)
				{
					//pre_right_data = right_phase_data;
					//pre_k = k;
					right = *(--right_phase_data);
					left_phase_data2 = leftphase.ptr<float>(y);
					k2 = 0;
					while ((abs(right - *left_phase_data2++) > PHASE_THRESHOLD) && (k2 < nc))
						k2++;

					if ((k2 < nc) && (abs(k2 - x) < 2))
					{
						fleft.x = (x + k2) / 2;
						fleft.y = y;
						fright.x = k1;
						fright.y = y;
						leftkeypoint.push_back(fleft);
						rightkeypoint.push_back(fright);
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
	Point2f fleft, fright;

	for (y = 0; y < nr; y += 1)
	{
		float *left_phase_data = leftphase.ptr<float>(y);
		float *right_phase_data = rightphase.ptr<float>(y);

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
					fleft.x = x;
					fleft.y = y;
					fright.x = k;
					fright.y = y;
					leftkeypoint.push_back(fleft);
					rightkeypoint.push_back(fright);
				}
			}
		}
	}
}

void find_featureBlock(Mat& leftphase, Mat& rightphase,
							vector<Point2f>& leftkeypoint, vector<Point2f>& rightkeypoint)
{
	int nr = leftphase.rows;
	int nc = leftphase.cols;
	Size Blocksize = Size(3, 3);

	int h = Blocksize.height;
	int w = Blocksize.width;
	int x, y, i, j, k;

	Point2f fleft, fright;

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
			leftdataaver = leftdataaver / (h*w);

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
					rightdataaver = rightdataaver / (h*w);
					if (abs(rightdataaver - leftdataaver) < BLOCK_THRESHOLD)
					{
						fleft.x = x / 2 + 1;
						fleft.y = y / 2 + 1;
						fright.x = k / 2 + 1;
						fright.y = y / 2 + 1;
						leftkeypoint.push_back(fleft);
						rightkeypoint.push_back(fright);
						break;
					}
				}
			}
		}
	}
}

void find_featureSAD(Mat& leftphase, Mat& rightphase)
{
	Mat leftimg8(Size(leftphase.cols, leftphase.rows), CV_8UC1, Scalar(0.0));
	Mat rightimg8(Size(leftphase.cols, leftphase.rows), CV_8UC1, Scalar(0.0));
	convertScaleAbs(leftphase, leftimg8);
	convertScaleAbs(rightphase, rightimg8);

	// imshow("leftimg8", leftimg8);
	// imshow("rightimg8", rightimg8);
	// waitKey(0);
	// cvConvertScale(&leftphase, leftimg8, 255.0, 0.0);
	// cvConvertScale(&rightphase, rightimg8, 255.0, 0.0);

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
	disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities*16.));
	double sf = 640. / MAX(disp8.rows, disp8.cols);
	resize(disp8, disp8, Size(), sf, sf);   //调整图像大小640 x 640
	imshow("disparity", disp8);
	waitKey(0);
}
