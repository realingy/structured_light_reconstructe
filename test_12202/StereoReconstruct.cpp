#include "StereoReconstruct.h"
#include "FileManager.h"

#define PHASE_THRESHOLD  0.01
#define BLOCK_THRESHOLD  0.3

void find_featurepionts(Mat& leftphase, Mat& rightphase, vector<Point2f>& leftkeypoint, vector<Point2f>& rightkeypoint)
{
	int nr = leftphase.rows;
	int nc = leftphase.cols;
	int x, y, k1, k2;
	float left, right;
	Point2f fleft, fright;
	float *pre_right_data;
	int pre_k;

	// 寻找匹配点的思路：在同一行中，先在右图中寻找和左图中的点P1相位绝对差小于阈值的匹配点P2，再在左图中寻找P2的相位绝对差小于阈值的匹配点P3，
	// 若P1和P3的距离够近，则（P1+P3）/2、P2就是一对匹配点对。
	for (y = 0; y < nr; y += 1)
	{
		float *left_phase_data = leftphase.ptr<float>(y);
		float *right_phase_data = rightphase.ptr<float>(y);
		float *left_phase_data2;

		for (x = 0; x < nc; x++)
		{
			left = *left_phase_data++;

			if (left > 2 * CV_PI)
			{
				right_phase_data = rightphase.ptr<float>(y);
				k1 = 0;

				while ((abs(left - *right_phase_data++) > PHASE_THRESHOLD) && (k1 < nc))
				{
					k1++;
				}
				if (k1 < nc)
				{
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

