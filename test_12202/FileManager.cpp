#include "FileManager.h"

//保存相位
void savePhase(const char* filename, cv::Mat& mat)
{
	FILE* fp = fopen(filename, "wt");
	for (int y = 0; y < mat.rows; y++)
	{
		float *pixel_phase_data = mat.ptr<float>(y);

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
void savepnts3D(const char* filename, cv::Mat& mat)
{
	FILE* fp = fopen(filename, "wt");

	float *pnts3D_row1 = mat.ptr<float>(0);
	float *pnts3D_row2 = mat.ptr<float>(1);
	float *pnts3D_row3 = mat.ptr<float>(2);
	float *pnts3D_row4 = mat.ptr<float>(3);
	int pixelsvel;

	for (int y = 0; y < mat.cols; y++)
	{
		float pnts3D_data4 = *(pnts3D_row4 + y);

		float pnts3D_data1 = *(pnts3D_row1 + y) / pnts3D_data4;
		float pnts3D_data2 = *(pnts3D_row2 + y) / pnts3D_data4;
		float pnts3D_data3 = *(pnts3D_row3 + y) / pnts3D_data4;

		fprintf(fp, "%f   %f   %f \n", pnts3D_data1, pnts3D_data2, pnts3D_data3);

		pixelsvel = (int)(225 * pnts3D_data3 / 1900.00);
	}

	fclose(fp);
}

