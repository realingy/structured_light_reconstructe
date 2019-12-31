#include <iostream>
#include<stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <direct.h>
#include <time.h>

#include "structuredlight.h"
#include "auxiliaryfunctions.h"
#include "phaseshift.h"
#include "scan.h"

#define _CRT_SECURE_NO_WARNINGS

using namespace std;
using namespace cv;

//colorize image like matlab,input must range from 0 to 255
void ColorizeWinter(Mat &src, Mat &dst, Mat &mask)
{
	//src is 1-channel,dst is 3-channel 
	vector<Mat> color_map(3);
	vector<Mat> output(1);
	Mat temp1(src.rows,src.cols, CV_8UC1,Scalar(255));
	Mat temp2(src.rows, src.cols, CV_8UC3, Scalar(0));

	src.copyTo(temp1, mask);
	color_map[0] =Scalar(255)-temp1;  //decreasing
	color_map[1] = src;				  //increasing
	temp1.setTo(0);
	color_map[2] = temp1;			  //keep zero
	output[0] = temp2;

	int fromTo[] = { 0, 0, 1, 2, 2, 1 }; //determine channel to mix, BGR
	mixChannels(color_map,output,fromTo,3);
	output[0].copyTo(dst);
}


//src:CV_64FC3,channel_3 is depth, dst:CV_8UC1, mask:CV_8UC1
void DepthMapConvertToGray(const Mat &src, Mat &dst,const Mat &mask,int depth_min,int depth_max)
{
	const double *ptr_src = src.ptr<double>(0);
	const unsigned char *ptr_mask = mask.ptr<unsigned char>(0);
	dst.create(src.size(), CV_8UC1);
	dst.setTo(0);
	unsigned char *ptr_dst = dst.ptr<unsigned char>(0);
	
	int step = src.cols;
	double scale = 255.0f / depth_max;

	for (int r = 0; r < src.rows - 1; ++r){
		for (int c = 0; c <src.cols - 1; c += 1){
			if (ptr_mask[r*step + c] != 0){
				ptr_dst[r*step + c] = (depth_max-ptr_src[r*step * 3 + 3 * c + 2]) *scale;
			}
		}
	}
}


// Find intersection between a 3D plane and a 3D line.
// Note: Finds the point of intersection of a line in parametric form 
//       P=Ql+namate*V; a plane W defined in implicit form: 
//       Nt*(P-Qp)=0 =>w0*x+w1*y+w2*z=w3
//       namate=Nt*(Qp-Ql)/(Nt*V)=(w3-Nt*ql)/(Nt*V)
int  IntersectLineWithPlane3D(double *ql, double *qv, double *w,double *intersect_point,double &namate)
{
	double dot_q = 0, dot_v = 0;
	//calculate inner product
	for (int i = 0; i < 3; ++i)
	{
		dot_q += w[i] * ql[i];
		dot_v += w[i] * qv[i];
		intersect_point[i] = 0.0f;
	}

	//calculate namate and intersect point
	namate = (w[3] - dot_q) / dot_v;
	for (int i = 0; i < 3; ++i)
		intersect_point[i] = ql[i] + namate*qv[i];
	return 0;
}


//save points and colors as an X3D file
int SaveX3DFile(char *filename, Mat &points, Mat &colors, Mat &mask)
{
	FILE *pFile = fopen(filename, "w");
	if (pFile == NULL)
	{
		cout << "can not open X3D file....." << endl;
		return -1;
	}
	fprintf(pFile, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
	fprintf(pFile, "<X3D profile=\"immersive\">\n");
	fprintf(pFile, "<Scene>\n");
	fprintf(pFile, "<Shape>\n");
	fprintf(pFile, "<Box size=\"1 1 2\"/>\n");
	fprintf(pFile, "<Appearance>\n");
	fprintf(pFile, "<Material diffuseColor=\"1 0 0\"/>\n");
	fprintf(pFile, "</Appearance>\n");
	fprintf(pFile, "</Shape>\n");


	//write points' coordinate if mask!=0
	fprintf(pFile, "<Shape>\n");
	fprintf(pFile, "<PointSet>\n");
	fprintf(pFile, "<Coordinate point=\n");
	fprintf(pFile, "\"");
	for (int r = 0; r < points.rows; ++r)
		for (int c = 0; c < points.cols-1; ++c)
		{
			if (mask.at<unsigned char>(r, c) != 0)
			{
				fprintf(pFile, "%f %f %f,\n", points.at<Vec3d>(r, c)[0], points.at<Vec3d>(r, c)[1], points.at<Vec3d>(r, c)[2]);
			}
		}
	fprintf(pFile, "%f %f %f", points.at<Vec3d>(points.rows - 1, points.cols - 1)[0], points.at<Vec3d>(points.rows - 1, points.cols - 1)[1], points.at<Vec3d>(points.rows-1, points.cols-1)[2]);
	fprintf(pFile, "\"/>\n");
	fprintf(pFile, "<Color color=\n");
	fprintf(pFile, "\"");
	for (int r = 0; r < colors.rows; ++r)
		for (int c = 0; c < colors.cols - 1; ++c)
		{
		if (mask.at<unsigned char>(r, c) != 0)
		{
			fprintf(pFile, "%f %f %f,\n", colors.at<Vec3d>(r, c)[0], colors.at<Vec3d>(r, c)[1], colors.at<Vec3d>(r, c)[2]);
			// fprintf(pFile, "%f %f %f", 0.5, 0.0, 0.5);
		}
		}
	fprintf(pFile, "%f %f %f", colors.at<Vec3d>(colors.rows - 1, colors.cols - 1)[0], colors.at<Vec3d>(colors.rows - 1, colors.cols - 1)[1], colors.at<Vec3d>(colors.rows - 1, colors.cols - 1)[2]);
	fprintf(pFile, "\"/>\n");

	fprintf(pFile, "</PointSet>\n");
	fprintf(pFile, "</Shape>\n");

	fprintf(pFile, "</Scene>\n");
	fprintf(pFile, "</X3D>\n");
	fclose(pFile);

	return 0;
}

int GenerateGrayCode(SlParameter &sl_parameter)
{
	ostringstream save_name;
	//define colum scan image,saving at 1~colum_scan_amount
	if (sl_parameter.colum_scan_flag){
		cout << "generate graycode colum scan image......" << endl;
		for (int i = 0; i < sl_parameter.projector_width; ++i){
			for (int j = 1; j <= sl_parameter.colum_scan_amount; ++j){
				if (j == 1)
					sl_parameter.projector_gray_code_image[j].at<unsigned char>(0, i) = (((i + sl_parameter.colum_scan_shift) >> (sl_parameter.colum_scan_amount - j)) & 1);
				else
					sl_parameter.projector_gray_code_image[j].at<unsigned char>(0, i) = (((i + sl_parameter.colum_scan_shift) >> (sl_parameter.colum_scan_amount - j + 1)) & 1) ^ ((i + sl_parameter.colum_scan_shift) >> (sl_parameter.colum_scan_amount - j) & 1);
				sl_parameter.projector_gray_code_image[j].at<unsigned char>(0, i) *= 255;
				for (int k = 1; k < sl_parameter.projector_height; ++k){
					sl_parameter.projector_gray_code_image[j].at<unsigned char>(k, i) = sl_parameter.projector_gray_code_image[j].at<unsigned char>(0, i);
				}
			}
		}
	}

	//define row scan image,saving at colum_scan_amount+1~colum_scan_amount+row_scan_amount+1
	if (sl_parameter.row_scan_flag){
		cout << "generate graycode row scan image......" << endl;
		for (int i = 0; i < sl_parameter.projector_height; ++i){
			for (int j = 1; j <= sl_parameter.row_scan_amount; ++j){
				if (j == 1)
					sl_parameter.projector_gray_code_image[sl_parameter.colum_scan_amount + j].at<unsigned char>(i, 0) = (((i + sl_parameter.row_scan_shift) >> (sl_parameter.row_scan_amount - j)) & 1);
				else
					sl_parameter.projector_gray_code_image[sl_parameter.colum_scan_amount + j].at<unsigned char>(i, 0) = (((i + sl_parameter.row_scan_shift) >> (sl_parameter.row_scan_amount - j + 1)) & 1) ^ (((i + sl_parameter.row_scan_shift) >> (sl_parameter.row_scan_amount - j)) & 1);
				sl_parameter.projector_gray_code_image[sl_parameter.colum_scan_amount + j].at<unsigned char>(i, 0) *= 255;
				for (int k = 1; k < sl_parameter.projector_width; ++k){
					sl_parameter.projector_gray_code_image[sl_parameter.colum_scan_amount + j].at<unsigned char>(i, k) = sl_parameter.projector_gray_code_image[sl_parameter.colum_scan_amount + j].at<unsigned char>(i, 0);
				}
			}
		}
	}

	//保存格雷码和反码
	// save graycode and inverse graycode
	Mat pattern_inverse(sl_parameter.projector_height, sl_parameter.projector_width, CV_8UC1,Scalar(0));
	if (sl_parameter.colum_scan_flag){
		for (int i = 0; i <= sl_parameter.colum_scan_amount; ++i){
			save_name.str("");
			save_name << ".\\output\\test\\projector_scan_image_" << i << ".jpg";
			imwrite(string(save_name.str()), sl_parameter.projector_gray_code_image[i]);
			imshow("projector window", sl_parameter.projector_gray_code_image[i]);
			waitKey(sl_parameter.project_capture_delay / 2);

			pattern_inverse.setTo(255);
			pattern_inverse = pattern_inverse - sl_parameter.projector_gray_code_image[i];
			save_name.str("");
			save_name << ".\\output\\test\\projector_scan_image_inverse_" << i << ".jpg";
			imwrite(string(save_name.str()), pattern_inverse);
			imshow("projector window", pattern_inverse);
			waitKey(sl_parameter.project_capture_delay / 2);
		}
	}

	if (sl_parameter.row_scan_flag){
		for (int i = sl_parameter.colum_scan_amount+1; i <= sl_parameter.colum_scan_amount + sl_parameter.row_scan_amount; ++i){	
			save_name.str("");
			save_name << ".\\output\\test\\projector_scan_image_" << i << ".jpg";
			imwrite(string(save_name.str()), sl_parameter.projector_gray_code_image[i]);
			imshow("projector window", sl_parameter.projector_gray_code_image[i]);
			waitKey(sl_parameter.project_capture_delay / 2);

			pattern_inverse.setTo(255);
			pattern_inverse = pattern_inverse - sl_parameter.projector_gray_code_image[i];
			save_name.str("");
			save_name << ".\\output\\test\\projector_scan_image_inverse_" << i << ".jpg";
			imwrite(string(save_name.str()), pattern_inverse);
			imshow("projector window", pattern_inverse);
			waitKey(sl_parameter.project_capture_delay / 2);
		}
	}
	imshow("projector window", sl_parameter.projector_gray_code_image[0]);
	waitKey(sl_parameter.project_capture_delay);
	return 0;
}

int ReadGrayCode(SlParameter &sl_parameter)
{
	ostringstream save_name;
	if (sl_parameter.colum_scan_flag){
		cout << "read graycode colum scan image....." << endl;
		for (int i = 0; i <= sl_parameter.colum_scan_amount; ++i){
			save_name.str("");
			save_name << ".\\output\\test\\projector_scan_image_" << i << ".jpg";	//read as grayscale
			sl_parameter.projector_gray_code_image[i] = imread(string(save_name.str()),0);
		}
	}
	if (sl_parameter.row_scan_flag){
		cout << "read graycode colum scan image....." << endl;
		for (int i = sl_parameter.colum_scan_amount + 1; i <= sl_parameter.colum_scan_amount + sl_parameter.row_scan_amount; ++i){
			save_name.str("");
			save_name << ".\\output\\test\\projector_scan_image_" << i << ".jpg";
			sl_parameter.projector_gray_code_image[i] = imread(string(save_name.str()),0);  //read as grayscale
		}
	}
	imshow("projector window", sl_parameter.projector_gray_code_image[0]);
	waitKey(sl_parameter.project_capture_delay);
	return 1;
}

int CaptureLivingImage(SlParameter &sl_parameter)
{
	cout << "please adjust object to least shade....." << endl;
	cout << "press 'e' to skip...." << endl;
	Mat frame_grab(sl_parameter.camera_height,sl_parameter.camera_width,CV_8UC3);
	Mat projector_frame(sl_parameter.projector_height,sl_parameter.projector_width,CV_8UC1);
	projector_frame.setTo(255);
	int key = -1;
	namedWindow("place object", WINDOW_NORMAL);
	createTrackbar("camera_gain:", "place object", &sl_parameter.camera_gain, 200, NULL);
	createTrackbar("projector_gain:", "place object", &sl_parameter.projector_gain, 200, NULL);
	while (key != 'e')
	{
		projector_frame.setTo(255);
		projector_frame = projector_frame * (sl_parameter.projector_gain/100.0f);
		imshow("projector window", projector_frame);
		
		GetImage(frame_grab);
		frame_grab = frame_grab*(sl_parameter.camera_gain / 100.0f);
		imshow("place object", frame_grab);
		key = waitKey(100);
	}

	projector_frame.setTo(0);
	imshow("projector window",projector_frame);
	waitKey(200);
	destroyWindow("place object");
	cout << "place object successful!" << endl << endl;
	return 0;
}

int RunScanObject(SlParameter &sl_parameter, bool save_enable)
{
	ostringstream save_name;
	Mat frame_grab(sl_parameter.camera_height, sl_parameter.camera_width, CV_8UC3);
	Mat projector_frame(sl_parameter.projector_height, sl_parameter.projector_width, CV_8UC1);
	namedWindow("scan object", WINDOW_AUTOSIZE);
	moveWindow("scan object", 80, 0);

	//enable colum scan 
	if (sl_parameter.colum_scan_flag){
		cout << "run colum scan object....." << endl;
		for (int i = 0; i <= sl_parameter.colum_scan_amount-1; ++i){
			//Display gray code image and capture,save
			projector_frame = sl_parameter.projector_gray_code_image[i] * (sl_parameter.projector_gain / 100.0f);
			imshow("projector window", projector_frame);
			waitKey(sl_parameter.project_capture_delay);
			if (GetImage(frame_grab)){
				sl_parameter.camera_gray_code_image[2 * i] = frame_grab;
//				sl_parameter.camera_gray_code_image[2 * i] = frame_grab*(sl_parameter.camera_gain / 100.0f);
				imshow("scan object", frame_grab);
				if (save_enable){
					save_name.str("");
					save_name << ".\\output\\test\\camera_scan_image_" << 2 * i << ".jpg";
					imwrite(string(save_name.str()), sl_parameter.camera_gray_code_image[2 * i]);
					
				}
			}
//			waitKey(sl_parameter.project_capture_delay); //must delay betweenn two frame

			//Display inverse gray code image and capture,save
			projector_frame.setTo(255);
			projector_frame = projector_frame - sl_parameter.projector_gray_code_image[i];
			projector_frame = projector_frame*(sl_parameter.projector_gain / 100.0f);
			imshow("projector window", projector_frame);
			waitKey(sl_parameter.project_capture_delay);
			if (GetImage(frame_grab)){
				sl_parameter.camera_gray_code_image[2 * i + 1] = frame_grab;
//				sl_parameter.camera_gray_code_image[2 * i + 1] = frame_grab*(sl_parameter.camera_gain / 100.0f);
				imshow("scan object", frame_grab);
				if (save_enable){
					save_name.str("");
					save_name << ".\\output\\test\\camera_scan_image_" << 2 * i + 1 << ".jpg";
					imwrite(string(save_name.str()), sl_parameter.camera_gray_code_image[2 * i + 1]);
				}
			}
//			waitKey(sl_parameter.project_capture_delay); //must delay betweenn two frame
		}
	}

	//enable row scan 
	if (sl_parameter.row_scan_flag){
		cout << "run row scan object....." << endl;
		for (int i = sl_parameter.colum_scan_amount + 1; i <= sl_parameter.colum_scan_amount + sl_parameter.row_scan_amount; ++i){
			//Display gray code image and capture,save
			projector_frame = sl_parameter.projector_gray_code_image[i] * (sl_parameter.projector_gain / 100.0f);
			imshow("projector window", projector_frame);
			waitKey(sl_parameter.project_capture_delay);
			if (GetImage(frame_grab)){
				sl_parameter.camera_gray_code_image[2 * i] = frame_grab*(sl_parameter.camera_gain / 100.0f);
				imshow("scanning object", sl_parameter.camera_gray_code_image[2 * i]);
				if (save_enable){
					save_name.str("");
					save_name << ".\\output\\test\\camera_scan_image_" << 2 * i << ".jpg";
					imwrite(string(save_name.str()), sl_parameter.camera_gray_code_image[2 * i]);
				}
			}
			//Display inverse gray code image and capture,save
			projector_frame.setTo(255);
			projector_frame = projector_frame - sl_parameter.projector_gray_code_image[i];
			projector_frame = projector_frame*(sl_parameter.projector_gain / 100.0f);
			imshow("projector window", projector_frame);
			waitKey(sl_parameter.project_capture_delay);
			if (GetImage(frame_grab)){
				sl_parameter.camera_gray_code_image[2 * i + 1] = frame_grab*(sl_parameter.camera_gain / 100.0f);
				imshow("scanning object", sl_parameter.camera_gray_code_image[2 * i + 1]);
				if (save_enable){
					save_name.str("");
					save_name << ".\\output\\test\\camera_scan_image_" << 2 * i + 1 << ".jpg";
					imwrite(string(save_name.str()), sl_parameter.camera_gray_code_image[2 * i + 1]);
				}
			}
		}
	}
	return 0;
}

int ReadScanImage(SlParameter &sl_parameter)
{
	ostringstream read_name;
	if (sl_parameter.colum_scan_flag){
		cout << "read graycode colum scan image....." << endl;
		for (int i = 0; i <= (2 * sl_parameter.colum_scan_amount + 1); ++i){
			read_name.str("");
			read_name << ".\\output\\test\\camera_scan_image_" << i << ".jpg";
			sl_parameter.camera_gray_code_image[i] = imread(string(read_name.str()));
		}
	}
	if (sl_parameter.row_scan_flag){
		cout << "read graycode row scan image....." << endl << endl;
		for (int i = 2 * (sl_parameter.colum_scan_amount + 1); i <= (2 * (sl_parameter.colum_scan_amount + sl_parameter.row_scan_amount) + 1); ++i){
			read_name.str("");
			read_name << ".\\output\\test\\camera_scan_image_" << i << ".jpg";
			sl_parameter.camera_gray_code_image[i] = imread(string(read_name.str()));
		}
	}
	return 1;
}

int DecodeGrayCode(SlParameter &sl_parameter)
{
	Mat temp(sl_parameter.camera_height, sl_parameter.camera_width, CV_16UC1,Scalar(0));  
	Mat gray_code_image_gray(sl_parameter.camera_height, sl_parameter.camera_width, CV_8UC1, Scalar(0));			//use for gray image of normal gray_code_image 
	Mat gray_code_inverse_image_gray(sl_parameter.camera_height, sl_parameter.camera_width, CV_8UC1, Scalar(0));    //use for gray image of inverse gray_code_image
	Mat gray_code_image_bit_h(sl_parameter.camera_height, sl_parameter.camera_width, CV_8UC1, Scalar(0));	  //image's pixel value is 0 or 1,use for converting graycode to decimal 
	Mat gray_code_image_bit_l(sl_parameter.camera_height, sl_parameter.camera_width, CV_8UC1, Scalar(0));	  //image's pixel value is 0 or 1,use for converting graycode to decimal 
	Mat gray_temp(sl_parameter.camera_height, sl_parameter.camera_width, CV_8UC1, Scalar(0));	  //gray difference  between image and inverse image
	sl_parameter.decode_colum_scan_image.setTo(0);
	sl_parameter.decode_row_scan_image.setTo(0);
	sl_parameter.gray_valid_image.setTo(0);
	sl_parameter.scene_color.setTo(0);

	//decode  gray code colum scan  
	if (sl_parameter.colum_scan_flag)
	{
		cout << "decode graycode colum scan image......." << endl;
		for (int i = 1; i <= sl_parameter.colum_scan_amount-1; ++i)
		{
			cvtColor(sl_parameter.camera_gray_code_image[2 * i], gray_code_image_gray, COLOR_RGB2GRAY);
			cvtColor(sl_parameter.camera_gray_code_image[2 * i + 1], gray_code_inverse_image_gray, COLOR_RGB2GRAY);
			gray_code_image_bit_h = (gray_code_image_gray >= gray_code_inverse_image_gray);

			absdiff(gray_code_image_gray, gray_code_inverse_image_gray, gray_temp);
			gray_temp = (gray_temp >= sl_parameter.contrast_threshold);
			sl_parameter.gray_valid_image = sl_parameter.gray_valid_image | gray_temp;

			if (i == 1)
				gray_code_image_bit_l = gray_code_image_bit_h.clone();
			else
				gray_code_image_bit_l = gray_code_image_bit_l^gray_code_image_bit_h;
			//sl_parameter.decode_colum_scan_image = sl_parameter.decode_colum_scan_image + gray_code_image_bit_l*(double(pow(2.0f, sl_parameter.colum_scan_amount - i) / 255.0f));
			add(sl_parameter.decode_colum_scan_image, Scalar(pow(2.0f, sl_parameter.colum_scan_amount - i)), sl_parameter.decode_colum_scan_image, gray_code_image_bit_l);
		}
		//subtraction colum shift
		sl_parameter.decode_colum_scan_image = sl_parameter.decode_colum_scan_image - sl_parameter.colum_scan_shift;

		//elimate invalid pixel:1.difference between image and inverse image lower than contrast thresold; 
		//2.x's value out range of 0~projector_width; 3.y's value out range of 0~projector_height  
		gray_temp = (sl_parameter.decode_colum_scan_image <sl_parameter.projector_width);
		sl_parameter.gray_valid_image = sl_parameter.gray_valid_image&gray_temp;
		gray_temp = (sl_parameter.decode_colum_scan_image >= 0);
		sl_parameter.gray_valid_image = sl_parameter.gray_valid_image&gray_temp;

		temp.setTo(0);  //pay attention to decode_colum_scan_image is CV_16U
		sl_parameter.decode_colum_scan_image.copyTo(temp, sl_parameter.gray_valid_image);
		temp.copyTo(sl_parameter.decode_colum_scan_image);
	}

	//decode gray code row scan
	if (sl_parameter.row_scan_flag)
	{
		cout << "decode gray code row scan image......." << endl;
		for (int i = 1; i <= sl_parameter.row_scan_amount; ++i)
		{
			cvtColor(sl_parameter.camera_gray_code_image[2 * (sl_parameter.colum_scan_amount + i)], gray_code_image_gray, COLOR_RGB2GRAY);
			cvtColor(sl_parameter.camera_gray_code_image[2 * (sl_parameter.colum_scan_amount + i) + 1], gray_code_inverse_image_gray, COLOR_RGB2GRAY);
			gray_code_image_bit_h = (gray_code_image_gray >= gray_code_inverse_image_gray);

			absdiff(gray_code_image_gray, gray_code_inverse_image_gray, gray_temp);
			gray_temp = (gray_temp >= sl_parameter.contrast_threshold);
			sl_parameter.gray_valid_image = sl_parameter.gray_valid_image | gray_temp;

			if (i == 1)
				gray_code_image_bit_l = gray_code_image_bit_h.clone();
			else
				gray_code_image_bit_l = gray_code_image_bit_l^gray_code_image_bit_h;
			//sl_parameter.decode_colum_scan_image = sl_parameter.decode_colum_scan_image + gray_code_image_bit_l*(double(pow(2.0f, sl_parameter.colum_scan_amount - i) / 255.0f));
			add(sl_parameter.decode_row_scan_image, Scalar(pow(2.0f, sl_parameter.row_scan_amount - i)), sl_parameter.decode_row_scan_image, gray_code_image_bit_l);
		}
		//subtraction row shift
		sl_parameter.decode_row_scan_image = sl_parameter.decode_row_scan_image - sl_parameter.row_scan_shift;

		//elimate invalid pixel:1.difference between image and inverse image lower than contrast thresold; 
		//2.x's value out range of 0~projector_width; 3.y's value out range of 0~projector_height  
		gray_temp = (sl_parameter.decode_row_scan_image <sl_parameter.projector_height);
		sl_parameter.gray_valid_image = sl_parameter.gray_valid_image&gray_temp;
		gray_temp = (sl_parameter.decode_row_scan_image >= 0);
		sl_parameter.gray_valid_image = sl_parameter.gray_valid_image&gray_temp;

		temp.setTo(0);  //pay attention to decode_colum_scan_image is CV_16U
		sl_parameter.decode_row_scan_image.copyTo(temp, sl_parameter.gray_valid_image);
		temp.copyTo(sl_parameter.decode_row_scan_image);
	}

	//choose which image as scene scene image
	Mat temp1(sl_parameter.camera_height, sl_parameter.camera_width, CV_16UC3, Scalar(0));
	temp1= (sl_parameter.camera_gray_code_image[0] + sl_parameter.camera_gray_code_image[1])/2;
	sl_parameter.scene_color = temp1;
	return 0;
}

int DisplayDecodeResult(SlParameter &sl_parameter)
{
	cout << "display decode result......" << endl;
	Mat image_temp1(sl_parameter.camera_height, sl_parameter.camera_width, CV_8UC1, Scalar(0));
	Mat image_temp2(sl_parameter.camera_height, sl_parameter.camera_width, CV_8UC3, Scalar(0));
	Mat image_temp3(sl_parameter.camera_height, sl_parameter.camera_width, CV_8UC3, Scalar(0));
	Mat depth_map(sl_parameter.camera_height, sl_parameter.camera_width, CV_8UC3, Scalar(0));

	//diaplay colum scan result  
	if (sl_parameter.colum_scan_flag)
	{
		namedWindow("colum scan result", WINDOW_NORMAL);
		resizeWindow("colum scan result", 500, 400);
		moveWindow("colum scan result", 100, 100);
		cout << "display gray code colum scan result..... " << endl;
		sl_parameter.decode_colum_scan_image.convertTo(image_temp1, CV_8UC1, double(255.0 / sl_parameter.projector_width));
		ColorizeWinter(image_temp1, image_temp2, sl_parameter.gray_valid_image);
		imshow("colum scan result", image_temp2);
		waitKey(20);

		image_temp3 = imread(".\\output\\test\\背景列扫描图1.jpg");
		cvtColor(image_temp3, image_temp3, COLOR_RGB2GRAY);
		subtract(image_temp1, image_temp3, depth_map, sl_parameter.gray_valid_image);
		depth_map = depth_map;
		depth_map.convertTo(depth_map, CV_8UC1, double(255.0 / (50)));
		destroyWindow("colum scan result");
	}

	//diaplay row scan result  
	if (sl_parameter.row_scan_flag)
	{
		namedWindow("row scan result", WINDOW_NORMAL);
		resizeWindow("row scan result", 500, 400);
		moveWindow("row scan result", 600, 100);
		cout << "display gray code row scan result..... " << endl << endl;
		sl_parameter.decode_row_scan_image.convertTo(image_temp1, CV_8UC1, double(255.0 / sl_parameter.projector_height));
		ColorizeWinter(image_temp1, image_temp2, sl_parameter.gray_valid_image);
		imshow("row scan result", image_temp2);
		waitKey(20);
		image_temp3 = imread(".\\output\\test\\背景行扫描图1.jpg");
		cvtColor(image_temp3, image_temp3, COLOR_RGB2GRAY);
		subtract(image_temp1, image_temp3, depth_map, sl_parameter.gray_valid_image);
		depth_map.convertTo(depth_map, CV_8UC1, double(255.0 / (50)));
		destroyWindow("row scan result");
	}
	return 0; 
}

//深度图重建
int ReconstructDepthMap(SlParameter &sl_parameter, SlCalibration &sl_calibration)
{
	sl_parameter.depth_valid.setTo(0);
	sl_parameter.depth_namate.setTo(0.0f);
	sl_parameter.depth_points.setTo(0.0f);
	sl_parameter.depth_colors.setTo(0);

	cout << "reconstruct depth map......" << endl;
	for (int r = 0; r < sl_parameter.camera_height; ++r)
		for (int c = 0; c < sl_parameter.camera_width; ++c)
		{
			double ql[3], qv[3], w[4];
			double namate_col = 0.0f, namate_row = 0.0f;
			double intersect_point_col[3], intersect_point_row[3];
			//just calculate depth for valid pixel
			if (sl_parameter.gray_valid_image.at<unsigned char>(r, c) != 0)
			{
				//reconstruct by colum scan
				if (sl_parameter.colum_scan_flag)
				{
					//read camera center and current pixel's ray vector
					for (int i = 0; i < 3; ++i)
					{
						ql[i] = sl_calibration.camera_projection_center.at<double>(i, 0);
						qv[i] = sl_calibration.camera_pixel_rays.at<double>(i, r*sl_parameter.camera_width + c);
					}
					//read current pixel's projector colum value and corresponding colum plane
					int corresponding_colum = 0;
					corresponding_colum = sl_parameter.decode_colum_scan_image.at<unsigned short>(r, c);
					if (corresponding_colum>sl_parameter.projector_width - 1)	//pay attention to this
						corresponding_colum = sl_parameter.projector_width - 1;
					for (int i = 0; i < 4; ++i)
					{
						w[i] = sl_calibration.projector_colum_planes.at<double>(i, corresponding_colum);
					}
					//calculate camera ray intersect with projector plane
					IntersectLineWithPlane3D(ql, qv, w, intersect_point_col, namate_col);
				}
				//reconstruct by row scan
				if (sl_parameter.row_scan_flag)
				{
					//read camera center and current pixel's ray vector
					for (int i = 0; i < 3; ++i)
					{
						ql[i] = sl_calibration.camera_projection_center.at<double>(i, 0);
						qv[i] = sl_calibration.camera_pixel_rays.at<double>(i, r*sl_parameter.camera_width + c);
					}
					//read current pixel's projector row value and corresponding  row plane
					int corresponding_row = 0;
					corresponding_row = sl_parameter.decode_row_scan_image.at<unsigned short>(r, c);
					if (corresponding_row>sl_parameter.projector_height - 1)	//pay attention to this
						corresponding_row = sl_parameter.projector_height - 1;
					for (int i = 0; i < 4; ++i)
					{
						w[i] = sl_calibration.projector_row_planes.at<double>(i, corresponding_row);
					}
					//calculate camera ray intersect with projector plane
					IntersectLineWithPlane3D(ql, qv, w, intersect_point_row, namate_row);
				}

				//determine which namate and point to be used 
				if (sl_parameter.colum_scan_flag&&sl_parameter.row_scan_flag)
				{
					sl_parameter.depth_namate.at<double>(r,c) = (namate_col + namate_row) / 2.0f;
					for (int i = 0; i < 3; ++i)
					sl_parameter.depth_points.at<Vec3d>(r,c)[i] = (intersect_point_col[i] + intersect_point_row[i]) / 2.0f;
				}
				else if (sl_parameter.colum_scan_flag)
				{
					sl_parameter.depth_namate.at<double>(r, c) = namate_col;
					for (int i = 0; i < 3; ++i)
						sl_parameter.depth_points.at<Vec3d>(r, c)[i] = intersect_point_col[i];
				}
				else if (sl_parameter.row_scan_flag)
				{
					sl_parameter.depth_namate.at<double>(r, c) = namate_row;
					for (int i = 0; i < 3; ++i)
						sl_parameter.depth_points.at<Vec3d>(r, c)[i] = intersect_point_row[i];
				}
				else
				{
					sl_parameter.gray_valid_image.at<unsigned char>(r, c) = 0;
					sl_parameter.depth_namate.at<double>(r, c) = 0.0f;
					for (int i = 0; i < 3; ++i)
						sl_parameter.depth_points.at<Vec3d>(r, c)[i] = 0.0f;
				}

				//extract color information from full write image,note: opencv's default color order is BGR differs from RGB order we desired 
				for (int i = 0; i < 3; ++i)
					sl_parameter.depth_colors.at<Vec3d>(r, c)[i] = (double)sl_parameter.scene_color.at<Vec3b>(r, c)[2-i]/255.0f;
				//update valid depth pixel
				sl_parameter.depth_valid.at<unsigned char>(r, c) = 1;

				//reject any points too near or far by using min_distance<z<max_distance
				if (sl_parameter.depth_points.at<Vec3d>(r, c)[2] < sl_parameter.distance_range[0] || sl_parameter.depth_points.at<Vec3d>(r, c)[2] > sl_parameter.distance_range[1])
				{
					sl_parameter.gray_valid_image.at<unsigned char>(r, c) = 0;
					sl_parameter.depth_valid.at<unsigned char>(r, c) = 0;
					sl_parameter.depth_namate.at<double>(r, c) = 0.0f;
					for (int i = 0; i < 3; ++i)
					{
						sl_parameter.depth_points.at<Vec3d>(r, c)[i] = 0.0f;
						sl_parameter.depth_colors.at<Vec3d>(r, c)[i] =0.0f;
					}
				}
				//reject background depth points by using z-z0>background_depth_threshold
			}
		}
	return 0;
}

int RunStructuredLight(SlParameter &sl_parameter, SlCalibration &sl_calibration)
{
	clock_t clock_begin;
	Mat depth_gray_temp;
	cout << "Run structuredlight scan........" << endl<<endl;

//	GenerateGrayCode(sl_parameter);
	ReadGrayCode(sl_parameter);

//	CaptureLivingImage(sl_parameter);

	ostringstream save_name;
	int i=0;
	namedWindow("depth map", WINDOW_AUTOSIZE);
	createTrackbar("capture delay:", "depth map", &sl_parameter.project_capture_delay, 500, NULL);
	createTrackbar("contrast:", "depth map", &sl_parameter.contrast_threshold, 100, NULL);
	while (1){
		clock_begin = clock();

		RunScanObject(sl_parameter, true);
		cout << endl;
	//	ReadScanImage(sl_parameter);

		DecodeGrayCode(sl_parameter);

	//	DisplayDecodeResult(sl_parameter);

		//reconstruct depth map
		ReconstructDepthMap(sl_parameter, sl_calibration);
		
		//save pointcloud as X3D File
		//char save_name[] = ".\\output\\test\\pointCloud.x3d";
		//	SaveX3DFile(save_name, sl_parameter.depth_points, sl_parameter.depth_colors, sl_parameter.depth_valid);

		DepthMapConvertToGray(sl_parameter.depth_points, depth_gray_temp, sl_parameter.depth_valid,sl_parameter.distance_range[0],sl_parameter.distance_range[1]);
		save_name.str("");
		save_name << ".\\output\\test\\cup\\depth_map_" << i++ << ".jpg";
		imwrite(string(save_name.str()), depth_gray_temp);
		imshow("depth map", depth_gray_temp);
		cout << clock() - clock_begin<<endl<<endl;
	}
	return 0;
}






