#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <direct.h>

#include "structuredlight.h"
#include "calibration.h"
#include "auxiliaryfunctions.h"

using namespace std;
using namespace cv;

//Run camera calibration

int GenerateChessBoardPattern(SlParameter &sl_parameter)
{
	cout << "generate chessboard pattern......." << endl;
	int row_index, colum_index;
	int pixel_x, pixel_y;
	sl_parameter.projector_chessboard_pattern = Mat::zeros(sl_parameter.projector_height, sl_parameter.projector_width, CV_8U);
	sl_parameter.projector_chessboard_pattern.setTo(255);

	//calculate starting colum and row
	sl_parameter.projector_border_colum = (int)((sl_parameter.projector_width - (sl_parameter.projector_board_size.width + 1)*sl_parameter.projector_square_size.width) / 2);
	sl_parameter.projector_border_row = (int)((sl_parameter.projector_height - (sl_parameter.projector_board_size.height + 1)*sl_parameter.projector_square_size.height) / 2);

	if (sl_parameter.projector_border_colum < 0 || sl_parameter.projector_border_row < 0)
	{
		cout << "projector physical width and height:" << sl_parameter.projector_width << "X" << sl_parameter.projector_height << endl;
		cerr << "can't generate " << (sl_parameter.projector_board_size.width + 1)*sl_parameter.projector_square_size.width << "X" <<
			(sl_parameter.projector_board_size.height + 1)*sl_parameter.projector_square_size.height << "chessboard" << endl;
		return -1;
	}

	//create odd black square
	for (row_index = 0; row_index <sl_parameter.projector_board_size.height + 1; row_index += 2)
	{
		for (colum_index = 0; colum_index < sl_parameter.projector_board_size.width + 1; colum_index += 2)
		{
			for (pixel_y = 0; pixel_y < sl_parameter.projector_square_size.height; pixel_y++)
			{
				for (pixel_x = 0; pixel_x < sl_parameter.projector_square_size.width; pixel_x++)
					sl_parameter.projector_chessboard_pattern.at<unsigned char>(pixel_y + row_index*sl_parameter.projector_square_size.height + sl_parameter.projector_border_row,
					pixel_x + colum_index*sl_parameter.projector_square_size.width + 500) = 0;  //pay attention to where the chessboard start
			}
		}
	}
	//create even black square
	for (row_index = 1; row_index < sl_parameter.projector_board_size.height; row_index += 2)
	{
		for (colum_index = 1; colum_index < sl_parameter.projector_board_size.width; colum_index += 2)
		{
			for (pixel_y = 0; pixel_y < sl_parameter.projector_square_size.height; pixel_y++)
			{
				for (pixel_x = 0; pixel_x < sl_parameter.projector_square_size.width; pixel_x++)
					sl_parameter.projector_chessboard_pattern.at<unsigned char>(pixel_y + row_index*sl_parameter.projector_square_size.height + sl_parameter.projector_border_row,
					pixel_x + colum_index*sl_parameter.projector_square_size.width + 500) = 0; //pay attention to where the chessboard start
			}
		}
	}
	cout << "generate chessboard pattern successful......." << endl;
}

int RunCameraCalibration(SlParameter &sl_parameter, SlCalibration &sl_calibration)
{
	//reset camera_intrinsic_calibration_flag
	sl_calibration.camera_intrinsic_calibration_flag = false;

	//create camera calibration directory
	cout << "Create camera calibration directory..................." << endl;
	_mkdir((sl_parameter.output_directory + "\\" + "calibration").c_str());
	_mkdir((sl_parameter.output_directory + "\\" + "calibration" + "\\" + "camera").c_str());
	//clean existing object_name directory and then recreate
	//string str = "rd /s /q \"" + sl_parameter.output_directory + "\\" + "calibration" +"\\"+ "camera" + "\"";
	//system(str.c_str());
	//if(_mkdir((sl_parameter.output_directory + "\\" + "calibration"+"\\"+ "camera").c_str()) != 0)
	//{
	//	cerr << "Can't open " << sl_parameter.output_directory + "\\" + "calibration" + "\\" + "camera" << endl;
	//	return -1;
	//}

	//show input-video from camera,corners-detected,image amount graped in real time
	//press 'o' to grap the desired image,'e' or images amount achieved frame_amount to exit
	cout << "Prepare to calibrate camera................" << endl;
	cout << "press 'o' to grap the desired image" << endl;
	cout << "press 'e' or images amount achieved frame_amount to exit" << endl;

	int frame_grap_count = 0;
	cv::Mat frame_grab;
	cv::Mat desired_image;
	vector<Point2f> image_points_buffer;			//array of detected points corners.
	vector<vector<Point2f> > image_points_array;	//point to image_points_buffer
	vector<Point3f>object_points_buffer;			//array of object points corners
	vector<vector<Point3f> > object_points_array;	//point to object_points_buffer					
	vector<Mat>  camera_rotation_vectors;			//output vector of translation vectors estimated for each pattern view.
	vector<Mat>  camera_translation_vectors;		//output vector of rotation vectors
	bool corners_found_result = false;
	char key = 'n';
	namedWindow("Grap desired image", WINDOW_AUTOSIZE);
	ostringstream str_temp;

	while (key != 'e' && (frame_grap_count<sl_parameter.frame_amount))
	{
		GetImage(frame_grab);
		frame_grab.copyTo(desired_image);
		corners_found_result = findChessboardCorners(frame_grab, sl_parameter.camera_board_size, image_points_buffer,
			CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);

		if (corners_found_result)
		{
			Mat view_temp;
			cvtColor(desired_image, view_temp, COLOR_BGR2GRAY);
			cornerSubPix(view_temp, image_points_buffer, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
			drawChessboardCorners(desired_image, sl_parameter.camera_board_size, image_points_buffer, corners_found_result);
		}
		//show result whether detected corners or not
		imshow("Grap desired image", desired_image);

		key = waitKey(200);
		if (key == 'o')			//save desired image	
		{
			key = 'n';
			frame_grap_count++;
			str_temp.str("");		//clean out stringstream
			str_temp << ".\\output\\calibration\\camera\\image_" << frame_grap_count << ".jpg";
			imwrite(string(str_temp.str()), frame_grab);
			str_temp.str("");		//clean out stringstream
			str_temp << ".\\output\\calibration\\camera\\image_corners_" << frame_grap_count << ".jpg";
			imwrite(string(str_temp.str()), desired_image);
			cout << "frame_grap_count:  " << frame_grap_count << '\r';
		}
	}
	destroyWindow("Grap desired image");

	if (key == 'e')
		cout << "stop grap image from camera........" << endl << endl;
	else
		cout << "grap desired imge up to " << 1 << endl;
	cout << "start to find ChessboardCorners......." << endl;
	for (frame_grap_count = 1; frame_grap_count <= sl_parameter.frame_amount; frame_grap_count++)
	{
		str_temp.str("");		//clean out stringstream
		str_temp << ".\\output\\calibration\\camera\\image_" << frame_grap_count << ".jpg";
		desired_image = imread(string(str_temp.str()));
		corners_found_result = findChessboardCorners(desired_image, sl_parameter.camera_board_size, image_points_buffer,
			CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
		if (corners_found_result)	//improve the found corners' coordinate accuracy for chessboard
		{
			Mat view_temp;
			cvtColor(desired_image, view_temp, COLOR_BGR2GRAY);
			cornerSubPix(view_temp, image_points_buffer, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
			image_points_array.push_back(image_points_buffer);
		}
		else
		{
			cout << "find chessboard corners error at image_" << frame_grap_count <<endl;
			return -1;
		}
	}

	//calculate board corners position
	int i, j;
	for (i = 0; i < sl_parameter.camera_board_size.height; i++)
		for (j = 0; j < sl_parameter.camera_board_size.width; j++)
			object_points_buffer.push_back(Point3f(j*sl_parameter.camera_square_size.width, i*sl_parameter.camera_square_size.height, 0));
	object_points_array.resize(image_points_array.size(), object_points_buffer);

	//run camera calibration
	cout << "start to run camera calibration......." << endl;
	sl_calibration.camera_intrinsic = Mat::eye(3, 3, CV_64F);
	sl_calibration.camera_distortion = Mat::ones(8, 1, CV_64F);
	double rms = calibrateCamera(object_points_array, image_points_array, Size(sl_parameter.camera_width, sl_parameter.camera_height), sl_calibration.camera_intrinsic, sl_calibration.camera_distortion,
		camera_rotation_vectors, camera_translation_vectors, CV_CALIB_FIX_PRINCIPAL_POINT | CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5);

	cout << "camera intrinsic matrix: " << endl << sl_calibration.camera_intrinsic << endl;
	cout << "camera distortion cofficients: " << endl << sl_calibration.camera_distortion << endl;
	cout << "reprojection error reported by calibrate_camera: " << rms << endl;
#ifdef DEBUG_PROJECT
	for (i = 0; i < camera_rotation_vectors.size(); i++)
	{
		cout << "camera rotation vectors_" << i << ": " << camera_rotation_vectors[i] << endl;
		cout << "camera translation vectors_" << i << ": " << camera_translation_vectors[i] << endl;
	}
#endif

	//save camera intrinsic parameters
	FileStorage slcalibration_file(".\\output\\calibration\\camera\\slcalibration.xml", FileStorage::WRITE);
	if (!slcalibration_file.isOpened())
	{
		cout << "can't open slcalibration.xml!" << endl;
		return -1;
	}

	slcalibration_file << "CameraIntrinsicMatrix" << sl_calibration.camera_intrinsic;
	slcalibration_file << "CameraDistortionCofficients" << sl_calibration.camera_distortion;
	for (i = 0; i < camera_rotation_vectors.size(); i++)
	{
		str_temp.str("");		//clean out stringstream
		str_temp << "CameraRotationVectors_" << i;
		slcalibration_file << string(str_temp.str()) << camera_rotation_vectors[i];
		str_temp.str("");		//clean out stringstream
		str_temp << "CameraTranslationVectors_" << i;
		slcalibration_file << string(str_temp.str()) << camera_translation_vectors[i];
	}
	slcalibration_file.release();		//must release file

	cout << "saving camera intrinsic parameters......" << endl;
	cout << "camera calibration was successful......" << endl;

	//set camera_intrinsic_calibration_flag
	sl_calibration.camera_intrinsic_calibration_flag = true;

#ifdef DEBUG_PROJECT
	//show the distortion images and undistortion images
	namedWindow("distortion image", WINDOW_AUTOSIZE);
	namedWindow("undistortion image", WINDOW_AUTOSIZE);
	Mat distortion_image, undistortion_image;
	for (frame_grap_count = 1; frame_grap_count <= sl_parameter.frame_amount; frame_grap_count++)
	{
		str_temp.str("");		//clean out stringstream
		str_temp << ".\\output\\calibration\\camera\\image_" << frame_grap_count << ".jpg";
		distortion_image = imread(string(str_temp.str()));
		undistort(distortion_image, undistortion_image, sl_calibration.camera_intrinsic, sl_calibration.camera_distortion);
		imshow("distortion image", distortion_image);
		imshow("undistortion image", undistortion_image);
		str_temp.str("");		//clean out stringstream
		str_temp << ".\\output\\calibration\\camera\\undistortion_image_" << frame_grap_count << ".jpg";
		imwrite(string(str_temp.str()), undistortion_image);
		waitKey(2000);
	}
#endif
	destroyWindow("distortion image");
	destroyWindow("undistortion image");
	return 0;
}

int RunProjectorCalibration(SlParameter &sl_parameter, SlCalibration &sl_calibration)
{
	//reset camera and projector_intrinsic_calibration_flag
	sl_calibration.camera_intrinsic_calibration_flag = false;
	sl_calibration.projector_intrinsic_calibration_flag = false;

	//create projector calibration directory
	cout << "Create projector calibration directory......" << endl;
	_mkdir((sl_parameter.output_directory + "\\" + "calibration").c_str());
	_mkdir((sl_parameter.output_directory + "\\" + "calibration" + "\\" + "projector").c_str());
	//clean existing object_name directory and then recreate
	//string str = "rd /s /q \"" + sl_parameter.output_directory + "\\" + "calibration" +"\\"+ "projector" + "\"";
	//system(str.c_str());
	//if(_mkdir((sl_parameter.output_directory + "\\" + "calibration"+"\\"+ "projector").c_str()) != 0)
	//{
	//	cerr << "Can't open " << sl_parameter.output_directory + "\\" + "calibration" + "\\" + "projector" << endl;
	//	return -1;
	//}

	//generate chessboard pattern 
	GenerateChessBoardPattern(sl_parameter);

	int frame_grap_count = 0;
	bool corners_found_result = false;
	int camera_corners_amount = sl_parameter.camera_board_size.width*sl_parameter.camera_board_size.height;
	int projector_corners_amount = sl_parameter.projector_board_size.width*sl_parameter.projector_board_size.height;
	cv::Mat frame_grab(sl_parameter.camera_height,sl_parameter.camera_width,CV_8UC3,Scalar(0));
	cv::Mat camera_desired_image1(sl_parameter.camera_height, sl_parameter.camera_width, CV_8UC3, Scalar(0));									//use for raw image
	cv::Mat camera_desired_image2(sl_parameter.camera_height, sl_parameter.camera_width, CV_8UC3, Scalar(0));									//use for drawing corners
	cv::Mat camera_desired_image_gray(sl_parameter.camera_height, sl_parameter.camera_width, CV_8UC1, Scalar(0));								//use for raw gray image		
	vector<Point2f> camera_distortion_image_points_buffer(camera_corners_amount);			//array of detected camera image corners
	vector<Point2f> camera_undistortion_image_points_buffer(camera_corners_amount);		//use for finding homography mapping image_corners to object_corners  
	vector<vector<Point2f> > camera_distortion_image_points_array(sl_parameter.frame_amount);	//point to detected camera image corners buffer
	vector<vector<Point2f> > camera_undistortion_image_points_array(sl_parameter.frame_amount);//use for finding homography mapping image_corners to object_corners 
	vector<Point3f>camera_object_points_buffer(camera_corners_amount);			//array of camera object points corners
	vector<vector<Point3f> >camera_object_points_array(sl_parameter.frame_amount);	//point to camera object points_buffer					
	vector<Mat>  camera_rotation_vectors(sl_parameter.frame_amount);				//output vector of translation vectors estimated for each pattern view.
	vector<Mat>  camera_translation_vectors(sl_parameter.frame_amount);				//output vector of rotation vectors
	
	cv::Mat projector_desired_image1(sl_parameter.camera_height, sl_parameter.camera_width, CV_8UC3, Scalar(0));					//use for raw image
	cv::Mat projector_desired_image2(sl_parameter.camera_height, sl_parameter.camera_width, CV_8UC3, Scalar(0));					//use for drawing corners
	cv::Mat projector_desired_image_gray1(sl_parameter.camera_height, sl_parameter.camera_width, CV_8UC1, Scalar(0));				//use for raw gray image
	cv::Mat projector_desired_image_gray2(sl_parameter.camera_height, sl_parameter.camera_width, CV_8UC1, Scalar(0));				//use for subtraction gray image
	cv::Mat projector_desired_image_gray3(sl_parameter.camera_height, sl_parameter.camera_width, CV_8UC1, Scalar(0));				//use for subtraction gray image invert
	vector<Point2f> projector_distortion_image_points_buffer(projector_corners_amount);				//array of detected projector image corners
	vector<Point2f> projector_undistortion_image_points_buffer(projector_corners_amount);			//use for finding homography mapping image_corners to object_corners  
	vector<vector<Point2f> > projector_distortion_image_points_array(sl_parameter.frame_amount);	//point to projector image corners buffer
	vector<vector<Point2f> > projector_undistortion_image_points_array(sl_parameter.frame_amount);	//use for finding homography mapping image_corners to object_corners  
	vector<Point3f>projector_object_points_buffer(projector_corners_amount);			//array of projector object points corners
	vector<vector<Point3f> >projector_object_points_array(sl_parameter.frame_amount);	//point to projector object points_buffer					
	vector<Mat>projector_rotation_vectors(sl_parameter.frame_amount);					//output vector of translation vectors estimated for each pattern view.
	vector<Mat>projector_translation_vectors(sl_parameter.frame_amount);				//output vector of rotation vectors

	vector<Point2f>projector_image_points_buffer(projector_corners_amount);			//raw image corners for projectoring
	vector<vector<Point2f>>projector_image_points_array(sl_parameter.frame_amount);  //point to raw image corners for projectoring

	char key = 'n';
	ostringstream str_temp;

	//show input-video from camera,corners-detected,image amount graped in real time
	//press 'o' to grap the desired image,'e' or images amount achieved frame_amount to exit
	cout << "Prepare to calibrate projector........" << endl;
	cout << "press 'o' to grap the desired image" << endl;
	cout << "press 'e' or images amount achieved frame_amount to exit" << endl;
	
	//show background pattern firstly
	imshow("projector window", sl_parameter.projector_background_pattern);
	namedWindow("camera desired image", WINDOW_NORMAL);
	resizeWindow("camera desired image", 800, 600);
	moveWindow("camera desired image", 50, 50);
	

	namedWindow("projector desired image", WINDOW_NORMAL);
	resizeWindow("projector desired image", 800, 600);
	moveWindow("projector desired image", 900, 50);

	key = waitKey();
	while ((key != 'e')&&(key!='E')&& (frame_grap_count<sl_parameter.frame_amount))
	{
		if (GetImage(frame_grab))
		{
			waitKey(sl_parameter.project_capture_delay);
			GetImage(frame_grab);
			frame_grab.copyTo(camera_desired_image1);
			frame_grab.copyTo(camera_desired_image2);
		}
		corners_found_result = findChessboardCorners(camera_desired_image1, sl_parameter.camera_board_size, camera_distortion_image_points_buffer,

			CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
		//If find chessboard's corners ,then display projector's chessboard and find corners
		if (corners_found_result)			
		{
			drawChessboardCorners(camera_desired_image2, sl_parameter.camera_board_size, camera_distortion_image_points_buffer, corners_found_result);
			imshow("camera desired image", camera_desired_image2);
			imshow("projector window", sl_parameter.projector_chessboard_pattern);
			waitKey(1500);
			//Get image including projecting chessboard
			if (GetImage(frame_grab))
			{
				waitKey(sl_parameter.project_capture_delay);
				GetImage(frame_grab);
				frame_grab.copyTo(projector_desired_image1);
				frame_grab.copyTo(projector_desired_image2);
			}
			//convert color image to gray image,and get subtraction gray image
			cvtColor(camera_desired_image1,camera_desired_image_gray, COLOR_RGB2GRAY);
			cvtColor(projector_desired_image1, projector_desired_image_gray1, COLOR_RGB2GRAY);
			subtract(camera_desired_image_gray,projector_desired_image_gray1, projector_desired_image_gray2);

			//invert subtraction projector chessboard pattern
			double min_val = 0, max_val = 0;
			projector_desired_image_gray2.copyTo(projector_desired_image_gray3);
			minMaxLoc(projector_desired_image_gray2,&min_val,&max_val);
			for (int i = 0; i < projector_desired_image_gray2.rows; i++)
				for (int j = 0; j < projector_desired_image_gray2.cols; j++)
				{
					projector_desired_image_gray3.at<unsigned char>(i, j) = projector_desired_image_gray2.at<unsigned char>(i, j)*(-255 / (max_val - min_val))
					+ 255 + 255 * min_val / (max_val - min_val);
				}

			//High-pass filtering to enhance edge
			Mat kernel(3, 3, CV_32F, Scalar(-1));
			kernel.at<float>(1, 1) = 8.9;
			filter2D(projector_desired_image_gray3, projector_desired_image_gray3, projector_desired_image_gray3.depth(),kernel);

			//find projector chessboard corners
			corners_found_result = findChessboardCorners(projector_desired_image_gray3, sl_parameter.projector_board_size, projector_distortion_image_points_buffer,
				CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
			if (corners_found_result)
			{
				drawChessboardCorners(projector_desired_image2, sl_parameter.projector_board_size, projector_distortion_image_points_buffer, corners_found_result);
				imshow("projector desired image", projector_desired_image2);
				key = waitKey(3000);
				if (key == 'o')			//save desired image	
				{
					key = 'n';
					str_temp.str("");		//clean out stringstream
					str_temp << ".\\output\\calibration\\projector\\camera_image_" << frame_grap_count << ".jpg";
					imwrite(string(str_temp.str()), camera_desired_image1);
					str_temp.str("");		//clean out stringstream
					str_temp << ".\\output\\calibration\\projector\\camera_image_corners_" << frame_grap_count << ".jpg";
					imwrite(string(str_temp.str()), camera_desired_image2);
					str_temp.str("");		//clean out stringstream
					str_temp << ".\\output\\calibration\\projector\\projector_image_" << frame_grap_count << ".jpg";
					imwrite(string(str_temp.str()), projector_desired_image1);
					str_temp.str("");		//clean out stringstream
					str_temp << ".\\output\\calibration\\projector\\projector_image_corners_" << frame_grap_count << ".jpg";
					imwrite(string(str_temp.str()), projector_desired_image2);
					str_temp.str("");		//clean out stringstream
					str_temp << ".\\output\\calibration\\projector\\projector_sub_image_gray_" << frame_grap_count << ".jpg";
					imwrite(string(str_temp.str()), projector_desired_image_gray3);

					//save detected distortion image corners coordinates
					//improve the found corners' coordinate accuracy for chessboard 
					cornerSubPix(camera_desired_image_gray, camera_distortion_image_points_buffer, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
					cornerSubPix(projector_desired_image_gray3, projector_distortion_image_points_buffer, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
					
					camera_distortion_image_points_array[frame_grap_count]=camera_distortion_image_points_buffer;
					projector_distortion_image_points_array[frame_grap_count]=projector_distortion_image_points_buffer;

					if ((++frame_grap_count) == sl_parameter.frame_amount - 1)
						cout << "please fix projector board to get projector-camera extrinsic parmeters" << endl;
					cout << "grap " << frame_grap_count << " frame of " << sl_parameter.frame_amount << endl;
				}	
			}	
		}

		//show raw iamge if corners detected not
		imshow("camera desired image", camera_desired_image1);
		imshow("projector desired image", projector_desired_image1);

		//show projector background pattern for next frame
		imshow("projector window", sl_parameter.projector_background_pattern);
		key = waitKey(1500);	
	}

	if ((key == 'e')||(key=='E'))
		cout << "stop grap image from camera........" << endl << endl;

	//save detected distortion image corners coordinates
	if (frame_grap_count == sl_parameter.frame_amount)
	{
		FileStorage slcalibration_corners_file(".\\output\\calibration\\projector\\slcalibration_corners.xml", FileStorage::WRITE);
		if (!slcalibration_corners_file.isOpened())
		{
			cout << "can't open slcalibration_corners.xml!" << endl;
			return -1;
		}
		for (int i = 0; i < sl_parameter.frame_amount; ++i)
		{
			str_temp.str("");
			str_temp << "camera_distortion_image_points_" << i;
			slcalibration_corners_file<<string(str_temp.str()) << camera_distortion_image_points_array[i];
			str_temp.str("");
			str_temp << "projector_distortion_image_points_" << i;
			slcalibration_corners_file << string(str_temp.str()) << projector_distortion_image_points_array[i];
		}
		slcalibration_corners_file.release();
		cout << "grap desired imge up to " << sl_parameter.frame_amount << endl << endl;
	}
	
	destroyWindow("camera desired image");
	destroyWindow("projector desired image");

	//Calibrate camera 
	//read camera chessboard corners
	cout << "start to calibrate camera" << endl;
	cout << "read camera chessboard corners......." << endl;
	FileStorage file_temp1(".\\output\\calibration\\projector\\slcalibration_corners.xml", FileStorage::READ);
	if (!file_temp1.isOpened())
	{
		cout << "can't open slcalibration_corners.xml" << endl;
		return -1;
	}
	for (int i= 0; i<sl_parameter.frame_amount; ++i)
	{
		str_temp.str("");
		str_temp << "camera_distortion_image_points_" << i;
		file_temp1[string(str_temp.str())] >> camera_distortion_image_points_array[i];
	}
	file_temp1.release();

	//calculate camera object corners position
	for (int i = 0; i < sl_parameter.camera_board_size.height; ++i)
		for (int j = 0; j < sl_parameter.camera_board_size.width; ++j)
			camera_object_points_buffer[i*sl_parameter.camera_board_size.width+j]=(Point3f(j*sl_parameter.camera_square_size.width, i*sl_parameter.camera_square_size.height, 0));
	for (int i = 0; i < sl_parameter.frame_amount;++i)
		camera_object_points_array[i]=camera_object_points_buffer;

	//run camera calibration
	cout << "start to run camera calibration......." << endl;
	sl_calibration.camera_intrinsic = Mat::eye(3, 3, CV_64F);
	sl_calibration.camera_distortion = Mat::ones(8, 1, CV_64F);
	double rms= calibrateCamera(camera_object_points_array, camera_distortion_image_points_array, Size(sl_parameter.camera_width, sl_parameter.camera_height), sl_calibration.camera_intrinsic, sl_calibration.camera_distortion,
		camera_rotation_vectors, camera_translation_vectors, CV_CALIB_FIX_PRINCIPAL_POINT |CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5);

		
#ifdef DEBUG_PROJECT
	//show and save distortion images and undistortion images
	namedWindow("camera distortion image", WINDOW_AUTOSIZE);
	namedWindow("camera undistortion image", WINDOW_AUTOSIZE);
	cout << "camera intrinsic matrix: " << endl << sl_calibration.camera_intrinsic << endl;
	cout << "camera distortion cofficients: " << endl << sl_calibration.camera_distortion << endl;
	cout << "reprojection error reported by calibrate_camera: " << rms << endl;
	for (int i = 0; i < camera_rotation_vectors.size(); i++)
	{
		cout << "camera rotation vectors_" << i << ": " << camera_rotation_vectors[i] << endl;
		cout << "camera translation vectors_" << i << ": " << camera_translation_vectors[i] << endl;
	}
	Mat camera_distortion_image, camera_undistortion_image;
	for (int image_count = 0; image_count <sl_parameter.frame_amount; ++image_count)
	{
		str_temp.str("");		//clean out stringstream
		str_temp << ".\\output\\calibration\\projector\\camera_image_" << image_count << ".jpg";
		camera_distortion_image = imread(string(str_temp.str()));
		undistort(camera_distortion_image, camera_undistortion_image, sl_calibration.camera_intrinsic, sl_calibration.camera_distortion);
		imshow("camera distortion image", camera_distortion_image);
		imshow("camera undistortion image", camera_undistortion_image);
		str_temp.str("");		//clean out stringstream
		str_temp << ".\\output\\calibration\\projector\\camera_image_" << image_count << "_undist.jpg";
		imwrite(string(str_temp.str()), camera_undistortion_image);
		waitKey(500);
	}
	destroyWindow("camera distortion image");
	destroyWindow("camera undistortion image");
#endif


	//Calibrate projector
	cout << "start to calibrate projector" << endl;
	cout << "read projector chessboard corners......." << endl;
	FileStorage file_temp2(".\\output\\calibration\\projector\\slcalibration_corners.xml", FileStorage::READ);
	if (!file_temp2.isOpened())
	{
		cout << "can't open slcalibration_corners.xml!" << endl;
		return -1;
	}
	for (int i = 0; i<sl_parameter.frame_amount; ++i)
	{
		str_temp.str("");
		str_temp << "projector_distortion_image_points_" << i;
		file_temp2[string(str_temp.str())] >> projector_distortion_image_points_array[i];
	}
	file_temp2.release();

	//calculate projector image points
	cout << "calculate projector image points....." << endl;
	for (int i = 0; i < sl_parameter.projector_board_size.height; ++i)
		for (int j = 0; j < sl_parameter.projector_board_size.width; ++j)
		{
			projector_image_points_buffer[i*sl_parameter.projector_board_size.width + j].x = j*sl_parameter.projector_square_size.width + 500 + sl_parameter.projector_square_size.width - 0.5f;
			projector_image_points_buffer[i*sl_parameter.projector_board_size.width + j].y = i*sl_parameter.projector_square_size.height + sl_parameter.projector_border_row + sl_parameter.projector_square_size.height - 0.5f;
		}
	for (int i = 0; i < sl_parameter.frame_amount; ++i)
		projector_image_points_array[i] = projector_image_points_buffer;

	// Estimate homography that maps undistorted image pixels to positions on the chessboard,then use perspectiveTransform to get projector object coordinate£¨under camera's world coordinate£©
	cout << "calculate projector object corners from calibrated camera......" <<endl;
	vector<Point2f> camera_desttination(sl_parameter.camera_board_size.width*sl_parameter.camera_board_size.height);
	vector<Point2f> projector_destination(sl_parameter.camera_board_size.width*sl_parameter.camera_board_size.height);
	Mat homography = Mat::zeros(3, 3, CV_64FC1);
	//initialize projector object corners 
	projector_object_points_array = camera_object_points_array;
	for (int image_count=0; image_count < sl_parameter.frame_amount; ++image_count)
	{
		undistortPoints(camera_distortion_image_points_array[image_count], camera_undistortion_image_points_array[image_count], sl_calibration.camera_intrinsic, sl_calibration.camera_distortion);
		undistortPoints(projector_distortion_image_points_array[image_count], projector_undistortion_image_points_array[image_count], sl_calibration.camera_intrinsic, sl_calibration.camera_distortion);
		//get camera_destination(x,y)(unit: mm) from camera_object(x,y,z)(unit: mm)
		for (int i = 0; i < sl_parameter.camera_board_size.width*sl_parameter.camera_board_size.height; ++i)
		{
			camera_desttination[i].x = camera_object_points_array[image_count][i].x;
			camera_desttination[i].y = camera_object_points_array[image_count][i].y;
		}
		homography = findHomography(camera_undistortion_image_points_array[image_count], camera_desttination);
		perspectiveTransform(projector_undistortion_image_points_array[image_count], projector_destination, homography);

		//get projector_object(x,y,z)(unit: mm) from projector_destination(x,y)(unit: mm)
		for (int i = 0; i < sl_parameter.camera_board_size.width*sl_parameter.camera_board_size.height; i++)
		{
			projector_object_points_array[image_count][i].x = projector_destination[i].x;
			projector_object_points_array[image_count][i].y = projector_destination[i].y;
			projector_object_points_array[image_count][i].z = 0.0f;
		}
	}
	//run projector calibration
	cout << "start to run projector calibration......" << endl;
	sl_calibration.projector_intrinsic = Mat::eye(3, 3, CV_64F);
	sl_calibration.projector_distortion = Mat::ones(8, 1, CV_64F);
	rms= calibrateCamera(projector_object_points_array, projector_image_points_array, Size(sl_parameter.projector_width, sl_parameter.projector_height), sl_calibration.projector_intrinsic, sl_calibration.projector_distortion,
		projector_rotation_vectors, projector_translation_vectors, CV_CALIB_FIX_PRINCIPAL_POINT | CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5);

	//save camera and projector intrinsic parameters
	sl_calibration.camera_intrinsic_calibration_flag = true;
	sl_calibration.projector_intrinsic_calibration_flag = true;
	FileStorage file_temp3(".\\output\\calibration\\projector\\slcalibration.xml", FileStorage::WRITE);
	if (!file_temp3.isOpened())
	{
		cout << "can't open slcalibration.xml!" << endl;
		return -1;
	}
	file_temp3 << "CameraIntrinsicMatrix" << sl_calibration.camera_intrinsic;
	file_temp3 << "CameraDistortionCofficients" << sl_calibration.camera_distortion;
	file_temp3 << "ProjectorIntrinsicMatrix" << sl_calibration.projector_intrinsic;
	file_temp3 << "ProjectorDistortionCofficients" << sl_calibration.projector_distortion;
	file_temp3 << "CameraIntrinsicCalibrationFlag" << sl_calibration.camera_intrinsic_calibration_flag;
	file_temp3 << "ProjectorIntrinsicCalibrationFlag" << sl_calibration.projector_intrinsic_calibration_flag;
	for (int i = 0; i < camera_rotation_vectors.size(); i++)
	{
		str_temp.str("");		//clean out stringstream
		str_temp << "CameraRotationVectors_" << i;
		file_temp3 << string(str_temp.str()) << camera_rotation_vectors[i];
		str_temp.str("");		//clean out stringstream
		str_temp << "CameraTranslationVectors_" << i;
		file_temp3 << string(str_temp.str()) << camera_translation_vectors[i];
	}
	for (int i = 0; i < projector_rotation_vectors.size(); i++)
	{
		str_temp.str("");		//clean out stringstream
		str_temp << "ProjectorRotationVectors_" << i;
		file_temp3 << string(str_temp.str()) << projector_rotation_vectors[i];
		str_temp.str("");		//clean out stringstream
		str_temp << "ProjectorTranslationVectors_" << i;
		file_temp3 << string(str_temp.str()) << projector_translation_vectors[i];
	}
	file_temp3.release();		//must release file
	cout << "saving camera intrinsic parameters......" << endl;
	cout << "camera calibration was successful......" << endl << endl;
	cout << "saving projector intrinsic parameters......" << endl;
	cout << "projector calibration was successful......" << endl << endl;

#ifdef DEBUG_PROJECT
	//show save  distortion images and undistortion images
	namedWindow("projector distortion image", WINDOW_AUTOSIZE);
	cout << "projector intrinsic matrix: " << endl << sl_calibration.projector_intrinsic << endl;
	cout << "projector distortion cofficients: " << endl << sl_calibration.projector_distortion << endl;
	cout << "reprojection error reported by calibrate_camera: " << rms << endl;
	for (int i = 0; i < projector_rotation_vectors.size(); i++)
	{
		cout << "projector rotation vectors_" << i << ": " << projector_rotation_vectors[i] << endl;
		cout << "projector translation vectors_" << i << ": " << projector_translation_vectors[i] << endl;
	}
	Mat projector_undistortion_image;
	undistort(sl_parameter.projector_chessboard_pattern,projector_undistortion_image, sl_calibration.projector_intrinsic, sl_calibration.projector_distortion);
	imshow("projector distortion image", sl_parameter.projector_chessboard_pattern);
	imshow("projector window", sl_parameter.projector_chessboard_pattern);
	str_temp.str("");		//clean out stringstream
	str_temp << ".\\output\\calibration\\projector\\projector_chessboard_pattern.jpg";
	imwrite(string(str_temp.str()), sl_parameter.projector_chessboard_pattern);
	str_temp.str("");		//clean out stringstream
	str_temp << ".\\output\\calibration\\projector\\projector_chessboard_pattern_undist.jpg";
	imwrite(string(str_temp.str()), projector_undistortion_image);

	imshow("projector window", sl_parameter.projector_chessboard_pattern);
	waitKey(2000);
	imshow("projector window", projector_undistortion_image);
	waitKey(5000);
	imshow("projector window", sl_parameter.projector_background_pattern);
	destroyWindow("projector distortion image");
	cout << endl;
#endif
	return 0;
}

int RunCameraProjectorExtrinsicCalibration(SlParameter &sl_parameter, SlCalibration &sl_calibration)
{
	//reset camera_projector_extrinsic_calibration flag;
	sl_calibration.camera_projector_extrinsic_calibration_flag= false;
	//check camera_intrinsic_calibration_flag and projector_intrinsic_calibration_flag
	if (!sl_calibration.camera_intrinsic_calibration_flag || !sl_calibration.projector_intrinsic_calibration_flag)
	{
		cout << "please calibrate camera and projector first" << endl<<endl;
		return -1;
	}
	//create camera_projector_extrinsic calibration directory
	cout << "Create  camera_projector_extrinsic calibration directory.............." << endl;
	_mkdir((sl_parameter.output_directory + "\\" + "calibration").c_str());
	_mkdir((sl_parameter.output_directory + "\\" + "calibration" + "\\" + "extrinsic").c_str());
	//clean existing object_name directory and then recreate
	//string str = "rd /s /q \"" + sl_parameter.output_directory + "\\" + "calibration" +"\\"+ "extrinsic" + "\"";
	//system(str.c_str());
	//if(_mkdir((sl_parameter.output_directory + "\\" + "calibration"+"\\"+ "extrinsic").c_str()) != 0)
	//{
	//	cerr << "Can't open " << sl_parameter.output_directory + "\\" + "calibration" + "\\" + "extrinsic" << endl;
	//	return -1;
	//}

	//generate chessboard pattern 
	GenerateChessBoardPattern(sl_parameter);

	int frame_grap_count = 0;
	bool corners_found_result = false;
	int camera_corners_amount = sl_parameter.camera_board_size.width*sl_parameter.camera_board_size.height;
	int projector_corners_amount = sl_parameter.projector_board_size.width*sl_parameter.projector_board_size.height;
	cv::Mat frame_grab(sl_parameter.camera_height, sl_parameter.camera_width, CV_8UC3);
	cv::Mat camera_desired_image1(sl_parameter.camera_height, sl_parameter.camera_width, CV_8UC3);									//use for raw image
	cv::Mat camera_desired_image2(sl_parameter.camera_height, sl_parameter.camera_width, CV_8UC3);									//use for drawing corners
	cv::Mat camera_desired_image_gray(sl_parameter.camera_height, sl_parameter.camera_width, CV_8UC1);								//use for raw gray image		
	vector<Point2f> camera_distortion_image_points_buffer(camera_corners_amount);			//array of detected camera image corners
	vector<Point2f> camera_undistortion_image_points_buffer(camera_corners_amount);		//use for finding homography mapping image_corners to object_corners  
	vector<vector<Point2f> > camera_distortion_image_points_array(1);	//point to detected camera image corners buffer
	vector<vector<Point2f> > camera_undistortion_image_points_array(1);//use for finding homography mapping image_corners to object_corners 
	vector<Point3f>camera_object_points_buffer(camera_corners_amount);			//array of camera object points corners
	vector<vector<Point3f> >camera_object_points_array(1);	//point to camera object points_buffer					


	cv::Mat projector_desired_image1(sl_parameter.camera_height, sl_parameter.camera_width, CV_8UC3);					//use for raw image
	cv::Mat projector_desired_image2(sl_parameter.camera_height, sl_parameter.camera_width, CV_8UC3);					//use for drawing corners
	cv::Mat projector_desired_image_gray1(sl_parameter.camera_height, sl_parameter.camera_width, CV_8UC1);				//use for raw gray image
	cv::Mat projector_desired_image_gray2(sl_parameter.camera_height, sl_parameter.camera_width, CV_8UC1);				//use for subtraction gray image
	cv::Mat projector_desired_image_gray3(sl_parameter.camera_height, sl_parameter.camera_width, CV_8UC1);				//use for subtraction gray image invert
	vector<Point2f> projector_distortion_image_points_buffer(projector_corners_amount);				//array of detected projector image corners
	vector<Point2f> projector_undistortion_image_points_buffer(projector_corners_amount);			//use for finding homography mapping image_corners to object_corners  
	vector<vector<Point2f> > projector_distortion_image_points_array(1);	//point to projector image corners buffer
	vector<vector<Point2f> > projector_undistortion_image_points_array(1);	//use for finding homography mapping image_corners to object_corners  
	vector<Point3f>projector_object_points_buffer(projector_corners_amount);			//array of projector object points corners
	vector<vector<Point3f> >projector_object_points_array(1);	//point to projector object points_buffer					


	vector<Point2f>projector_image_points_buffer(projector_corners_amount);			//raw image corners for projectoring
	vector<vector<Point2f>>projector_image_points_array(1);  //point to raw image corners for projectoring

	char key = 'n';
	ostringstream str_temp;

	//show input-video from camera,corners-detected,image amount graped in real time
	//press 'o' to grap the desired image
	cout << "please fix camera_projector system....." << endl;
	cout << "press 'o' to grap the desired image....." << endl;

	//show background pattern firstly
	imshow("projector window", sl_parameter.projector_background_pattern);
	namedWindow("camera desired image", WINDOW_NORMAL);
	resizeWindow("camera desired image", 800, 600);
	moveWindow("camera desired image", 50, 50);

	namedWindow("projector desired image", WINDOW_NORMAL);
	resizeWindow("projector desired image", 800, 600);
	moveWindow("projector desired image", 900, 50);

	waitKey(3000);
	key = 'n';
	while (frame_grap_count<1)
	{
		if (GetImage(frame_grab))
		{
			frame_grab.copyTo(camera_desired_image1);
			frame_grab.copyTo(camera_desired_image2);
		}
		corners_found_result = findChessboardCorners(camera_desired_image1, sl_parameter.camera_board_size, camera_distortion_image_points_buffer,
			CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
		//If find chessboard's corners ,then display projector's chessboard and find corners
		if (corners_found_result)
		{
			drawChessboardCorners(camera_desired_image2, sl_parameter.camera_board_size, camera_distortion_image_points_buffer, corners_found_result);
			imshow("camera desired image", camera_desired_image2);
			imshow("projector window", sl_parameter.projector_chessboard_pattern);
			waitKey(1500);
			//Get image including projecting chessboard
			if (GetImage(frame_grab))
			{
				frame_grab.copyTo(projector_desired_image1);
				frame_grab.copyTo(projector_desired_image2);
			}
			//convert color image to gray image,and get subtraction gray image
			cvtColor(camera_desired_image1, camera_desired_image_gray, COLOR_RGB2GRAY);
			cvtColor(projector_desired_image1, projector_desired_image_gray1, COLOR_RGB2GRAY);
			subtract(camera_desired_image_gray, projector_desired_image_gray1, projector_desired_image_gray2);
			//invert subtraction projector chessboard pattern
			double min_val = 0, max_val = 0;
			projector_desired_image_gray2.copyTo(projector_desired_image_gray3);
			minMaxLoc(projector_desired_image_gray2, &min_val, &max_val);
			for (int i = 0; i < projector_desired_image_gray2.rows; i++)
				for (int j = 0; j < projector_desired_image_gray2.cols; j++)
				{
				projector_desired_image_gray3.at<unsigned char>(i, j) = projector_desired_image_gray2.at<unsigned char>(i, j)*(-255 / (max_val - min_val))
					+ 255 + 255 * min_val / (max_val - min_val);
				}

			//High-pass filtering to enhance edge
			Mat kernel(3, 3, CV_32F, Scalar(-1));
			kernel.at<float>(1, 1) = 8.9;
			filter2D(projector_desired_image_gray3, projector_desired_image_gray3, projector_desired_image_gray3.depth(), kernel);

			//find projector chessboard corners
			corners_found_result = findChessboardCorners(projector_desired_image_gray3, sl_parameter.projector_board_size, projector_distortion_image_points_buffer,
				CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
			if (corners_found_result)
			{
				drawChessboardCorners(projector_desired_image2, sl_parameter.projector_board_size, projector_distortion_image_points_buffer, corners_found_result);
				imshow("projector desired image", projector_desired_image2);
				key = waitKey(3000);
				if (key == 'o')			//save desired image	
				{
					key = 'n';
					str_temp.str("");		//clean out stringstream
					str_temp << ".\\output\\calibration\\extrinsic\\camera_image_" << frame_grap_count << ".jpg";
					imwrite(string(str_temp.str()), camera_desired_image1);
					str_temp.str("");		//clean out stringstream
					str_temp << ".\\output\\calibration\\extrinsic\\camera_image_corners_" << frame_grap_count << ".jpg";
					imwrite(string(str_temp.str()), camera_desired_image2);
					str_temp.str("");		//clean out stringstream
					str_temp << ".\\output\\calibration\\extrinsic\\projector_image_" << frame_grap_count << ".jpg";
					imwrite(string(str_temp.str()), projector_desired_image1);
					str_temp.str("");		//clean out stringstream
					str_temp << ".\\output\\calibration\\extrinsic\\projector_image_corners_" << frame_grap_count << ".jpg";
					imwrite(string(str_temp.str()), projector_desired_image2);
					str_temp.str("");		//clean out stringstream
					str_temp << ".\\output\\calibration\\extrinsic\\projector_sub_image_gray_" << frame_grap_count << ".jpg";
					imwrite(string(str_temp.str()), projector_desired_image_gray3);

					//save detected distortion image corners coordinates
					cornerSubPix(camera_desired_image_gray, camera_distortion_image_points_buffer, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
					cornerSubPix(projector_desired_image_gray3, projector_distortion_image_points_buffer, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
					camera_distortion_image_points_array[frame_grap_count] = camera_distortion_image_points_buffer;
					projector_distortion_image_points_array[frame_grap_count] = projector_distortion_image_points_buffer;
					++frame_grap_count;
					cout << "grap " << frame_grap_count <<" frame" << endl;
				}
			}
		}
		//show raw iamge if corners detected not
		imshow("camera desired image", camera_desired_image1);
		imshow("projector desired image", projector_desired_image1);
		//show projector background pattern for next frame
		imshow("projector window", sl_parameter.projector_background_pattern);
		key = waitKey(3000);
	}

	//save detected distortion image corners coordinates
	if (frame_grap_count == 1)
	{
		FileStorage extrinsic_file(".\\output\\calibration\\extrinsic\\extrinsic_corners.xml", FileStorage::WRITE);
		if (!extrinsic_file.isOpened())
		{
			cout << "can't open extrinsic_corners.xml!" << endl;
			return -1;
		}
		for (int i = 0; i < 1; ++i)
		{
			str_temp.str("");
			str_temp << "camera_distortion_image_points_" << i;
			extrinsic_file << string(str_temp.str()) << camera_distortion_image_points_array[i];
			str_temp.str("");
			str_temp << "projector_distortion_image_points_" << i;
			extrinsic_file << string(str_temp.str()) << projector_distortion_image_points_array[i];
		}
		extrinsic_file.release();
		cout << "grap desired imge up to " << frame_grap_count << endl << endl;
	}

	destroyWindow("camera desired image");
	destroyWindow("projector desired image");

	
	//read camera and projector chessboard corners
	cout << "read camera chessboard corners......." << endl;
	FileStorage file_temp1(".\\output\\calibration\\extrinsic\\extrinsic_corners.xml", FileStorage::READ);
	if (!file_temp1.isOpened())
	{
		cout << "can't open extrinsic_corners.xml!" << endl;
		return -1;
	}
	for (int i = 0; i<1; ++i)
	{
		str_temp.str("");
		str_temp << "camera_distortion_image_points_" << i;
		file_temp1[string(str_temp.str())] >> camera_distortion_image_points_array[i];
	}
	for (int i = 0; i<1; ++i)
	{
		str_temp.str("");
		str_temp << "projector_distortion_image_points_" << i;
		file_temp1[string(str_temp.str())] >> projector_distortion_image_points_array[i];
	}
	file_temp1.release();

	//read camera and projector intrinsic and distortion parameters
	cout << "read camera intrinsic and distortion ......." << endl;
	FileStorage file_temp2(".\\output\\calibration\\projector\\slcalibration.xml", FileStorage::READ);
	if (!file_temp2.isOpened())
	{
		cout << "can't open slcalibration.xml!" << endl;
		return -1;
	}
	file_temp2["CameraIntrinsicMatrix"] >> sl_calibration.camera_intrinsic;
	file_temp2["CameraDistortionCofficients"] >> sl_calibration.camera_distortion;
	file_temp2["ProjectorIntrinsicMatrix"] >> sl_calibration.projector_intrinsic;
	file_temp2["ProjectorDistortionCofficients"] >> sl_calibration.projector_distortion;
	file_temp2.release();


	//estimate camera extrinsic parameter
	//calculate camera object corners position
	for (int i = 0; i < sl_parameter.camera_board_size.height; ++i)
		for (int j = 0; j < sl_parameter.camera_board_size.width; ++j)
			camera_object_points_buffer[i*sl_parameter.camera_board_size.width + j] = (Point3f(j*sl_parameter.camera_square_size.width, i*sl_parameter.camera_square_size.height, 0));
	for (int i = 0; i < 1; ++i)
		camera_object_points_array[i] = camera_object_points_buffer;

	cout << "estimate camera extrinsic parameter......." << endl;
	solvePnP(camera_object_points_array[0], camera_distortion_image_points_array[0],sl_calibration.camera_intrinsic, sl_calibration.camera_distortion,
		sl_calibration.camera_extrinsic_rotation, sl_calibration.camera_extrinsic_translation);

#ifdef DEBUG_PROJECT
	cout << "camera rotation vectors: " << sl_calibration.camera_extrinsic_rotation << endl;
	cout << "camera translation vectors: " << sl_calibration.camera_extrinsic_translation << endl;
#endif



	//estimate projector extrinsic parameter
	//calculate projector image points
	cout << "calculate projector image points....." << endl;
	for (int i = 0; i < sl_parameter.projector_board_size.height; ++i)
		for (int j = 0; j < sl_parameter.projector_board_size.width; ++j)
		{
			projector_image_points_buffer[i*sl_parameter.projector_board_size.width + j].x = j*sl_parameter.projector_square_size.width + 500 + sl_parameter.projector_square_size.width - 0.5f;
			projector_image_points_buffer[i*sl_parameter.projector_board_size.width + j].y = i*sl_parameter.projector_square_size.height + sl_parameter.projector_border_row + sl_parameter.projector_square_size.height - 0.5f;
		}
	for (int i = 0; i < 1; ++i)
		projector_image_points_array[i] = projector_image_points_buffer;

	// Estimate homography that maps undistorted image pixels to positions on the chessboard,then use perspectiveTransform to get projector object coordinate
	cout << "calculate projector object corners from calibrated camera......" << endl;
	vector<Point2f> camera_desttination(sl_parameter.camera_board_size.width*sl_parameter.camera_board_size.height);
	vector<Point2f> projector_destination(sl_parameter.camera_board_size.width*sl_parameter.camera_board_size.height);
	Mat homography = Mat::zeros(3, 3, CV_64FC1);
	//initialize projector object corners 
	projector_object_points_array = camera_object_points_array;
	for (int image_count = 0; image_count < 1; ++image_count)
	{
		undistortPoints(camera_distortion_image_points_array[image_count], camera_undistortion_image_points_array[image_count], sl_calibration.camera_intrinsic, sl_calibration.camera_distortion);
		undistortPoints(projector_distortion_image_points_array[image_count], projector_undistortion_image_points_array[image_count], sl_calibration.camera_intrinsic, sl_calibration.camera_distortion);
		//get camera_destination(x,y)(unit: mm) from camera_object(x,y,z)(unit: mm)
		for (int i = 0; i < sl_parameter.camera_board_size.width*sl_parameter.camera_board_size.height; ++i)
		{
			camera_desttination[i].x = camera_object_points_array[image_count][i].x;
			camera_desttination[i].y = camera_object_points_array[image_count][i].y;
		}
		homography = findHomography(camera_undistortion_image_points_array[image_count], camera_desttination);
		perspectiveTransform(projector_undistortion_image_points_array[image_count], projector_destination, homography);

		//get projector_object(x,y,z)(unit: mm) from projector_destination(x,y)(unit: mm)
		for (int i = 0; i < sl_parameter.camera_board_size.width*sl_parameter.camera_board_size.height; i++)
		{
			projector_object_points_array[image_count][i].x = projector_destination[i].x;
			projector_object_points_array[image_count][i].y = projector_destination[i].y;
			projector_object_points_array[image_count][i].z = 0.0f;
		}
	}
	cout << "start to run estimate projector extrinsic parameters......" << endl;
	solvePnP(projector_object_points_array[0], projector_image_points_array[0],sl_calibration.projector_intrinsic, sl_calibration.projector_distortion,
		sl_calibration.projector_extrinsic_rotation, sl_calibration.projector_extrinsic_translation);

	sl_calibration.camera_projector_extrinsic_calibration_flag = true;
	//save camera and projector extrinsic parameters
	cout << "saving camera extrinsic parameters......" << endl;
	FileStorage file_temp3(".\\output\\calibration\\extrinsic\\extrinsic.xml", FileStorage::WRITE);
	if (!file_temp3.isOpened())
	{
		cout << "can't open extrinsic.xml!" << endl;
		return -1;
	}
	file_temp3 << "CameraRotationVectors" << sl_calibration.camera_extrinsic_rotation;
	file_temp3 << "CameraTranslationVectors" << sl_calibration.camera_extrinsic_translation;
	file_temp3 << "ProjectorRotationVectors" << sl_calibration.projector_extrinsic_rotation;
	file_temp3 << "ProjectorTranslationVectors" << sl_calibration.projector_extrinsic_translation;
	file_temp3 << "CameraProjectorExtrinsicCalibrationFlag" << sl_calibration.camera_projector_extrinsic_calibration_flag;
	file_temp3.release();		//must release file
	cout << "estimate camera extrinsic parameter was successful......" << endl;
	cout << "estimate projector extrinsic parameter was successful......" << endl << endl;

#ifdef DEBUG_PROJECT
	cout << "projector rotation vectors: " << ": " << sl_calibration.projector_extrinsic_rotation << endl;
	cout << "projector translation vectors: " << ": " << sl_calibration.projector_extrinsic_translation << endl;
#endif
	return 0;
}

int EvaluteCameraProjectorGeometry(SlParameter &sl_parameter, SlCalibration &sl_calibration)
{
	cout << "evalute camera-projector geometry......" << endl;
	//check camera and projector intrinsic and extrinsic calibration 
	if (!sl_calibration.camera_intrinsic_calibration_flag || !sl_calibration.projector_intrinsic_calibration_flag || !sl_calibration.camera_projector_extrinsic_calibration_flag)
	{
		cout << "please calibrate camera and projector first" << endl << endl;
		return -1;
	}

	//read and translate camera and projector rotation and translation
	Mat camera_rotation(3, 3, CV_64FC1,Scalar(0.0f));
	Mat camera_translation(3, 1, CV_64FC1, Scalar(0.0f));
	Mat projector_rotation(3, 3, CV_64FC1, Scalar(0.0f));
	Mat projector_translation(3, 1, CV_64FC1, Scalar(0.0f));
	sl_calibration.camera_projection_center.create(3, 1, CV_64FC1);
	sl_calibration.projector_projection_center.create(3, 1, CV_64FC1);

	//translate rotation vector into matrix
	Rodrigues(sl_calibration.camera_extrinsic_rotation,camera_rotation);
	Rodrigues(sl_calibration.projector_extrinsic_rotation, projector_rotation);
	camera_translation = sl_calibration.camera_extrinsic_translation;
	projector_translation = sl_calibration.projector_extrinsic_translation;

	//get projector's center of projection(under camera coordinate):Rp*Xw+Tp=Xp(0;0;0),(Rc*(Xp->Xw)+Tc)->Xc;
	//mind that projector and camera own the same world coordinate
	sl_calibration.projector_projection_center = projector_rotation.inv()*(-1 * projector_translation);        //under Xw:(100.27;61.644;390.56)
	sl_calibration.projector_projection_center = camera_rotation*sl_calibration.projector_projection_center+camera_translation; //under Xc:(-105.9;0.08797;40.017)
	
	vector<Point2f> camera_distortion_pixels(sl_parameter.camera_height*sl_parameter.camera_width);
	vector<Point2f> camera_undistortion_pixels(sl_parameter.camera_height*sl_parameter.camera_width);
	vector<Point2f> projector_distortion_pixels(sl_parameter.projector_height*sl_parameter.projector_width);
	vector<Point2f> projector_undistortion_pixels(sl_parameter.projector_height*sl_parameter.projector_width);
	sl_calibration.camera_pixel_rays.create(3, sl_parameter.camera_height*sl_parameter.camera_width, CV_64FC1);
	sl_calibration.projector_pixel_rays.create(3, sl_parameter.camera_height*sl_parameter.camera_width, CV_64FC1);

	int row_index=0, colum_index=0;
	float normalization=0.0f;

	//Initialize camera center with 0.0f
	sl_calibration.camera_projection_center.setTo(0.0f);

	//Pre-compute optical rays for each camera pixel. mind lens distortion compensation must be performed firstly
	for (row_index = 0; row_index < sl_parameter.camera_height; ++row_index)
		for (colum_index = 0; colum_index < sl_parameter.camera_width; ++colum_index)
			camera_distortion_pixels[row_index*sl_parameter.camera_width + colum_index] = Point2f(colum_index,row_index);
	undistortPoints(camera_distortion_pixels, camera_undistortion_pixels,sl_calibration.camera_intrinsic,sl_calibration.camera_distortion);
	for (row_index = 0; row_index < sl_parameter.camera_height; ++row_index)
		for (colum_index = 0; colum_index < sl_parameter.camera_width; ++colum_index)
		{
			normalization = (double)sqrt(pow(camera_undistortion_pixels[row_index*sl_parameter.camera_width + colum_index].x, 2.0) + pow(camera_undistortion_pixels[row_index*sl_parameter.camera_width + colum_index].y, 2.0) + 1.0);
		//	sl_calibration.camera_pixel_rays.push_back(Point3f(camera_undistortion_pixels[row_index*sl_parameter.camera_width + colum_index].x / normalization, camera_undistortion_pixels[row_index*sl_parameter.camera_width + colum_index].y / normalization, 1.0 / normalization));
			sl_calibration.camera_pixel_rays.at<double>(0, row_index*sl_parameter.camera_width + colum_index) = camera_undistortion_pixels[row_index*sl_parameter.camera_width + colum_index].x / normalization;
			sl_calibration.camera_pixel_rays.at<double>(1, row_index*sl_parameter.camera_width + colum_index) = camera_undistortion_pixels[row_index*sl_parameter.camera_width + colum_index].y / normalization;
			sl_calibration.camera_pixel_rays.at<double>(2, row_index*sl_parameter.camera_width + colum_index) = 1.0 / normalization;
		}

	//Pre-compute optical rays for each projector pixel
	for (row_index = 0; row_index < sl_parameter.projector_height; ++row_index)
		for (colum_index = 0; colum_index < sl_parameter.projector_width; ++colum_index)
			projector_distortion_pixels[row_index*sl_parameter.projector_width + colum_index] = Point2f(colum_index, row_index);
	undistortPoints(projector_distortion_pixels, projector_undistortion_pixels, sl_calibration.projector_intrinsic, sl_calibration.projector_distortion);
	for (row_index = 0; row_index < sl_parameter.projector_height; ++row_index)
		for (colum_index = 0; colum_index < sl_parameter.projector_width; ++colum_index)
		{
			normalization = (double)sqrt(pow(projector_undistortion_pixels[row_index*sl_parameter.projector_width + colum_index].x, 2.0) + pow(projector_undistortion_pixels[row_index*sl_parameter.projector_width + colum_index].y, 2.0) + 1.0);
	//		sl_calibration.projector_pixel_rays.push_back(Point3f(projector_undistortion_pixels[row_index*sl_parameter.projector_width + colum_index].x / normalization, projector_undistortion_pixels[row_index*sl_parameter.projector_width + colum_index].y / normalization, 1.0 / normalization));
			sl_calibration.projector_pixel_rays.at<double>(0, row_index*sl_parameter.projector_width + colum_index) = projector_undistortion_pixels[row_index*sl_parameter.projector_width + colum_index].x / normalization;
			sl_calibration.projector_pixel_rays.at<double>(1, row_index*sl_parameter.projector_width + colum_index) = projector_undistortion_pixels[row_index*sl_parameter.projector_width + colum_index].y / normalization;
			sl_calibration.projector_pixel_rays.at<double>(2, row_index*sl_parameter.projector_width + colum_index) = 1.0/ normalization;
		}

	//rotate each projector optical rays into camera coordinate
	Mat rotate_temp = camera_rotation*projector_rotation.inv();

	Mat temp3(3, 1, CV_64FC1, Scalar(0.0f));
	for (int i = 0; i < sl_parameter.projector_height*sl_parameter.projector_width; ++i)
	{
		sl_calibration.projector_pixel_rays.col(i) = rotate_temp*sl_calibration.projector_pixel_rays.col(i);
	}
	// Evaluate scale factor (to assist in plane-fitting)
	double scale = 0.0f;
	for (int i = 0; i < 3; ++i)
		scale += pow(sl_calibration.projector_projection_center.at<double>(i, 0),2.0);
	scale = sqrt(scale);

	// Estimate plane equations describing every projector column.
	// Note: Resulting coefficient vector is in camera coordinate system
	Mat colum_points(sl_parameter.projector_height + 1, 3, CV_64FC1);
	sl_calibration.projector_colum_planes.create(4, sl_parameter.projector_width, CV_64FC1);
	for (int c = 0; c < sl_parameter.projector_width; ++c)
	{
		for (int r = 0; r < sl_parameter.projector_height; ++r)
			for (int i = 0; i < 3; ++i)
				colum_points.at<double>(r, i) = sl_calibration.projector_projection_center.at<double>(i, 0)
				+ scale*sl_calibration.projector_pixel_rays.at<double>(i,c+r*sl_parameter.projector_width);
		for (int i = 0; i < 3; ++i)
			colum_points.at<double>(sl_parameter.projector_height, i) = sl_calibration.projector_projection_center.at<double>(i, 0);
		double plane[4];
		LineFitPlane(colum_points, plane);
		for (int i = 0; i < 4; ++i)
			sl_calibration.projector_colum_planes.at<double>(i, c) = plane[i];
	}

	// Estimate plane equations describing every projector row.
	// Note: Resulting coefficient vector is in camera coordinate system
	Mat row_points(sl_parameter.projector_width + 1, 3, CV_64FC1);
	sl_calibration.projector_row_planes.create(4, sl_parameter.projector_height, CV_64FC1);
	for (int r = 0; r < sl_parameter.projector_height; ++r)
	{
		for (int c = 0; c < sl_parameter.projector_width; ++c)
			for (int i = 0; i < 3; ++i)
				row_points.at<double>(c, i) = sl_calibration.projector_projection_center.at<double>(i, 0)
				+ scale*sl_calibration.projector_pixel_rays.at<double>(i, c + r*sl_parameter.projector_width);
		for (int i = 0; i < 3; ++i)
			row_points.at<double>(sl_parameter.projector_width, i) = sl_calibration.projector_projection_center.at<double>(i, 0);
		double plane[4];
		LineFitPlane(row_points, plane);
		for (int i = 0; i < 4; ++i)
			sl_calibration.projector_row_planes.at<double>(i, r) = plane[i];
	}

	cout << "evalute camera - projector geometry successful!" << endl << endl;
	return 0;
}

// Fit a hyperplane to a set of ND points.
// Note: Input points must be in the form of an NxM matrix, where M is the dimensionality.
//       This function finds the best-fit plane P, in the least-squares
//       sense, between the points (X,Y,Z). The resulting plane P is described
//       by the coefficient vector W, where W(1)*X + W(2)*Y +W(3)*Z = W(3), for
//       (X,Y,Z) on the plane P.
int LineFitPlane(Mat &points, double *plane)
{
	// Estimate geometric centroid for (x0,y0,z0;x1,y1,z1;.....;xi,yi,zi)
	int nrows = points.size().height;
	int ncols = points.size().width;
	Mat center(1, ncols, CV_64FC1,Scalar(0.0f));
	for (int c = 0; c < ncols; ++c)
	{
		for (int r = 0; r < nrows; ++r)
			center.at<double>(0, c) += points.at<double>(r, c);
		center.at<double>(0, c) /= nrows;
	}

	// Subtract geometric centroid from each point
	Mat points2(nrows, ncols, CV_64FC1,Scalar(0.0f));
	for (int c = 0; c < ncols; ++c)
	{
		for (int r = 0; r < nrows; ++r)
			points2.at<double>(r, c) = points.at<double>(r, c) - center.at<double>(0, c);
	}

//	Mat A(3, 3, CV_64FC1, Scalar(0.0f));
	Mat A;
	A = points2.t();
	A = points2.t()*points;

	Mat w(1, ncols, CV_64FC1, Scalar(0.0f));
	Mat u(ncols, ncols, CV_64FC1, Scalar(0.0f));
	Mat vt(ncols, ncols, CV_64FC1, Scalar(0.0f));
	SVD::compute(A, w, u, vt);

	plane[ncols] = 0;
	for (int c = 0; c < ncols; ++c)
	{
		plane[c] = vt.at<double>(ncols - 1, c);
		plane[ncols] += plane[c] * center.at<double>(0, c);
		
	}
	Mat errors(nrows, 1, CV_64FC1, Scalar(0.0f));
	for (int i = 0; i < nrows; ++i)
	{
		for (int j = 0; j < ncols; ++j)
			errors.at<double>(i, 0) += plane[j] * points.at<double>(i, j);
		errors.at<double>(i, 0) -= plane[ncols];
	}
	
	return 0;
}

