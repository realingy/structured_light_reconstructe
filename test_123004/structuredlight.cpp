#include <iostream>
#include <string>
#include <direct.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "structuredlight.h"

using namespace std;
using namespace cv;

SlParameter::SlParameter(int i)
{
	//Open configure file 
	FileStorage slparameter_file("slparameter.xml", FileStorage::READ);
	if (!slparameter_file.isOpened())
	{
		cerr << "Failed to open slparameter.xml!" << endl;
	}
	//Read output directory and object name
	output_directory = slparameter_file["output"]["output_directory"];
	object_name = slparameter_file["output"]["object_name"];
	save_enable = (int)(slparameter_file["output"]["save_enable"]);
#ifdef DEBUG_PROJECT
	cout << "output:" << endl;
	cout << "output_directory: " << output_directory << endl;
	cout << "object_name: " << object_name << endl;
	cout << "save_enable: " << save_enable << endl << endl;
#endif

	//Read camera parameters
	camera_id = slparameter_file["camera"]["id"];
	camera_width = slparameter_file["camera"]["width"];
	camera_height = slparameter_file["camera"]["height"];
#ifdef DEBUG_PROJECT
	cout << "camera:" << endl;
	cout << "width: " << camera_width << endl;
	cout << "height: " << camera_height << endl << endl;
#endif

	//Read projector parameters
	projector_width = slparameter_file["projector"]["width"];
	projector_height = slparameter_file["projector"]["height"];
	projector_invert = (int)slparameter_file["projector"]["invert"];
#ifdef DEBUG_PROJECT
	cout << "projector:" << endl;
	cout << "width: " << projector_width << endl;
	cout << "height: " << projector_height << endl;
	cout << "invert: " << projector_invert << endl << endl;
#endif

	//Read gain parameters
	camera_gain = slparameter_file["gain_parameters"]["camera_gain"];
	projector_gain = slparameter_file["gain_parameters"]["projector_gain"];
#ifdef DEBUG_PROJECT
	cout << "gain:" << endl;
	cout << "camera_gain: " << camera_gain << endl;
	cout << "projector_gain: " << projector_gain << endl << endl;
#endif

	//Read camera calibration  parameters
	camera_board_size.width = slparameter_file["camera_calibration_parameters"]["board_size_width"];
	camera_board_size.height = slparameter_file["camera_calibration_parameters"]["board_size_height"];
	camera_square_size.width = slparameter_file["camera_calibration_parameters"]["square_size_width"];
	camera_square_size.height = slparameter_file["camera_calibration_parameters"]["square_size_height"];
	frame_amount = slparameter_file["camera_calibration_parameters"]["frame_amount"];
#ifdef DEBUG_PROJECT
	cout << "camera calibration:" << endl;
	cout << "board_size_width: " << camera_board_size.width << endl;
	cout << "board_size_height: " << camera_board_size.height << endl;
	cout << "square_size_width: " << camera_square_size.width << endl;
	cout << "square_size_height: " << camera_square_size.height << endl;
	cout << "frame_amount: " << frame_amount << endl << endl;
#endif

	//Read projector calibration  parameters
	projector_board_size.width = slparameter_file["projector_calibration_parameters"]["board_size_width"];
	projector_board_size.height = slparameter_file["projector_calibration_parameters"]["board_size_height"];
	projector_square_size.width = slparameter_file["projector_calibration_parameters"]["square_size_width"];
	projector_square_size.height = slparameter_file["projector_calibration_parameters"]["square_size_height"];
#ifdef DEBUG_PROJECT
	cout << "projector calibration:" << endl;
	cout << "board_size_width: " << projector_board_size.width << endl;
	cout << "board_size_height: " << projector_board_size.height << endl;
	cout << "square_size_width: " << projector_square_size.width << endl;
	cout << "square_size_height: " << projector_square_size.height << endl << endl;
#endif

	//Read scan options
	colum_scan_flag = (int)slparameter_file["scan_colums_flag"];
	row_scan_flag = (int)slparameter_file["scan_rows_flag"];
	project_capture_delay = slparameter_file["project_capture_delay"];
	contrast_threshold = slparameter_file["contrast_threshold"];
	distance_range[0] = slparameter_file["distance_range_min"];
	distance_range[1] = slparameter_file["distance_range_max"];
	distance_reject = slparameter_file["distance_reject"];
	background_depth_threshold = slparameter_file["background_depth_threshold"];
#ifdef DEBUG_PROJECT
	cout << "colum_scan_flag: " << colum_scan_flag << endl;
	cout << "row_scan_flag: " << row_scan_flag << endl;
	cout << "project_capture_delay: " << project_capture_delay << endl;
	cout << "contrast_threshold: " << contrast_threshold << endl;
	cout << "distance_range_min: " << distance_range[0] << endl;
	cout << "distance_range_max: " << distance_range[1] << endl;
	cout << "distance_reject: " << distance_reject << endl;
	cout << "background_depth_threshold: " << background_depth_threshold << endl << endl;
#endif
	slparameter_file.release();	 //must release file


	 //Create output directory
	_mkdir(output_directory.c_str()); //c_str:convert string to c_string(string's data plus'\0')
	_mkdir((output_directory + "\\" + object_name).c_str());


	//Init parameter for graycode
	if (colum_scan_flag){
		colum_scan_amount = (int)(ceil(log(projector_width) / log(2)));
		colum_scan_shift = floor((pow(2.0, colum_scan_amount) - projector_width) / 2);
	}
	else{
		colum_scan_amount = 0;
		colum_scan_shift = 0;
	}
	if (row_scan_flag){
		row_scan_amount = (int)(ceil(log(projector_height) / log(2)));
		row_scan_shift = floor((pow(2.0, row_scan_amount) - projector_height) / 2);
	}
	else{
		row_scan_amount = 0;
		row_scan_shift = 0;
	}
	Mat temp1(projector_height, projector_width, CV_8UC1);
	if (colum_scan_flag && (!row_scan_flag)){
		for (int i = 0; i <= colum_scan_amount; ++i)
			projector_gray_code_image.push_back(temp1.clone());  //pay attention to clone()
	}
	else{
		for (int i = 0; i <= colum_scan_amount + row_scan_amount; ++i)
			projector_gray_code_image.push_back(temp1.clone());  //pay attention to clone()
	}
	projector_gray_code_image[0].setTo(255);	//define first code as an white image


	//init parameter of scanning object
	Mat temp2(camera_height, camera_width, CV_8UC3,Scalar(0));
	if (colum_scan_flag && (!row_scan_flag)){
		for (int i = 0; i <= 2 * (colum_scan_amount) + 1; ++i)
			camera_gray_code_image.push_back(temp2.clone());  //pay attention to clone()
	}
	else{
		for (int i = 0; i <= 2 * (colum_scan_amount + row_scan_flag) + 1; ++i)
			camera_gray_code_image.push_back(temp2.clone());  //pay attention to clone()
	}

	//init parameter for decoding graycode
	decode_colum_scan_image = Mat(camera_height, camera_width, CV_16UC1, Scalar(0));
	decode_row_scan_image = Mat(camera_height, camera_width, CV_16UC1, Scalar(0));
	gray_valid_image=Mat(camera_height,camera_width, CV_8UC1,Scalar(0));	
	scene_color=Mat(camera_height, camera_width, CV_8UC3,Scalar(0));

	//init parameter for reconstruct depthmap 
	depth_valid=Mat(camera_height, camera_width, CV_8UC1,Scalar(0));
	depth_namate=Mat(camera_height, camera_width, CV_64FC1,Scalar(0.0f));
	depth_points = Mat(camera_height, camera_width, CV_64FC3, Scalar(0.0f));
	depth_colors = Mat(camera_height, camera_width, CV_64FC3, Scalar(0.0f));
}

SlCalibration::SlCalibration(int i)
{
	//Open calibration file 
	FileStorage slcalibration_file(".\\output\\calibration\\projector\\slcalibration.xml", FileStorage::READ);
	if (!slcalibration_file.isOpened())
	{
		cerr << "Failed to open slcalibration.xml" << endl;
	}
	//read intrinsic and distortion parmeters
	slcalibration_file["CameraIntrinsicMatrix"] >> camera_intrinsic;
	slcalibration_file["CameraDistortionCofficients"] >> camera_distortion;
	slcalibration_file["ProjectorIntrinsicMatrix"] >> projector_intrinsic;
	slcalibration_file["ProjectorDistortionCofficients"] >> projector_distortion;
	slcalibration_file["CameraIntrinsicCalibrationFlag"] >> camera_intrinsic_calibration_flag;
	slcalibration_file["ProjectorIntrinsicCalibrationFlag"] >> projector_intrinsic_calibration_flag;
	slcalibration_file.release();

#ifdef DEBUG_PROJECT
	cout << "camera intrinsic matrix: " << endl << camera_intrinsic << endl;
	cout << "camera distortion cofficients: " << endl << camera_distortion << endl;
	cout << "projector intrinsic matrix: " << endl << projector_intrinsic << endl;
	cout << "projector distortion cofficients: " << endl << projector_distortion << endl;
	cout << "camera_intrinsic_calibration_flag: " << camera_intrinsic_calibration_flag << endl;
	cout << "projector_intrinsic_calibration_flag: " << projector_intrinsic_calibration_flag << endl << endl;
#endif

	//open extrinsic file
	FileStorage extrinsic_file(".\\output\\calibration\\extrinsic\\extrinsic.xml", FileStorage::READ);
	if (!extrinsic_file.isOpened())
	{
		cerr << "Failed to open extrinsic.xml!" << endl;
	}
	//read intrinsic and distortion parmeters
	extrinsic_file["CameraRotationVectors"] >> camera_extrinsic_rotation;
	extrinsic_file["CameraTranslationVectors"] >> camera_extrinsic_translation;
	extrinsic_file["ProjectorRotationVectors"] >> projector_extrinsic_rotation;
	extrinsic_file["ProjectorTranslationVectors"] >> projector_extrinsic_translation;
	extrinsic_file["CameraProjectorExtrinsicCalibrationFlag"] >> camera_projector_extrinsic_calibration_flag;

#ifdef DEBUG_PROJECT
	cout << "camera rotation vectors: " << endl << camera_extrinsic_rotation << endl;
	cout << "camera translation vectors: " << endl << camera_extrinsic_translation << endl;
	cout << "projector rotation vectors: " << endl << projector_extrinsic_rotation << endl;
	cout << "projector translation vectors: " << endl << projector_extrinsic_translation << endl;
	cout << "camera projector extrinsic calibration flag: " << camera_projector_extrinsic_calibration_flag << endl;
#endif
	extrinsic_file.release();
}