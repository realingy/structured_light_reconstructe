#include <iostream>
#include <string>
#include <direct.h>
#include <deque>
#include <queue>
#define _USE_MATH_DEFINES
#include <math.h>
#include <time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "structuredlight.h"
#include "auxiliaryfunctions.h"
#include "scan.h"
#include "phaseshift.h"

using namespace std;
using namespace cv;

PhaseShift::PhaseShift(PhaseShiftMode m1, PhaseShiftScanMode m2) : phaseshift_mode(m1), phaseshift_scan_mode(m2)
{
	Init(); 
}

void PhaseShift::Init()
{
	_mkdir("./output");
	_mkdir("./output/phaseshift");
	switch (phaseshift_mode){
		case PHASESHIFT_THREE:
			_mkdir("./output/phaseshift/phaseshift_three");
			if (phaseshift_scan_mode==VERTICAL)
				_mkdir("./output/phaseshift/phaseshift_three/vertical");
			else
				_mkdir("./output/phaseshift/phaseshift_three/horizontal");
			break;
		case PHASESHIFT_RGB:
			_mkdir("./output/phaseshift/phaseshift_rgb"); 
			if (phaseshift_scan_mode == VERTICAL)
				_mkdir("./output/phaseshift/phaseshift_rgb/vertical");
			else
				_mkdir("./output/phaseshift/phaseshift_rgb/horizontal");
			break;
		default:
			break;
	}
	//Read structuredlight setting parameters from structuredlight.xml
	FileStorage configuration_file("slparameter.xml", FileStorage::READ);
	if (!configuration_file.isOpened())
	{
		cout << "Failed to open slparameter.xml!" << endl;
		return ;
	}
	//Read image parameters
	image_width = configuration_file["camera"]["width"];
	image_height = configuration_file["camera"]["height"];
#ifdef DEBUG_PROJECT
	cout << "image:" << endl;
	cout << "width: " << image_width  << endl;
	cout << "height: " << image_height << endl << endl;
#endif
	//Read pattern parameters
	pattern_width = configuration_file["projector"]["width"];
	pattern_height = configuration_file["projector"]["height"];
	pattern_period = configuration_file["projector"]["period"];
#ifdef DEBUG_PROJECT
	cout << "projector:" << endl;
	cout << "width: " << pattern_width << endl;
	cout << "height: " << pattern_height << endl;
	cout << "period: " << pattern_period << endl << endl;
#endif

	//Read gain parameters
	image_gain = configuration_file["gain_parameters"]["camera_gain"];
	pattern_gain = configuration_file["gain_parameters"]["projector_gain"];
#ifdef DEBUG_PROJECT
	cout << "gain:" << endl;
	cout << "image_gain: " << image_gain << endl;
	cout << "pattern_gain: " << pattern_gain << endl << endl;
#endif

	//Read scan options
	project_capture_delay = configuration_file["project_capture_delay"];
	three_pixel_range_threeshold = configuration_file["three_pixel_range_threeshold"];
	int temp = configuration_file["three_noise_threshold"];
	three_noise_threshold=temp/100.0f;
#ifdef DEBUG_PROJECT
	cout << "project_capture_delay: " << project_capture_delay << endl;
	cout<<"three_pixel_range_threeshold: "<<three_pixel_range_threeshold<<endl;
	cout<<"three_noise_threshold:"<<three_noise_threshold<<endl;
#endif

	switch (phaseshift_mode){
	case PHASESHIFT_THREE:
		three_pattern_phase1 = Mat(pattern_height, pattern_width, CV_8UC1, Scalar(0));
		three_pattern_phase2 = Mat(pattern_height, pattern_width, CV_8UC1, Scalar(0));
		three_pattern_phase3 = Mat(pattern_height, pattern_width, CV_8UC1, Scalar(0));

		three_image1 = Mat(image_height, image_width, CV_8UC3, Scalar(0));
		three_image2 = Mat(image_height, image_width, CV_8UC3, Scalar(0));
		three_image3 = Mat(image_height, image_width, CV_8UC3, Scalar(0));
		three_image_gray1 = Mat(image_height, image_width, CV_8UC1, Scalar(0));
		three_image_gray2 = Mat(image_height, image_width, CV_8UC1, Scalar(0));
		three_image_gray3 = Mat(image_height, image_width, CV_8UC1, Scalar(0));
		break;

	case PHASESHIFT_RGB:
		rgb_pattern = Mat(pattern_height, pattern_width, CV_8UC3, Scalar(0));
		rgb_image = Mat(image_height, image_width, CV_8UC3, Scalar(0));
		break;
	default:
		break;
	}
	three_phase_mask = Mat(image_height, image_width, CV_8UC1, Scalar(0));
	three_unwrap_process = Mat(image_height, image_width, CV_8UC1, Scalar(0));
	three_phase_color = Mat(image_height, image_width, CV_8UC3, Scalar(0));
	three_phase_range = Mat(image_height, image_width, CV_64FC1, Scalar(0));
	three_wrapped_atan = Mat(image_height, image_width, CV_64FC1, Scalar(0));
	three_wrapped_fast = Mat(image_height, image_width, CV_64FC1, Scalar(0));
	three_phase_quality = Mat(image_height, image_width, CV_64FC1, Scalar(0));
	three_unwrapped_quality = Mat(image_height, image_width, CV_64FC1, Scalar(0));
	three_unwrapped_result = Mat(image_height, image_width, CV_16UC1, Scalar(0));
}

int PhaseShift::GeneratePhaseShiftPattern()
{
	unsigned int row_index = 0, colum_index = 0;
	if (phaseshift_mode == PHASESHIFT_THREE){
		string save_directory("./output/phaseshift/phaseshift_three");
		string save_name("");

		if (phaseshift_scan_mode == VERTICAL){
			save_directory += "/vertical/pattern";
			_mkdir(save_directory.c_str());

			for (row_index = 0; row_index < pattern_height; ++row_index){
				if (row_index == 0){
					for (colum_index = 0; colum_index < pattern_width; ++colum_index){
						three_pattern_phase1.at<unsigned char>(0, colum_index) = 127.5* (cos(2 * M_PI*colum_index / pattern_period-2*M_PI/3)+1);
						three_pattern_phase2.at<unsigned char>(0, colum_index) = 127.5* (cos(2 * M_PI*colum_index / pattern_period) + 1);
						three_pattern_phase3.at<unsigned char>(0, colum_index) = 127.5* (cos(2 * M_PI*colum_index / pattern_period+2*M_PI/3) + 1);
					}
				}
				else{
					for (colum_index = 0; colum_index < pattern_width; ++colum_index){
						three_pattern_phase1.at<unsigned char>(row_index, colum_index) = three_pattern_phase1.at<unsigned char>(0, colum_index);
						three_pattern_phase2.at<unsigned char>(row_index, colum_index) = three_pattern_phase2.at<unsigned char>(0, colum_index);
						three_pattern_phase3.at<unsigned char>(row_index, colum_index) = three_pattern_phase3.at<unsigned char>(0, colum_index);
					}
				}
			}
			save_name = "";
			save_name = save_directory + "/v_three_phase1.jpg";
			imwrite(save_name, three_pattern_phase1);
			save_name = "";
			save_name = save_directory + "/v_three_phase2.jpg";
			imwrite(save_name, three_pattern_phase2);
			save_name = "";
			save_name = save_directory + "/v_three_phase3.jpg";
			imwrite(save_name, three_pattern_phase3);
		}

		else if (phaseshift_scan_mode == HORIZONTAL){
			save_directory += "/horizontal/pattern";
			_mkdir(save_directory.c_str());

			for (colum_index = 0; colum_index < pattern_width; ++colum_index){
				if (colum_index == 0){
					for (row_index = 0; row_index < pattern_height; ++row_index){
						three_pattern_phase1.at<unsigned char>(row_index, 0) = 127.5* (cos(2 * M_PI*row_index / pattern_period - 2 * M_PI / 3) + 1);
						three_pattern_phase2.at<unsigned char>(row_index, 0) = 127.5* (cos(2 * M_PI*row_index / pattern_period) + 1);
						three_pattern_phase3.at<unsigned char>(row_index, 0) = 127.5* (cos(2 * M_PI*row_index / pattern_period + 2 * M_PI / 3) + 1);
					}
				}
				else{
					for (row_index = 0; row_index < pattern_height; ++row_index){
						three_pattern_phase1.at<unsigned char>(row_index, colum_index) = three_pattern_phase1.at<unsigned char>(row_index, 0);
						three_pattern_phase2.at<unsigned char>(row_index, colum_index) = three_pattern_phase2.at<unsigned char>(row_index, 0);
						three_pattern_phase3.at<unsigned char>(row_index, colum_index) = three_pattern_phase3.at<unsigned char>(row_index, 0);
					}
				}
			}
			save_name = "";
			save_name = save_directory + "/h_three_phase1.jpg";
			imwrite(save_name, three_pattern_phase1);
			save_name = "";
			save_name = save_directory + "/h_three_phase2.jpg";
			imwrite(save_name, three_pattern_phase2);
			save_name = "";
			save_name = save_directory + "/h_three_phase3.jpg";
			imwrite(save_name, three_pattern_phase3);
		}
	}
	else if (phaseshift_mode == PHASESHIFT_RGB){
		string save_directory("./output/phaseshift/phaseshift_rgb");
		string save_name("");

		if (phaseshift_scan_mode == VERTICAL){
			save_directory += "/vertical/pattern";
			_mkdir(save_directory.c_str());
			for (row_index = 0; row_index < pattern_height; ++row_index){
				if (row_index == 0){
					for (colum_index = 0; colum_index < pattern_width; ++colum_index){
						rgb_pattern.at<Vec3b>(0, colum_index)[2] = 127.5* (cos(2 * M_PI*colum_index / pattern_period - 2 * M_PI / 3) + 1); //R
						rgb_pattern.at<Vec3b>(0, colum_index)[1] = 127.5* (cos(2 * M_PI*colum_index / pattern_period) + 1);				 //G
						rgb_pattern.at<Vec3b>(0, colum_index)[0] = 127.5* (cos(2 * M_PI*colum_index / pattern_period + 2 * M_PI / 3) + 1); //B
					}
				}
				else{
					for (colum_index = 0; colum_index < pattern_width; ++colum_index){
						rgb_pattern.at<Vec3b>(row_index, colum_index)[2] = rgb_pattern.at<Vec3b>(0, colum_index)[2];
						rgb_pattern.at<Vec3b>(row_index, colum_index)[1] = rgb_pattern.at<Vec3b>(0, colum_index)[1];
						rgb_pattern.at<Vec3b>(row_index, colum_index)[0] = rgb_pattern.at<Vec3b>(0, colum_index)[0];
					}
				}
			}
			save_name = "";
			save_name = save_directory + "/v_rgb.jpg";
			imwrite(save_name, rgb_pattern);
		}
		else if (phaseshift_scan_mode == HORIZONTAL){
			save_directory += "/horizontal/pattern";
			_mkdir(save_directory.c_str());
			for (colum_index = 0; colum_index < pattern_width; ++colum_index){
				if (colum_index == 0){
					for (row_index = 0; row_index < pattern_height; ++row_index){
						rgb_pattern.at<Vec3b>(row_index, 0)[2] = 127.5* (cos(2 * M_PI*row_index / pattern_period - 2 * M_PI / 3) + 1); //R
						rgb_pattern.at<Vec3b>(row_index, 0)[1] = 127.5* (cos(2 * M_PI*row_index / pattern_period) + 1);				 //G
						rgb_pattern.at<Vec3b>(row_index, 0)[0] = 127.5* (cos(2 * M_PI*row_index / pattern_period + 2 * M_PI / 3) + 1); //B
					}
				}
				else{
					for (row_index = 0; row_index < pattern_height; ++row_index){
						rgb_pattern.at<Vec3b>(row_index, colum_index)[2] = rgb_pattern.at<Vec3b>(row_index, 0)[2];
						rgb_pattern.at<Vec3b>(row_index, colum_index)[1] = rgb_pattern.at<Vec3b>(row_index, 0)[1];
						rgb_pattern.at<Vec3b>(row_index, colum_index)[0] = rgb_pattern.at<Vec3b>(row_index, 0)[0];
					}
				}
			}
			save_name = "";
			save_name = save_directory + "/h_rgb.jpg";
			imwrite(save_name, rgb_pattern);
		}
	}
	return 1;
}

int PhaseShift::ReadPhaseShiftPattern()
{
	if (phaseshift_mode== PHASESHIFT_THREE){
		string read_directory("./output/phaseshift/phaseshift_three");
		ostringstream read_name("");
		Mat pattern_temp(pattern_height, pattern_width, CV_8UC1, Scalar(0));

		if (phaseshift_scan_mode == VERTICAL){
			read_directory += "/vertical/pattern";
			for (int i = 1; i < 4; ++i){
				read_name.str("");
				read_name << read_directory<<"/v_three_phase" << i << ".jpg";
				pattern_temp = imread(read_name.str(), CV_LOAD_IMAGE_GRAYSCALE);
				if (pattern_temp.data==NULL){
					cout << "can not open " << read_name.str()<< endl;
					return 0;
				}
				else{
					switch (i){
						case 1:
							three_pattern_phase1 = pattern_temp; break;
						case 2:
							three_pattern_phase2 = pattern_temp; break;
						case 3:
							three_pattern_phase3 = pattern_temp; break;
						default:
							break;
					}
				}
			}
		}
		else if (phaseshift_scan_mode == HORIZONTAL){
			read_directory += "/horizontal/pattern";
			for (int i = 1; i < 4; ++i){
				read_name.str("");
				read_name << read_directory << "/h_three_phase" << i << ".jpg";
				pattern_temp = imread(read_name.str(), CV_LOAD_IMAGE_GRAYSCALE);
				if (pattern_temp.data == NULL){
					cout << "can not open " << read_name.str() << endl;
					return 0;
				}
				else{
					switch (i){
					case 1:
						three_pattern_phase1 = pattern_temp; break;
					case 2:
						three_pattern_phase2 = pattern_temp; break;
					case 3:
						three_pattern_phase3 = pattern_temp; break;
					default:
						break;
					}
				}	
			}
		}
	}
	else if (phaseshift_mode == PHASESHIFT_RGB){
		string read_directory("./output/phaseshift/phaseshift_rgb");
		ostringstream read_name("");
		Mat pattern_temp(pattern_height, pattern_width, CV_8UC3, Scalar(0));

		if (phaseshift_scan_mode == VERTICAL){
			read_directory += "/vertical/pattern";
			read_name.str("");
			read_name << read_directory << "/v_rgb.jpg";
			pattern_temp = imread(read_name.str(), CV_LOAD_IMAGE_COLOR);
			if (pattern_temp.data == NULL){
				cout << "can not open " << read_name.str()<<endl;
				return 0;
			}
			else{
				rgb_pattern = pattern_temp;
			}
		}
		else if (phaseshift_scan_mode == HORIZONTAL){
			read_directory += "/horizontal/pattern";
			read_name.str("");
			read_name << read_directory << "/h_rgb.jpg";
			pattern_temp = imread(read_name.str(), CV_LOAD_IMAGE_COLOR);
			if (pattern_temp.data == NULL){
				cout << "can not open " << read_name.str() << endl;
				return 0;
			}
			else{
				rgb_pattern = pattern_temp;
			}	
		}
	}
	return 1;
}

int PhaseShift::ScanObject(bool save_enable)
{
	Mat image_grab(image_height, image_width, CV_8UC3, Scalar(0));
	Mat project_pattern(pattern_height, pattern_width, CV_8UC1,Scalar(255));
	if (save_enable){
		//create a window to show capture result
		cout << "run phaseshift scan object....." << endl;
		namedWindow("scan object", WINDOW_AUTOSIZE);
		moveWindow("scan object", 50, 0);
		imshow("projector window", project_pattern*(pattern_gain / 100.0f));
		waitKey(100);
	}

	if (phaseshift_mode == PHASESHIFT_THREE){
		string save_directory("./output/phaseshift/phaseshift_three");
		string save_name("");
		if (phaseshift_scan_mode == VERTICAL){
			save_directory += "/vertical/image";
			_mkdir(save_directory.c_str());
			imshow("projector window", three_pattern_phase1*(pattern_gain / 100.0f));  //capture v_phase_1 image
			waitKey(project_capture_delay);
			if (GetImage(image_grab))
			{
				//waitKey(project_capture_delay);
				//GetImage(image_grab);
				three_image1 = image_grab*(image_gain / 100.0f);
				cvtColor(three_image1, three_image_gray1, COLOR_BGR2GRAY);
				imshow("scan object", three_image1);
				if (save_enable){
					save_name = "";
					save_name = save_directory + "/v_three_image1.jpg";
					imwrite(save_name.c_str(), three_image1);
					save_name = "";
					save_name = save_directory + "/v_three_image_gray1.jpg";
					imwrite(save_name.c_str(), three_image_gray1);
				}
			}

			imshow("projector window", three_pattern_phase2*(pattern_gain / 100.0f)); //capture v_phase_2 image
			waitKey(project_capture_delay);
			if (GetImage(image_grab))
			{
				//waitKey(project_capture_delay);
				//GetImage(image_grab);
				three_image2 = image_grab*(image_gain / 100.0f);
				cvtColor(three_image2, three_image_gray2, COLOR_RGB2GRAY);
				imshow("scan object", three_image2);
				if (save_enable){
					save_name = "";
					save_name = save_directory + "/v_three_image2.jpg";
					imwrite(save_name.c_str(), three_image2);
					save_name = "";
					save_name = save_directory + "/v_three_image_gray2.jpg";
					imwrite(save_name.c_str(), three_image_gray2);
				}
			}

			imshow("projector window", three_pattern_phase3*(pattern_gain / 100.0f)); //capture v_phase_3 image
			waitKey(project_capture_delay);
			if (GetImage(image_grab)){
				//waitKey(project_capture_delay);
				//GetImage(image_grab);
				three_image3 = image_grab*(image_gain / 100.0f);
				cvtColor(three_image3, three_image_gray3, COLOR_RGB2GRAY);
				imshow("scan object", three_image3);
				if (save_enable){
					save_name = "";
					save_name = save_directory + "/v_three_image3.jpg";
					imwrite(save_name.c_str(), three_image3);
					save_name = "";
					save_name = save_directory + "/v_three_image_gray3.jpg";
					imwrite(save_name.c_str(), three_image_gray3);
				}
			}
		}
		else if (phaseshift_scan_mode == HORIZONTAL){
			save_directory += "/horizontal/image";
			_mkdir(save_directory.c_str());
			imshow("projector window", three_pattern_phase1*(pattern_gain / 100.0f));  //capture h_phase_1 image
			waitKey(project_capture_delay);
			if (GetImage(image_grab))
			{
				//waitKey(project_capture_delay);
				//GetImage(image_grab);
				three_image1 = image_grab*(image_gain / 100.0f);
				cvtColor(three_image1, three_image_gray1, COLOR_RGB2GRAY);
				imshow("scan object", three_image1);
				if (save_enable){
					save_name = "";
					save_name = save_directory + "/h_three_image1.jpg";
					imwrite(save_name.c_str(), three_image1);
					save_name = "";
					save_name = save_directory + "/h_three_image_gray1.jpg";
					imwrite(save_name.c_str(), three_image_gray1);
				}
			}

			imshow("projector window", three_pattern_phase2*(pattern_gain / 100.0f)); //capture h_phase_2 image
			waitKey(project_capture_delay);
			if (GetImage(image_grab))
			{
				//waitKey(project_capture_delay);
				//GetImage(image_grab);
				three_image2 = image_grab*(image_gain / 100.0f);
				cvtColor(three_image2, three_image_gray2, COLOR_RGB2GRAY);
				imshow("scan object", three_image2);
				if (save_enable){
					save_name = "";
					save_name = save_directory + "/h_three_image2.jpg";
					imwrite(save_name.c_str(), three_image2);
					save_name = "";
					save_name = save_directory + "/h_three_image_gray2.jpg";
					imwrite(save_name.c_str(), three_image_gray2);
				}
			}

			imshow("projector window", three_pattern_phase3*(pattern_gain / 100.0f)); //capture h_phase_3 image
			waitKey(project_capture_delay);
			if (GetImage(image_grab)){
				//waitKey(project_capture_delay);
				//GetImage(image_grab);
				three_image3 = image_grab*(image_gain / 100.0f);
				cvtColor(three_image3, three_image_gray3, COLOR_RGB2GRAY);
				imshow("scan object", three_image3);
				if (save_enable){
					save_name = "";
					save_name = save_directory + "/h_three_image3.jpg";
					imwrite(save_name.c_str(), three_image3);
					save_name = "";
					save_name = save_directory + "/h_three_image_gray3.jpg";
					imwrite(save_name.c_str(), three_image_gray3);
				}
			}
		}
	}
	else if (phaseshift_mode == PHASESHIFT_RGB){
		string save_directory("./output/phaseshift/phaseshift_rgb");
		string save_name("");
		if (phaseshift_scan_mode == VERTICAL){
			save_directory += "/vertical/image";
			_mkdir(save_directory.c_str());
			imshow("projector window", rgb_pattern*(pattern_gain / 100.0f));  //capture v_rgb image
			waitKey(project_capture_delay);
			if (GetImage(image_grab))
			{
				//waitKey(project_capture_delay);
				//GetImage(image_grab);
				rgb_image = image_grab*(image_gain / 100.0f);
				imshow("scan object", rgb_image);
				if (save_enable){
					save_name = "";
					save_name = save_directory + "/v_rgb_image.jpg";
					imwrite(save_name.c_str(), rgb_image);
				}
			}
		}
		else if(phaseshift_scan_mode == HORIZONTAL){
			save_directory += "/horizontal/image";
			_mkdir(save_directory.c_str());
			imshow("projector window", rgb_pattern*(pattern_gain / 100.0f));  //capture h_rgb image
			waitKey(project_capture_delay);
			if (GetImage(image_grab))
			{
				//waitKey(project_capture_delay);
				//GetImage(image_grab);
				rgb_image = image_grab*(image_gain / 100.0f);
				imshow("scan object", rgb_image);
				if (save_enable){
					save_name = "";
					save_name = save_directory + "/h_rgb_image.jpg";
					imwrite(save_name.c_str(), rgb_image);
				}
			}
		}
	}
	if (save_enable){
		destroyWindow("scan object");
	}
	return 1;
}

int PhaseShift::ReadScanImage()
{
	if (phaseshift_mode == PHASESHIFT_THREE){
		string read_directory("./output/phaseshift/phaseshift_three");
		ostringstream read_name("");
		Mat image_temp(image_height, image_width, CV_8UC3, Scalar(0));
		Mat image_gray_temp(image_height, image_width, CV_8UC1, Scalar(0));

		if (phaseshift_scan_mode == VERTICAL){
			read_directory += "/vertical/image";
			for (int i = 1; i < 4; ++i){
				read_name.str("");
				read_name << read_directory << "/v_three_image" << i << ".jpg";
				image_temp = imread(read_name.str(), CV_LOAD_IMAGE_COLOR);
				read_name.str("");
				read_name << read_directory << "/v_three_image_gray" << i << ".jpg";
				image_gray_temp = imread(read_name.str(), CV_LOAD_IMAGE_GRAYSCALE);
				if ((image_temp.data == NULL)||(image_gray_temp.data==NULL)){
					cout << "can not open scan files"<<endl;
					return 0;
				}
				else{
					switch (i){
					case 1:
						three_image1 = image_temp;
						three_image_gray1 = image_gray_temp;
						break;
					case 2:
						three_image2 = image_temp;
						three_image_gray2 = image_gray_temp;
					case 3:
						three_image3 = image_temp;
						three_image_gray3 = image_gray_temp;
					default:
						break;
					}
				}
			}
		}
		else if (phaseshift_scan_mode == HORIZONTAL){
			read_directory += "/horizontal/image";
			for (int i = 1; i < 4; ++i){
				read_name.str("");
				read_name << read_directory << "/h_three_image" << i << ".jpg";
				image_temp = imread(read_name.str(), CV_LOAD_IMAGE_COLOR);
				read_name.str("");
				read_name << read_directory << "/h_three_image_gray" << i << ".jpg";
				image_gray_temp = imread(read_name.str(), CV_LOAD_IMAGE_GRAYSCALE);
				if ((image_temp.data == NULL) || (image_gray_temp.data == NULL)){
					cout << "can not open scan files" << endl;
					return 0;
				}
				else{
					switch (i){
					case 1:
						three_image1 = image_temp;
						three_image_gray1 = image_gray_temp;
						break;
					case 2:
						three_image2 = image_temp;
						three_image_gray2 = image_gray_temp;
					case 3:
						three_image3 = image_temp;
						three_image_gray3 = image_gray_temp;
					default:
						break;
					}
				}
			}
		}
	}
	else if (phaseshift_mode == PHASESHIFT_RGB){
		string read_directory("./output/phaseshift/phaseshift_rgb");
		ostringstream read_name("");
		Mat image_temp(image_height, image_width, CV_8UC3, Scalar(0));

		if (phaseshift_scan_mode == VERTICAL){
			read_directory += "/vertical/image";
			read_name << read_directory << "/v_rgb_image.jpg";
			image_temp = imread(read_name.str(), CV_LOAD_IMAGE_COLOR);
			if (image_temp.data == NULL){
				cout << "can not open " << read_name.str() << endl;
				return 0;
			}
			else{
				rgb_image = image_temp;
			}
		}
		else if (phaseshift_scan_mode == HORIZONTAL){
			read_directory += "/horizontal/image";
			read_name << read_directory << "/h_rgb_image.jpg";
			image_temp = imread(read_name.str(), CV_LOAD_IMAGE_COLOR);
			if (image_temp.data == NULL){
				cout << "can not open " << read_name.str() << endl;
				return 0;
			}
			else{
				rgb_image = image_temp;
			}
		}
	}
	return 1;
}

int PhaseShift::ComputeMaskRegion()
{
	if (phaseshift_mode == PHASESHIFT_THREE){
		unsigned char phase1_value = 0, phase2_value = 0, phase3_value = 0, phase_min = 0, phase_max = 0;
		double phase_range = 0.0f, phase_sum = 0.0f,signal_noise_ratio = 0.0f;

		////use .at<> to call each element : 764ms
		//int row_index = 0, colum_index = 0;
		//for (row_index = 0; row_index < image_height; ++row_index){
		//	for (colum_index = 0; colum_index < image_width; ++colum_index){
		//		phase1_value = three_image_gray1.at<unsigned char>(row_index, colum_index);
		//		phase2_value = three_image_gray2.at<unsigned char>(row_index, colum_index);
		//		phase3_value = three_image_gray3.at<unsigned char>(row_index, colum_index);
		//		phase_min = Min(phase1_value, phase2_value, phase3_value);
		//		phase_max = Max(phase1_value, phase2_value, phase3_value);
		//		phase_range = phase_max - phase_min;
		//		phase_sum = phase1_value + phase2_value + phase3_value;
		//		
		//		if (phase_sum != 0){
		//			signal_noise_ratio = (double)phase_range / phase_sum;
		//			if (signal_noise_ratio> three_noise_threshold){
		//				three_phase_mask.at<unsigned char>(row_index, colum_index) = 255;
		//				if (phase_max == phase1_value){
		//					for (int i = 0; i < 3;++i)
		//						three_color_image.at<Vec3b>(row_index, colum_index)[i] = three_image1.at<Vec3b>(row_index, colum_index)[i];
		//				}
		//				else if (phase_max == phase2_value){
		//					for (int i = 0; i < 3; ++i)
		//						three_color_image.at<Vec3b>(row_index, colum_index)[i] = three_image2.at<Vec3b>(row_index, colum_index)[i];
		//				}
		//				else{
		//					for (int i = 0; i < 3;++i)
		//						three_color_image.at<Vec3b>(row_index, colum_index)[i] = three_image3.at<Vec3b>(row_index, colum_index)[i];
		//				}
		//			}
		//		}
		//	}
		//}
		//use .ptr to call each element : 63ms
		if (three_image_gray1.isContinuous() && three_image_gray2.isContinuous() && three_image_gray3.isContinuous()){
			Size size = three_image_gray1.size();
			size.width *= size.height;
			size.height = 1;
			//these are gray images
			const unsigned char *ptr_phase1 = three_image_gray1.ptr<unsigned char>(0);
			const unsigned char *ptr_phase2 = three_image_gray2.ptr<unsigned char>(0);
			const unsigned char *ptr_phase3 = three_image_gray3.ptr<unsigned char>(0);
			unsigned char *ptr_mask = three_phase_mask.ptr<unsigned char>(0);
			unsigned char *ptr_process = three_unwrap_process.ptr<unsigned char>(0);
			double *ptr_range = three_phase_range.ptr<double>(0);
			//these are color images
			const unsigned char *ptr_image1 = three_image1.ptr<unsigned char>(0);
			const unsigned char *ptr_image2 = three_image2.ptr<unsigned char>(0);
			const unsigned char *ptr_image3 = three_image3.ptr<unsigned char>(0);
			unsigned char *ptr_color = three_phase_color.ptr<unsigned char>(0);
			for (int i = 0; i < size.width; ++i){
				phase1_value = ptr_phase1[i];		//read operation cost 15ms
				phase2_value = ptr_phase2[i];
				phase3_value = ptr_phase3[i];
				phase_min = Min(phase1_value, phase2_value, phase3_value);  //max,min cost 45ms
				phase_max = Max(phase1_value, phase2_value, phase3_value);
				phase_range = phase_max - phase_min;
				phase_sum = phase1_value + phase2_value + phase3_value;
							
				if (phase_sum != 0){
					signal_noise_ratio = phase_range / phase_sum;
					if ((phase_range>three_pixel_range_threeshold)&&(signal_noise_ratio > three_noise_threshold)){
						ptr_mask[i] = 255;
						ptr_process[i] = 255;
						ptr_range[i] = phase_range;
						unsigned int temp = ptr_image1[i * 3] + ptr_image2[i * 3] + ptr_image3[i * 3];
						ptr_color[i * 3] = temp / 3;
						temp = ptr_image1[i * 3 + 1] + ptr_image2[i * 3 + 1] + ptr_image3[i * 3 + 2];
						ptr_color[i * 3 + 1] = temp / 3;
						temp = ptr_image1[i * 3 + 2] + ptr_image2[i * 3 + 2] + ptr_image3[i * 3 + 2];
						ptr_color[i * 3 + 2] = temp / 3;
					}
				}
			}
		}
	}
	else if (phaseshift_scan_mode == PHASESHIFT_RGB){
		return 1;
	}
	return 1;
}

// 解相位主值
int PhaseShift::ThreePhaseDecodeAtan()
{
	double sqrt3 = sqrt(3);
	double TWO_PI = 2 * M_PI;
	if (three_phase_mask.isContinuous()){
		Size size = three_phase_mask.size();
		size.width *= size.height;
		size.height = 1;
		const unsigned char *ptr_phase1 = three_image_gray1.ptr<unsigned char>(0);
		const unsigned char *ptr_phase2 = three_image_gray2.ptr<unsigned char>(0);
		const unsigned char *ptr_phase3 = three_image_gray3.ptr<unsigned char>(0);
		const unsigned char *ptr_mask = three_phase_mask.ptr<unsigned char>(0);
		double *ptr_wrapped = three_wrapped_atan.ptr<double>(0);
		for (int i = 0; i < size.width; ++i){
			if (ptr_mask[i] != 0){
				ptr_wrapped[i] = (double)atan2((double)sqrt3*(ptr_phase1[i] - ptr_phase3[i]), (double)(2 * ptr_phase2[i] - ptr_phase1[i] - ptr_phase3[i]))/TWO_PI;
			}
			else{
				ptr_wrapped[i] = 0.0f;
			}
		}
	}
	return 1;
}

int PhaseShift::ThreePhaseDecodeFast()
{
	return 1;
}

int PhaseShift::ThreePhaseComputeQuality()
{
	if (three_phase_mask.isContinuous()){
		Size size = three_phase_mask.size();
		int step = size.width;
		size.width *= size.height;
		
		const unsigned char *ptr_mask = three_phase_mask.ptr<unsigned char>(0);	 
		const double *ptr_range = three_phase_range.ptr<double>(0);
		const double *ptr_phase = three_wrapped_atan.ptr<double>(0);
		double *ptr_quality = three_phase_quality.ptr<double>(0);				
		for (int c = 0; c < size.width - 1; ++c){						
			if (ptr_mask[c] != 0){
				ptr_quality[c] =(DistSquare(ptr_phase[c], ptr_phase[c - 1])	  //the bigger,the better
					+ DistSquare(ptr_phase[c], ptr_phase[c + 1])
					+ DistSquare(ptr_phase[c], ptr_phase[c - step])
					+ DistSquare(ptr_phase[c], ptr_phase[c + step])) ;
				three_quality_average += ptr_quality[c];
				three_quality_count++;
			}
		}
		three_quality_average /= three_quality_count;
		for (int c = 1; c < size.width - 1; ++c){
			if (ptr_mask[c] != 0){
				three_quality_deviation += (ptr_quality[c] - three_quality_average)*(ptr_quality[c] - three_quality_average);
			}
		}
		three_quality_deviation = sqrt(three_quality_deviation / three_quality_count);
	}

	return 1;
}

//三步相移解包裹
int PhaseShift::ThreePhaseUnwrapBasedQuality()
{
	unsigned char *ptr_process = three_unwrap_process.ptr<unsigned char>(0);
	double *ptr_wrapped = three_wrapped_atan.ptr<double>(0);
	double *ptr_quality = three_phase_quality.ptr<double>(0);
	double *ptr_unwrapped = three_unwrapped_quality.ptr<double>(0);

	int x = image_width/2, y =image_height/2 ;  //start from centroid of image
	int step = image_width;
	double quality = ptr_quality[y*step + x];
	double phase = ptr_wrapped[y*step+x];
	UnWrapPath path = UnWrapPath(x, y, quality, phase);
	unwrap_queue.push(path);

	while (!unwrap_queue.empty()){
		path = unwrap_queue.top();
		unwrap_queue.pop();
		x = path.x;
		y = path.y;
		quality = path.quality;
		phase = path.phase;
		if (ptr_process[y*step + x]){
			ptr_process[y*step + x] = 0;
			ptr_unwrapped[y*step + x] = path.phase;
			if ((x > 0) && (ptr_process[y*step + x-1]))
				ThreePhaseUnwrapBasedQuality(x - 1, y, quality, phase);
			if ((x < image_width - 1) && (ptr_process[y*step + x + 1]))
				ThreePhaseUnwrapBasedQuality(x + 1, y, quality, phase);
			if ((y > 0) && (ptr_process[(y-1)*step + x]))
				ThreePhaseUnwrapBasedQuality(x, y-1, quality, phase);
			if ((y < image_height - 1) && (ptr_process[(y+1)*step + x]))
				ThreePhaseUnwrapBasedQuality(x, y+1, quality, phase);
		}
	}
	return 1;
}

int PhaseShift::ThreePhaseUnwrapBasedQuality(int x, int y, double quality, double phase)
{
	double *ptr_wrapped = three_wrapped_atan.ptr<double>(0);
	double *ptr_quality = three_phase_quality.ptr<double>(0);
	double adjacent_phase_diff = ptr_wrapped[y*image_width + x] - (phase - (int)phase);
	if (adjacent_phase_diff > 0.5f)
		adjacent_phase_diff -= 1.0f;
	if (adjacent_phase_diff < -0.5f)
		adjacent_phase_diff += 1.0f;
	unwrap_queue.push(UnWrapPath(x, y, ptr_quality[y*image_width + x]+quality, phase+adjacent_phase_diff));
	return 1;
}

int PhaseShift::ThreePhaseUnwrapBasedMultiLevelQuality(const double quality_threshold)
{
	const unsigned char *ptr_mask = three_phase_mask.ptr<unsigned char>(0);
	unsigned char *ptr_processed = three_unwrap_process.ptr<unsigned char>(0);
	const double *ptr_wrapped = three_wrapped_atan.ptr<double>(0);
	const double *ptr_quality = three_phase_quality.ptr<double>(0);
	double *ptr_unwrapped = three_unwrapped_quality.ptr<double>(0);

	int start_x = image_width / 2, start_y = image_height / 2;
	int step = image_width;
	priority_queue<UnWrapPath> queue_temp;
	//find one point around image's centroid as start point
	for (int i = -2; i < 3; ++i){
		start_x = image_width/2+i;
		for (int j = -2; j < 3; ++j){
			start_y = image_height / 2 + j;
			if (ptr_mask[start_y*step + start_x])
				queue_temp.push(UnWrapPath(start_x, start_y, ptr_quality[start_y*step + start_x], ptr_wrapped[start_y*step + start_x]));
		}
	}
	UnWrapPath start_point = queue_temp.top();
	if (start_point.quality < quality_threshold){ return 0; }	//start point's quality is lower than quality_threshold
	ptr_processed[start_point.y*step + start_point.x] = 255;
	ptr_unwrapped[start_point.y*step + start_point.x] = start_point.phase;

	//scan the first region 
	for (int r = start_point.y; r > 0; --r){
		for (int c = start_point.x; c > 0; --c){
			if ((ptr_mask[r*step + c])&&(!ptr_processed[r*step+c])&&(ptr_quality[r*step+c]>quality_threshold)){	
				//choose unwrapped point facing start point as reference
				if (ptr_processed[r*step + c + 1]){				//processed points' quality is higher than quality threshold  
						ThreePhaseUnwrapBasedMultiLevelQuality(c + 1, r, c, r);
						ptr_processed[r*step + c] = 255;
					}
				else if (ptr_processed[(r+1)*step + c]){
						ThreePhaseUnwrapBasedMultiLevelQuality(c, r + 1, c, r);
						ptr_processed[r*step + c ] = 255;
					}
				}
			}
		}
	//scan the second region 
	for (int r = start_point.y; r > 0; --r){
		for (int c = start_point.x+1; c <image_width-1 ; ++c){
			if ((ptr_mask[r*step + c]) && (!ptr_processed[r*step + c]) && (ptr_quality[r*step + c]>quality_threshold)){
				//choose unwrapped point facing start point as reference
				if (ptr_processed[r*step + c - 1]){
					ThreePhaseUnwrapBasedMultiLevelQuality(c - 1, r, c, r);
					ptr_processed[r*step + c] = 255;
				}
				else if (ptr_processed[(r + 1)*step + c]){
					ThreePhaseUnwrapBasedMultiLevelQuality(c, r + 1, c, r);
					ptr_processed[r*step + c] = 255;
				}
			}
		}
	}
	//scan the third region 
	for (int r = start_point.y+1; r < image_height-1; ++r){
		for (int c = start_point.x; c <image_width-1; ++c){
			if ((ptr_mask[r*step + c]) && (!ptr_processed[r*step + c]) && (ptr_quality[r*step + c]>quality_threshold)){
				//choose unwrapped point facing start point as reference
				if (ptr_processed[r*step + c - 1]){
					ThreePhaseUnwrapBasedMultiLevelQuality(c - 1, r, c, r);
					ptr_processed[r*step + c] = 255;
				}
				else if (ptr_processed[(r - 1)*step + c]){
					ThreePhaseUnwrapBasedMultiLevelQuality(c, r - 1, c, r);
					ptr_processed[r*step + c] = 255;
				}
			}
		}
	}
	//scan the fourth region 
	for (int r = start_point.y + 1; r < image_height-1; ++r){
		for (int c = start_point.x-1; c > 0; --c){
			if ((ptr_mask[r*step + c]) && (!ptr_processed[r*step + c]) && (ptr_quality[r*step + c]>quality_threshold)){
				//choose unwrapped point facing start point as reference
				if (ptr_processed[r*step + c + 1]){
					ThreePhaseUnwrapBasedMultiLevelQuality(c + 1, r, c, r);
					ptr_processed[r*step + c] = 255;
				}
				else if (ptr_processed[(r - 1)*step + c]){
					ThreePhaseUnwrapBasedMultiLevelQuality(c, r - 1, c, r);
					ptr_processed[r*step + c] = 255;
				}
			}
		}
	}
	Mat temp = three_phase_mask - three_unwrap_process;
	return 1;
}

int PhaseShift::ThreePhaseUnwrapBasedMultiLevelQuality(int source_x, int source_y, int dst_x, int dst_y)
{
	double *ptr_wrapped = three_wrapped_atan.ptr<double>(0);
	double *ptr_unwrapped = three_unwrapped_quality.ptr<double>(0);
	double adjacent_phase_diff = ptr_wrapped[dst_y*image_width + dst_x] - 
		(ptr_unwrapped[source_y*image_width + source_x] - (int)ptr_unwrapped[source_y*image_width + source_x]);
	if (adjacent_phase_diff > 0.5f)
		adjacent_phase_diff -= 1.0f;
	if (adjacent_phase_diff < -0.5f)
		adjacent_phase_diff += 1.0f;
	ptr_unwrapped[dst_y*image_width + dst_x] = ptr_unwrapped[source_y*image_width + source_x] + adjacent_phase_diff;
	return 1;
}

int PhaseShift::MultiLevelQualitySolveRemain(const double quality_threshold)
{
	unsigned char *ptr_processed = three_unwrap_process.ptr<unsigned char>(0);
	const unsigned char *ptr_mask = three_phase_mask.ptr<unsigned char>(0);
	const double *ptr_wrapped = three_wrapped_atan.ptr<double>(0);
	const double *ptr_quality = three_phase_quality.ptr<double>(0);
	double *ptr_unwrapped = three_unwrapped_quality.ptr<double>(0);

	int step = image_width;
	UnWrapPath ref_point;

	for (int r = 1; r < image_height - 1; ++r){
		for (int c = 1; c < image_width - 1; ++c){
			if ((ptr_mask[r*step + c]) && (!ptr_processed[r*step + c]) && (ptr_quality[r*step + c] > quality_threshold)){
				priority_queue<UnWrapPath> queue_temp;
				if (ptr_processed[r*step + c - 1])	 queue_temp.push(UnWrapPath(c - 1, r, ptr_quality[r*step + c - 1], ptr_wrapped[r*step + c - 1]));
				if (ptr_processed[r*step + c + 1])	 queue_temp.push(UnWrapPath(c + 1, r, ptr_quality[r*step + c + 1], ptr_wrapped[r*step + c + 1]));
				if (ptr_processed[(r - 1)*step + c]) queue_temp.push(UnWrapPath(c, r - 1, ptr_quality[(r - 1)*step + c], ptr_wrapped[(r - 1)*step + c]));
				if (ptr_processed[(r + 1)*step + c]) queue_temp.push(UnWrapPath(c, r + 1, ptr_quality[(r + 1)*step + c], ptr_wrapped[(r + 1)*step + c]));
				if (!queue_temp.empty()){
					ref_point = queue_temp.top();
					ThreePhaseUnwrapBasedMultiLevelQuality(ref_point.x, ref_point.y, c, r);
					ptr_processed[r*step + c] = 255;
				}
			}
		}
	}
	return 1;
}

void PhaseShift::DisplayUnwrapResult()
{
	const unsigned char *ptr_processed = three_unwrap_process.ptr<unsigned char>(0);
	const double *ptr_unwrapped = three_unwrapped_quality.ptr<double>(0);
	unsigned short *ptr_result = three_unwrapped_result.ptr<unsigned short>(0);

	//pay attention to initial value
	double phase_min,phase_max ,phase_scale=0.0f;
	if (phaseshift_scan_mode == VERTICAL){
		phase_min = pattern_width /pattern_period , phase_max = -phase_min;	 
		phase_scale = pattern_width;
	}
	else if (phaseshift_scan_mode == HORIZONTAL){
		phase_min = pattern_height / pattern_period, phase_max = -phase_min;
		phase_scale = pattern_height;
	}
	for (int i = 0; i < image_width*image_height - 1; ++i){
		if (ptr_processed[i]){
			if (ptr_unwrapped[i] < phase_min)	phase_min = ptr_unwrapped[i];
			if (ptr_unwrapped[i] > phase_max)	phase_max = ptr_unwrapped[i];
		}
	}
	phase_scale = phase_scale / (phase_max - phase_min);
	for (int i = 0; i < image_width*image_height - 1; ++i){
		if (ptr_processed[i])
			ptr_result[i] = (ptr_unwrapped[i] - phase_min)*phase_scale;
	}

	//calculate first-derivative
	Mat first_derivative(image_height, image_width, CV_16UC1, Scalar(0));
	Mat second_derivative(image_height, image_width, CV_16UC1, Scalar(0));
	Mat third_derivative(image_height, image_width, CV_16UC1, Scalar(0));
	Mat	abs_temp1, abs_temp2,abs_temp3;
	Sobel(three_unwrapped_result, first_derivative, -1, 1, 0, 3, 1.0, 0.0, 4);
	first_derivative.convertTo(abs_temp1, CV_8UC1, 0.2, 0.0);

	Laplacian(three_unwrapped_result, second_derivative, -1,3, 1.0, 0.0, 4);
	second_derivative.convertTo(abs_temp2, CV_8UC1, 0.2, 0.0);

	Sobel(second_derivative, third_derivative, -1, 1, 0, 3, 1.0, 0.0, 4);
	third_derivative.convertTo(abs_temp3, CV_8UC3, 0.2, 0.0);

}