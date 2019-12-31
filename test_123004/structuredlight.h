//Describe: header file for structuredlight.h
//Author: Frank
//Date:   3/9/2015

//#define DEBUG_PROJECT
#undef  DEBUG_PROJECT

#define _CRT_SECURE_NO_WARNINGS

using namespace std;
using namespace cv;

class  SlParameter
{
	public:
		SlParameter() = default;
		SlParameter(int i);
		//output directory
		string output_directory="./output";
		string object_name="test";
		bool save_enable=false;

		//camera parameters
		int camera_id=0;	
		int camera_width=1024;		//camera physical width 	
		int camera_height=768;		//camera physical height

		//projector parameters
		int projector_width=1024;	 //projector physical width
		int projector_height=768;	 //projector physical height
		bool projector_invert=false;

		//gain parameters
		int camera_gain=100;		//0-0.00,100-1.00,200-2.00
		int projector_gain=100;		//0-0.00,100-1.00,200-2.00

		//define camera calibration chessboard parameters
		Size camera_board_size;			//Number of inner corners per a chessboard row and column
		Size camera_square_size;		//The size of a square(unit:mm)
		int frame_amount=10;				//The number of frame to use for calibration

		//define projector calibration chessboard parameters
		Size projector_board_size;			//Number of inner corners per a chessboard row and column
		Size projector_square_size;			//The size of a square(unit:pixel)
		Mat projector_background_pattern;	//use for projector background
		Mat projector_chessboard_pattern;	//use for generate chessboard						
		int projector_border_row=0;			//the i-th row where chessboard pattern start
		int projector_border_colum=0;			//the j-th colum where chessboard pattern start

		//scan flag and threshold 
		bool colum_scan_flag=true;					//enable/disable column scanning
		bool row_scan_flag=false;						//enable/disable row scanning
		int project_capture_delay=300;				//frame delay between projection and image capture (in ms)
		int contrast_threshold=50;					//minimum contrast threshold for decoding (maximum of 255)
		float distance_range[2] ;				//{minimum, maximum} distance(from camera), otherwise point is rejected
		float distance_reject=700;					// rejection distance (for outlier removal) if row and column scanning are both enabled (in mm)
		float background_depth_threshold=20;		// threshold distance for background removal (in mm)	

		//scan option
		int colum_scan_amount=10;					//frames amout if use gray code colum scan
		int row_scan_amount=0;					//frames amount if use gray code row scan
		int colum_scan_shift=0;					//colum beginning of gray code colum scan 
		int row_scan_shift=0;						//row beginning of gray code row scan
		vector<Mat>projector_gray_code_image;	//use for saving gray code to project
		vector<Mat>camera_gray_code_image;		//use for saving frames graped
		Mat decode_colum_scan_image;			//1*M*N,use for saving colum scan decode
		Mat decode_row_scan_image;				//1*M*N,use for saving row scan decode
		Mat gray_valid_image;					//1*M*N,valid decoded pixel;

		Mat depth_valid;		 //1*M*N, valid depth pixel
		Mat depth_namate;	     //1*M*N,decribe each point in parametric form 
		Mat depth_points;		 //1*3*(M*N),points' 3D coordinate under camera coordinate
		Mat scene_color;		 //1*3*(M*N),BGR image used to covert to depth_colors 
		Mat depth_colors;	     //1*3*(M*N),points' color in RGB order which is different from opencv's BGR order
		
		Mat background_depth_namate;	 //1*M*N,decribe each point in parametric form 

//	private:
};

class SlCalibration
{
	public:
		SlCalibration(int i);
		//camera calibration
		Mat camera_intrinsic;		//output 3x3 floating-point camera matrix
		Mat camera_distortion;		//output vector of distortion coefficients
		Mat camera_extrinsic_rotation;
		Mat camera_extrinsic_translation;

		//projector calibration
		Mat projector_intrinsic;
		Mat projector_distortion;
		Mat projector_extrinsic_rotation;
		Mat projector_extrinsic_translation;

		//Flags to indicate calibration status
		bool camera_intrinsic_calibration_flag;
		bool projector_intrinsic_calibration_flag;
		bool camera_projector_extrinsic_calibration_flag;

		//projector-camera geometric parameter
		Mat camera_projection_center;		//1*3*1,camera center of projection
		Mat projector_projection_center;	//1*3*1,projector center of projection
		Mat camera_pixel_rays;				//1*3*(M*N),optical rays for each camera pixel
		Mat projector_pixel_rays;			//1*3*(M*N),optical rays for each projector pixel
		Mat projector_colum_planes;			//1*4*M,plane equations describing every projector column
		Mat projector_row_planes;			//1*4*N,plane equations describing every projector row
//	private:
};

