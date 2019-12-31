#ifndef _PHASESHIFT_H
#define _PHASESHIFT_H

#include <deque>
#include <queue>

using namespace std;
using namespace cv;

enum PhaseShiftMode{
	PHASESHIFT_THREE = 0,
	PHASESHIFT_RGB
};

enum PhaseShiftScanMode{
	VERTICAL=0,
	HORIZONTAL
};

class UnWrapPath{
public:
	UnWrapPath() = default;
	UnWrapPath(int x, int y, double q, double p) :x(x), y(y), quality(q), phase(p){}
	int x = 0, y = 0;
	double quality = 0.0f;
	double phase = 0.0f;
	bool operator<(const UnWrapPath &m)const { return quality < m.quality; }
};

class PhaseShift
{
public:
	PhaseShift()= default;
	PhaseShift(PhaseShiftMode m1, PhaseShiftScanMode m2);
	//image parameters
	int image_width=1024;	//capture image width 	
	int image_height=768;	//capture iamge height

	//pattern parameters
	int pattern_width=1024;	 //phaseshift pattern width
	int pattern_height=768;	 //phaseshift pattern height
	int pattern_period=64;   //phaseshift pattern period

	//gain parameters
	int image_gain=100;		//0-0.00,100-1.00,200-2.00
	int pattern_gain=100;   //0-0.00,100-1.00,200-2.00

	//scan and compute parameters
	int project_capture_delay=300;	 //frame delay between projection and image capture (in ms)
	int   three_pixel_range_threeshold = 10; //pixel's value range ,use for compute mask region  
	float three_noise_threshold = 0.20;	 //three phaseshift signal noise ratio threshold,0.00~1.00,use for compute mask region

	//define parameter for multi level compute;
	double three_quality_average = 0.0f;	//quality_map's average value,use for multi-level quality map
	double three_quality_deviation = 0.0f;	//quality_map's standard deviation,use for multi-level quality map
	double three_quality_count = 0.0f;		//quality_map's vaild pixel amount,use for multi-level quality map

	//projected pattern 
	Mat three_pattern_phase1;
	Mat three_pattern_phase2;
	Mat three_pattern_phase3;
	Mat rgb_pattern;

	//captured image 
	Mat three_image1,three_image_gray1;
	Mat three_image2,three_image_gray2;
	Mat three_image3,three_image_gray3;
	Mat rgb_image;

	//image mask region 
	Mat three_phase_mask;		//mask region,determine operating one pixel or not
	Mat three_unwrap_process;   //equal to mask region initially,record unwrapped or not 
	Mat three_phase_range;		//gray-value range of each pixel
	Mat three_phase_color;		//recovered color information from three image
	
	//decoded phase
	Mat three_wrapped_atan;
	Mat three_wrapped_fast;
	
	//quality map based on wrapped phase 
	Mat three_phase_quality;

	//unwrapped phase
	Mat three_unwrapped_quality;	//initial result(not start from zero,may including negative)
	Mat three_unwrapped_result;	    //covert from other result(start from zero and all are positive)

	int GeneratePhaseShiftPattern();
	int ReadPhaseShiftPattern();
	int ScanObject(bool save_enable=true);
	int ReadScanImage();

	int ComputeMaskRegion();

	int ThreePhaseDecodeAtan();
	int ThreePhaseDecodeFast();

	int ThreePhaseComputeQuality();

	int ThreePhaseUnwrapBasedQuality();

	int ThreePhaseUnwrapBasedMultiLevelQuality(const double quality_threshold);
	int MultiLevelQualitySolveRemain(const double quality_threshold);

	void DisplayUnwrapResult();

private:
	PhaseShiftMode phaseshift_mode=PHASESHIFT_THREE;
	PhaseShiftScanMode phaseshift_scan_mode=VERTICAL;
	priority_queue<UnWrapPath> unwrap_queue;
	void Init();
	template<typename T>
    T Max(T a, T b, T c){ return a > b ? (a > c ? a : c) : (b > c ? b : c); }
	template<typename T>
    T Min(T a, T b, T c){ return a < b ? (a < c ? a : c) : (b < c ? b : c); }
	template<typename T>
	T DistSquare(T a, T b){ T temp = a > b ? (a - b) : (b - a);  return (temp<0.5?(1-temp):temp); }   
	int ThreePhaseUnwrapBasedQuality(int x, int y, double quality, double phase);
	int PhaseShift::ThreePhaseUnwrapBasedMultiLevelQuality(int source_x, int source_y, int dst_x, int dst_y);
};


#endif