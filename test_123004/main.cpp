#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <time.h>
#include <queue>

using namespace std;
using namespace cv;

#include "structuredlight.h"
#include "auxiliaryfunctions.h"
#include "calibration.h"
#include "scan.h"
#include "phaseshift.h"

clock_t clock_begin, clock_end;

int main()
{
	SlParameter slparameter(1);
	SlCalibration slcalibration(1);

//	//Initialize projector
	namedWindow("projector window", 0);
	moveWindow("projector window", 1920, 0);
	setWindowProperty("projector window", WND_PROP_FULLSCREEN, 1);
	ProjectorInitialize(slparameter); //投影仪初始化

	GenerateGrayCode(slparameter); //生成格雷码

	//Initialize camera
	CameraInitialize(slparameter); //相机初始化

	//Run camera calibration
//	RunCameraCalibration(slparameter,slcalibration); //相机校正

	//Run projector calibration
//	RunProjectorCalibration(slparameter,slcalibration); //投影仪校正

	//Run camera projector extrinsic calibration
//	RunCameraProjectorExtrinsicCalibration(slparameter, slcalibration); // 相机和投影仪外参校正

	EvaluteCameraProjectorGeometry(slparameter, slcalibration); //估计投影仪几何（位姿估计）

	//Run structuredlight scan
	RunStructuredLight(slparameter, slcalibration); //运行结构光扫描

	// 深度图转换成灰度图
	Mat temp;
	DepthMapConvertToGray(slparameter.depth_points, temp, slparameter.depth_valid, slparameter.distance_range[0], slparameter.distance_range[1]);

	// 三步相移或者RGB相移
	PhaseShift slphaseshift(PHASESHIFT_THREE, VERTICAL);
	slphaseshift.GeneratePhaseShiftPattern(); //生成相移pattern
	slphaseshift.ReadPhaseShiftPattern(); //读取相移pattern
	
	//while (1){
	//	clock_begin = clock();
	//	slphaseshift.ScanObject(true);
	//	clock_end = clock();
	//	cout << clock_end - clock_begin << endl;
	//}
	
	slphaseshift.ReadScanImage(); //读取扫描图像（pattern调制图像）

	slphaseshift.ComputeMaskRegion();  //计算mask区域

	slphaseshift.ThreePhaseDecodeAtan(); //三步相移解相位主值

	slphaseshift.ThreePhaseComputeQuality(); //三步相移计算相位值？？

	//slphaseshift.ThreePhaseUnwrapBasedQuality(); //解包裹

	slphaseshift.three_unwrap_process.setTo(0);
	slphaseshift.ThreePhaseUnwrapBasedMultiLevelQuality(slphaseshift.three_quality_average-slphaseshift.three_quality_deviation);

	slphaseshift.ThreePhaseUnwrapBasedMultiLevelQuality(slphaseshift.three_quality_average - 2*slphaseshift.three_quality_deviation);

	slphaseshift.MultiLevelQualitySolveRemain(slphaseshift.three_quality_average - 15 * slphaseshift.three_quality_deviation);

	slphaseshift.DisplayUnwrapResult();

//	just test phaseshift's result ,use gray code's 3d reconstruct,need to modify 
	slparameter.gray_valid_image = slphaseshift.three_unwrap_process;
	slparameter.decode_colum_scan_image=slphaseshift.three_unwrapped_result;
	slparameter.scene_color = slphaseshift.three_phase_color.clone();
	ReconstructDepthMap(slparameter,slcalibration);

	Mat temp1;
	DepthMapConvertToGray(slparameter.depth_points, temp1, slphaseshift.three_unwrap_process, slparameter.distance_range[0], slparameter.distance_range[1]);

	clock_begin = clock();
	char save_name[] = ".\\output\\phaseshift\\phaseshift_three\\vertical\\image\\pointCloud.x3d";
	SaveX3DFile(save_name, slparameter.depth_points, slparameter.depth_colors, slparameter.depth_valid);
	cout << clock() - clock_begin << endl;

	//Clear ram and camera
    CameraClear();
}






















