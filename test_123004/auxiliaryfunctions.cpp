#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <direct.h>
#include <time.h>

#include "structuredlight.h"
#include "auxiliaryfunctions.h"

using namespace std;
using namespace cv;
//use mv-ub500 sdk api function
#include <process.h>
#include "windows.h"
#pragma comment(lib,"..\\structuredLight\\MVCAMSDK.lib")
#include "CameraApi.h"

CameraSdkStatus camera_sdk_status;	//相机状态
INT  CameraNums = 1;				//设置iCameraNums = 1，表示最多只读取1个设备，如需枚举更多的设备，更改sCameraList大小和iCameraNums的值
tSdkCameraDevInfo CameraList[1];	//设备列表数组指针
CameraHandle    m_hCamera;			//相机句柄，多个相机同时使用时，可以用数组代替	
tSdkCameraCapbility CameraInfo;		//相机特性描述结构体
tSdkFrameHead 	FrameInfo;			//图像的帧头信息指针
BYTE*			PbyBuffer;			//指向原始图像数据的缓冲区指针
BYTE*           FrameBuffer;		//将原始图像数据转换为RGB图像的缓冲区

IplImage *iplImage = NULL;

//注意相机输出的是上一次触发捕获指令缓存的图像
int GetImage(Mat &frame_grab)
{
	clock_t clock_begin;
	clock_begin = clock();
	if (CameraGetImageBuffer(m_hCamera, &FrameInfo, &PbyBuffer, 200) == CAMERA_STATUS_SUCCESS)
	{
		////将获得的原始数据转换成RGB格式的数据，同时经过ISP模块，对图像进行降噪，边沿提升，颜色校正等处理。
		camera_sdk_status = CameraImageProcess(m_hCamera, PbyBuffer, FrameBuffer, &FrameInfo);//连续模式
		if (camera_sdk_status == CAMERA_STATUS_SUCCESS)
		{
			//转换数据并显示
			iplImage = cvCreateImageHeader(cvSize(FrameInfo.iWidth, FrameInfo.iHeight), IPL_DEPTH_8U, 3);
			cvSetData(iplImage, FrameBuffer, FrameInfo.iWidth * 3);
			//cvShowImage("camera", iplImage);
			//Mat frame_temp = Mat(iplImage, true);
			Mat frame_temp = cvarrToMat(iplImage);
			frame_grab=frame_temp.clone();
		}
		//在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
		CameraReleaseImageBuffer(m_hCamera, PbyBuffer);
		cvReleaseImageHeader(&iplImage);
		cout << clock() - clock_begin <<" ";
		return -1;
	}
	else
		return 0;
	
}

//Initialize camera
int CameraInitialize(SlParameter &sl_parameter)
{
//	cv::Mat frame_grab;
	//use usb webcam
//	VideoCapture videocapture(sl_parameter.camera_id);
//	if (!videocapture.isOpened())
//	{
//		cerr << "Failed to open camera" << endl;
//		return -1;
//	}
//	cout << "Initialize camera------------------------------------------" << endl;
//	//set properties of the camera
//	videocapture.set(CV_CAP_PROP_FRAME_WIDTH, sl_parameter.camera_width);
//	videocapture.set(CV_CAP_PROP_FRAME_HEIGHT, sl_parameter.camera_height);
//
//	namedWindow("Video in real-time", WINDOW_NORMAL);
//	while (1)
//	{
//		videocapture >> frame_grab;
//		imshow("Video in real-time", frame_grab);
//		waitKey(50);
//#ifdef DEBUG_PROJECT
//		cout << "Camera Properties:" << endl;
//		cout << "camera id: " << sl_parameter.camera_id << endl;
//		cout << "frame rate: " << videocapture.get(CV_CAP_PROP_FPS) << endl;
//		cout << "width: " << videocapture.get(CV_CAP_PROP_FRAME_WIDTH) << endl;
//		cout << "height: " << videocapture.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;
//		cout << "brightness: " << videocapture.get(CV_CAP_PROP_BRIGHTNESS) << endl;
//		cout << "contrast: " << videocapture.get(CV_CAP_PROP_CONTRAST) << endl;
//		cout << "saturation: " << videocapture.get(CV_CAP_PROP_SATURATION) << endl;
//		cout << "hue: " << videocapture.get(CV_CAP_PROP_HUE) << endl;
//		cout << "gain: " << videocapture.get(CV_CAP_PROP_GAIN) << endl;
//		cout << "exposure: " << videocapture.get(CV_CAP_PROP_EXPOSURE) << endl;
//#endif
//		cout << "-------------------------------------------------------" << endl << endl;
//	}

	//use industry camera mv ub500
	Mat frame_grab;
	//相机SDK初始化
	if ((camera_sdk_status= CameraSdkInit(1)) != CAMERA_STATUS_SUCCESS)
	{
		cout << "Camera sdk init failed: " << camera_sdk_status<<endl;
		return -1;
	}
	//枚举设备，获得设备列表
	if ((camera_sdk_status=CameraEnumerateDevice(CameraList, &CameraNums)) != CAMERA_STATUS_SUCCESS || CameraNums == 0)
	{
		cout << "No camera was found: " << camera_sdk_status << endl;
		return -1;
	}
	//初始化设备
	if ((camera_sdk_status  = CameraInit(&CameraList[0], -1, -1, &m_hCamera)) != CAMERA_STATUS_SUCCESS)
	{
		cout << "Camera  init failed: " << camera_sdk_status << endl;
		return -1;
	}
	//初始化相机的特性描述
	CameraGetCapability(m_hCamera, &CameraInfo);
	FrameBuffer = (BYTE *)malloc(CameraInfo.sResolutionRange.iWidthMax*CameraInfo.sResolutionRange.iWidthMax * 3);
	//进入图像采集模式
	CameraPlay(m_hCamera);
	waitKey(2000);			//wait for camera start
	//for (int i = 0; i < 5; i++)
	//{
	//	GetImage(frame_grab);
	//	if (!frame_grab.empty())
	//		imshow("camera initialize", frame_grab);
	//	waitKey(200);
	//}
	GetImage(frame_grab);
	if (!frame_grab.empty())
		cout << "Camera initialize successful......" <<endl<<endl;
	else
		cout << "Camera initializa failed......" << endl<<endl;
//	destroyWindow("camera initialize");
	
	return 0;
}

void CameraClear(void)
{
	CameraUnInit(m_hCamera);
}

//Initialize projector
int ProjectorInitialize(SlParameter &sl_parameter)
{
	//show full white projector background pattern
	sl_parameter.projector_background_pattern = Mat::zeros(sl_parameter.projector_height, sl_parameter.projector_width, CV_8U);
	sl_parameter.projector_background_pattern.setTo(255);
	imshow("projector window", sl_parameter.projector_background_pattern);
	waitKey(1000);
	cout << "projector initialize successful......" << endl << endl;
	return 0;
}






