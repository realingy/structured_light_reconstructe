#ifndef _SCAN_H
#define _SCAN_H


void ColorizeWinter(Mat &src, Mat &dst, Mat &mask);
void DepthMapConvertToGray(const Mat &src, Mat &dst, const Mat &mask,int depth_min, int depth_max);
int SaveX3DFile(char *filename, Mat &points, Mat &colors, Mat &mask);

int GenerateGrayCode(SlParameter &sl_parameter);
int ReadGrayCode(SlParameter &sl_parameter);

int CaptureLivingImage(SlParameter &sl_parameter);

int RunScanObject(SlParameter &sl_parameter,bool save_enable);
int ReadScanImage(SlParameter &sl_parameter);


int DecodeGrayCode(SlParameter &sl_parameter);
int DisplayDecodeResult(SlParameter &sl_parameter);
int IntersectLineWithPlane3D(double *ql, double *qv, double *w, double *intersect_point, double &namate);

int ReconstructDepthMap(SlParameter &sl_parameter, SlCalibration &sl_calibration);

int RunStructuredLight(SlParameter &sl_parameter, SlCalibration &sl_calibration);

#endif
