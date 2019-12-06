#ifndef __UNWRAPPEDPHASEGRAYCODE__
#define __UNWRAPPEDPHASEGRAYCODE__

#include <opencv2/opencv.hpp>
#include <math.h>
#include <time.h>
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

void PhaseShiftPatternGenerate(int freq);

void LoadImage();

// 计算相位主值(包裹相位)
void CalWrappedPhase();

// 解包裹相位(相位展开)
void CalUnwrappedPhase();

#endif // !__UNWRAPPEDPHASEGRAYCODE__
