#include "UnwrappedPhaseGrayCode.h"

int main(int argc, char **argv) 
{
	PhaseShiftPatternGenerate(pow(2, gray_bit)); //pattern生成

	LoadImage(); //加载图像
	CalWrappedPhase(); //计算包裹相位
	CalUnwrappedPhase(); //解包裹

    return 0;
}

