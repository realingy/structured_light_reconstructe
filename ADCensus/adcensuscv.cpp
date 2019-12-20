#include "adcensuscv.h"

ADCensusCV::ADCensusCV(const cv::Mat &leftImage, const cv::Mat &rightImage, cv::Size censusWin, float lambdaAD, float lambdaCensus)
{
    this->leftImage = leftImage;
    this->rightImage = rightImage;
    this->censusWin = censusWin; //census变换核尺寸
    this->lambdaAD = lambdaAD;	 //AD代价权重
    this->lambdaCensus = lambdaCensus; //Census代价权重
}

// AD匹配代价计算
// 输入：左右图像目标像素位置
// 输出：相似度，值越小相似度越高，作为匹配点的可能性就越高
float ADCensusCV::ad(int wL, int hL, int wR, int hR) const
{
    float dist = 0;
    const cv::Vec3b &colorLP = leftImage.at<cv::Vec3b>(hL, wL);
    const cv::Vec3b &colorRP = rightImage.at<cv::Vec3b>(hR, wR);

    for(uchar color = 0; color < 3; ++color)
    {
        dist += std::abs(colorLP[color] - colorRP[color]);
    }
    return (dist / 3);
}

// census匹配代价计算
// 输入：左右图像目标像素位置
// 输出：相似度，值越小相似度越高，作为匹配点的可能性就越高
float ADCensusCV::census(int wL, int hL, int wR, int hR) const
{
    float dist = 0;
    const cv::Vec3b &colorRefL = leftImage.at<cv::Vec3b>(hL, wL); //左图目标点p
    const cv::Vec3b &colorRefR = rightImage.at<cv::Vec3b>(hR, wR); //右图目标点q

	//遍历census窗口像素，估算窗口内各个位置像素相似度，相加作为目标点的相似度
    for(int h = -censusWin.height / 2; h <= censusWin.height / 2; ++h)
    {
        for(int w = -censusWin.width / 2; w <= censusWin.width / 2; ++w)
        {
            const cv::Vec3b &colorLP = leftImage.at<cv::Vec3b>(hL + h, wL + w);
            const cv::Vec3b &colorRP = rightImage.at<cv::Vec3b>(hR + h, wR + w);
            for(uchar color = 0; color < 3; ++color)
            {
                // bool diff = (colorLP[color] < colorRefL[color]) ^ (colorRP[color] < colorRefR[color]);
                bool diff = (colorLP[color] - colorRefL[color]) * (colorRP[color] - colorRefR[color]) < 0;
                dist += (diff)? 1: 0;
            }
        }
    }

    return dist;
}

// AD和Census匹配代价结合
// 输入：目标点在左右图像中的搜索位置
// 输出：目标点在左右图像中的相似度，值越小相似度越高，作为匹配点的可能性就越高
float ADCensusCV::adCensus(int wL, int hL, int wR, int hR) const
{
    float dist;

    // compute Absolute Difference cost
    float cAD = ad(wL, hL, wR, hR);

    // compute Census cost
    float cCensus = census(wL, hL, wR, hR);

    // combine the two costs
	// 将两种匹配代价进行归一化，并相加
    dist = 1 - exp(-cAD / lambdaAD); 
    dist += 1 - exp(-cCensus / lambdaCensus);

    return dist;
}
