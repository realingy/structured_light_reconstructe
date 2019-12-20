#include "disparityrefinement.h"

const int DisparityRefinement::DISP_OCCLUSION = 1;
const int DisparityRefinement::DISP_MISMATCH = 2;

// occlusionValue：遮挡值 int
// mismatchValue：误匹配值 int
// dispTolerance：视差容忍 uint
// dMin：最小视差 int
// dMax：最大视差 int
// votingThreshold：投票阈值 uint
// votingRatioThreshold：投票率阈值 float 
// maxSearchDepth：最大搜索深度 uint 
// blurKernelSize：滤波核尺寸 uint 
// cannyThreshold1：canny边缘检测阈值1 uint 
// cannyThreshold2：canny边缘检测阈值2 uint 
// cannyKernelSize：canny边缘检测核尺寸 uint 
DisparityRefinement::DisparityRefinement(uint dispTolerance, int dMin, int dMax,
                                         uint votingThreshold, float votingRatioThreshold, uint maxSearchDepth,
                                         uint blurKernelSize, uint cannyThreshold1, uint cannyThreshold2, uint cannyKernelSize)
{
    this->occlusionValue = dMin - DISP_OCCLUSION;	//最小视差减去1，得到遮挡特征值
    this->mismatchValue = dMin - DISP_MISMATCH;		//最小视差减去2，得到误匹配特征值
    this->dispTolerance = dispTolerance;			//视差容忍
    this->dMin = dMin;								//视差范围之最小视差
    this->dMax = dMax;								//视差范围之最大视差
    this->votingThreshold = votingThreshold;		//区域投票阈值
    this->votingRatioThreshold = votingRatioThreshold; //区域投票率阈值
    this->maxSearchDepth = maxSearchDepth;			//最大搜索深度
    this->blurKernelSize = blurKernelSize;
    this->cannyThreshold1 = cannyThreshold1;
    this->cannyThreshold2 = cannyThreshold2;
    this->cannyKernelSize = cannyKernelSize;
}

// rgb三通道的最大差值
// 输入：匹配目标点p1和p2
// 输出：三通道灰度值最大的差值
int DisparityRefinement::colorDiff(const cv::Vec3b &p1, const cv::Vec3b &p2)
{
    int colorDiff, diff = 0;

    for(uchar color = 0; color < 3; color++)
    {
        colorDiff = std::abs(p1[color] - p2[color]);
        diff = (diff > colorDiff)? diff: colorDiff;
    }

    return diff;
}

// 视差图离群点识别，包括遮挡和误匹配
// 输入：左右视差图
// 输出：校正后的视差图
cv::Mat DisparityRefinement::outlierElimination(const cv::Mat &leftDisp, const cv::Mat &rightDisp)
{
    cv::Size dispSize = leftDisp.size();	//以左视差图为参考，找出其中的离群点
    cv::Mat disparityMap(dispSize, CV_32S); //视差图

	//遍历左视差图，离群点的判定标准是：x-disp小于零，或者左右视差图的差值大于视差容忍
    for(int h = 0; h < dispSize.height; h++)
    {
        for(int w = 0; w < dispSize.width; w++)
        {
            int disparity = leftDisp.at<int>(h, w);

			// 如果是离群点，区分遮挡/不匹配
            if(w - disparity < 0 || abs(disparity - rightDisp.at<int>(h, w - disparity)) > dispTolerance)
            {
                bool occlusion = true;
                for(int d = dMin; d <= dMax && occlusion; d++)
                {
                    if(w - d >= 0 && d == rightDisp.at<int>(h, w - d))
                    {
                        occlusion = false;
                    }
                }
                disparity = (occlusion)? occlusionValue : mismatchValue;
            }

            disparityMap.at<int>(h, w) = disparity;
        }
    }

    return disparityMap;
}

/***************************************************************************
	区域投票法确定P点视差，并建立视差直方图H，当视差d满足公式：
	
	时，将P点标记为视差有效点且其视差等于d

// 视差校正—区域投票法
// 输入：
	upLimits: 存放cv::Mat的vector，
	downLimits: 存放cv::Mat的vector，
	leftLimits: 存放cv::Mat的vector，
	rightLimits: 存放cv::Mat的vector，
	horizatalFirst: bool
// 输出：
	校正后的视差图
***************************************************************************/
void DisparityRefinement::regionVoting(cv::Mat &disparity, const vector<cv::Mat> &upLimits, const vector<cv::Mat> &downLimits,
                                       const vector<cv::Mat> &leftLimits, const vector<cv::Mat> &rightLimits, bool horizontalFirst)
{
	// 临时视差图
    cv::Size dispSize = disparity.size();
    cv::Mat dispTemp(dispSize, CV_32S);

	// 视差直方图
    vector<int> hist(dMax - dMin + 1, 0);

    const cv::Mat *outerLimitsA, *outerLimitsB;
    const cv::Mat *innerLimitsA, *innerLimitsB;

    if(horizontalFirst)
    {
        outerLimitsA = &upLimits[0];
        outerLimitsB = &downLimits[0];
        innerLimitsA = &leftLimits[0];
        innerLimitsB = &rightLimits[0];
    }
    else
    {
        outerLimitsA = &leftLimits[0];
        outerLimitsB = &rightLimits[0];
        innerLimitsA = &upLimits[0];
        innerLimitsB = &downLimits[0];
    }

	// 遍历视差图
    for(size_t h = 0; h < dispSize.height; h++)
    {
        for(size_t w = 0; w < dispSize.width; w++)
        {
            // if the pixel is not an outlier
			// 像素非离群点(视差大于最小视差)
            if(disparity.at<int>(h, w) >= dMin)
            {
                dispTemp.at<int>(h, w) = disparity.at<int>(h, w);
            }
            else
            {
				//离群点
                int outerLimitA = -outerLimitsA->at<int>(h, w);
                int outerLimitB = outerLimitsB->at<int>(h, w);
                int innerLimitA;
                int innerLimitB;
                int vote = 0;
                for(int outer = outerLimitA; outer <= outerLimitB; outer++)
                {
                    if(horizontalFirst)
                    {
                        innerLimitA = -innerLimitsA->at<int>(h + outer, w);
                        innerLimitB = innerLimitsB->at<int>(h + outer, w);
                    }
                    else
                    {
                        innerLimitA = -innerLimitsA->at<int>(h, w + outer);
                        innerLimitB = innerLimitsB->at<int>(h, w + outer);
                    }


                    for(int inner = innerLimitA; inner <= innerLimitB; inner++)
                    {
                        int height, width;
                        if(horizontalFirst)
                        {
                            height = h + outer;
                            width = w + inner;
                        }
                        else
                        {
                            height = h + inner;
                            width = w + outer;
                        }

                        // if the pixel is an outlier, there is no vote to take into account
						// 离群点，没有投票
                        if(disparity.at<int>(height, width) >= dMin)
                        {
                            // increase the number of votes
                            vote++;
                            // update the histogram
                            hist[disparity.at<int>(height, width) - dMin] += 1;
                        }
                    }
                }

                if(vote <= votingThreshold)
                {
                    dispTemp.at<int>(h, w) = disparity.at<int>(h, w);
                }
                else
                {
                    int disp = disparity.at<int>(h, w);
                    float voteRatio;
                    float voteRatioMax = 0;
                    for(int d = dMin; d <= dMax; d++)
                    {
                        voteRatio = hist[d - dMin] / (float)vote;
                        if(voteRatio > voteRatioMax)
                        {
                            voteRatioMax = voteRatio;
                            disp = (voteRatioMax > votingRatioThreshold)? d: disp;
                        }
                        hist[d - dMin] = 0;
                    }
                    dispTemp.at<int>(h, w) = disp;
                }
            }
        }
    }

	// 校正后的视差图拷贝回去
    dispTemp.copyTo(disparity);
}

// 视差图校正—插值法
// 二次线性插值
void DisparityRefinement::properInterpolation(cv::Mat &disparity, const cv::Mat &leftImage)
{
    cv::Size dispSize = disparity.size();
    cv::Mat dispTemp(dispSize, CV_32S);

    // look on the 16 different directions
	// 16个插值方向
    int directionsW[] = {0, 2, 2, 2, 0, -2, -2, -2, 1, 2, 2, 1, -1, -2, -2, -1};
    int directionsH[] = {2, 2, 0, -2, -2, -2, 0, 2, 2, 1, -1, -2, -2, -1, 1, 2};

	// 遍历视差图
    for(size_t h = 0; h < dispSize.height; h++)
    {
        for(size_t w = 0; w < dispSize.width; w++)
        {
            if(disparity.at<int>(h, w) >= dMin)
            {
                dispTemp.at<int>(h, w) = disparity.at<int>(h, w);
            }
            else
            {
				// 邻域视差
                vector<int> neighborDisps(16, disparity.at<int>(h, w));
                vector<int> neighborDiffs(16, -1);
                for(uchar direction = 0; direction < 16; direction++)
                {
                    int hD = h, wD = w;
                    bool inside = true, gotDisp = false;
                    for(uchar sD = 0; sD < maxSearchDepth && inside && !gotDisp; sD++)
                    {
                        // go one step further
                        if(sD % 2 == 0)
                        {
                            hD += directionsH[direction] / 2;
                            wD += directionsW[direction] / 2;
                        }
                        else
                        {
                            hD += directionsH[direction] - directionsH[direction] / 2;
                            wD += directionsW[direction] - directionsW[direction] / 2;
                        }
                        inside = hD >= 0 && hD < dispSize.height && wD >= 0 && wD < dispSize.width;
                        if(inside && disparity.at<int>(hD, wD) >= dMin)
                        {
                            neighborDisps[direction] = disparity.at<int>(hD, wD);
                            neighborDiffs[direction] = colorDiff(leftImage.at<cv::Vec3b>(h, w), leftImage.at<cv::Vec3b>(hD, wD));
                            gotDisp = true;
                        }
                    }

                }

                if(disparity.at<int>(h, w) == dMin - DISP_OCCLUSION)
                {
                    int minDisp = neighborDisps[0];
                    for(uchar direction = 1; direction < 16; direction++)
                    {
                        if(minDisp > neighborDisps[direction])
                            minDisp = neighborDisps[direction];
                    }
                    dispTemp.at<int>(h, w) = minDisp;
                }
                else
                {
                    int minDisp = neighborDisps[0];
                    int minDiff = neighborDiffs[0];
                    for(uchar dir = 1; dir < 16; dir++)
                    {
                        if(minDiff < 0 || (minDiff > neighborDiffs[dir] && neighborDiffs[dir] > 0))
                        {
                            minDisp = neighborDisps[dir];
                            minDiff = neighborDiffs[dir];
                        }
                    }
                    dispTemp.at<int>(h, w) = minDisp;
                }
            }
        }
    }

    dispTemp.copyTo(disparity);
}

// 不连续校正
void DisparityRefinement::discontinuityAdjustment(cv::Mat &disparity, const vector<vector<cv::Mat> > &costs)
{
    cv::Size dispSize = disparity.size();
    cv::Mat dispTemp, detectedEdges, dispGray;

    disparity.copyTo(dispTemp);

    //Edge Detection
    dispGray = convertDisp2Gray(disparity);
    blur(dispGray, detectedEdges, cv::Size(blurKernelSize, blurKernelSize));
    Canny(detectedEdges, detectedEdges, cannyThreshold1, cannyThreshold2, cannyKernelSize);

    int directionsH[] = {-1, 1, -1, 1, -1, 1, 0, 0};
    int directionsW[] = {-1, 1, 0, 0, 1, -1, -1, 1};

    for(size_t h = 1; h < dispSize.height - 1; h++)
    {
        for(size_t w = 1; w < dispSize.width - 1; w++)
        {
            // if pixel is on an edge
            if(detectedEdges.at<uchar>(h, w) != 0)
            {
                int direction = -1;
                if(detectedEdges.at<uchar>(h - 1, w - 1) != 0 && detectedEdges.at<uchar>(h + 1, w + 1) != 0)
                {
                    direction = 0;
                }
                else if(detectedEdges.at<uchar>(h - 1, w + 1) != 0 && detectedEdges.at<uchar>(h + 1, w - 1) != 0)
                {
                    direction = 4;
                }
                else if(detectedEdges.at<uchar>(h - 1, w) != 0 || detectedEdges.at<uchar>(h + 1, w) != 0)
                {
                    if(detectedEdges.at<uchar>(h - 1, w - 1) != 0 || detectedEdges.at<uchar>(h - 1, w) != 0 || detectedEdges.at<uchar>(h - 1, w + 1) != 0)
                        if(detectedEdges.at<uchar>(h + 1, w - 1) != 0 || detectedEdges.at<uchar>(h + 1, w) != 0 || detectedEdges.at<uchar>(h + 1, w + 1) != 0)
                            direction = 2;
                }
                else
                {
                    if(detectedEdges.at<uchar>(h - 1, w - 1) != 0 || detectedEdges.at<uchar>(h, w - 1) != 0 || detectedEdges.at<uchar>(h + 1, w - 1) != 0)
                        if(detectedEdges.at<uchar>(h - 1, w + 1) != 0 || detectedEdges.at<uchar>(h, w + 1) != 0 || detectedEdges.at<uchar>(h + 1, w + 1) != 0)
                            direction = 6;
                }

                if (direction != -1)
                {
                    dispTemp.at<int>(h, w) = dMin - DISP_MISMATCH;

                    int disp = disparity.at<int>(h, w);

                    // select pixels from both sides of the edge
                    direction = (direction + 4) % 8;

                    if(disp >= dMin)
                    {
                        costType cost = costs[0][disp - dMin].at<costType>(h, w);
                        int d1 = disparity.at<int>(h + directionsH[direction], w + directionsW[direction]);
                        int d2 = disparity.at<int>(h + directionsH[direction + 1], w + directionsW[direction + 1]);

                        costType cost1 = (d1 >= dMin)
                                     ? costs[0][d1 - dMin].at<costType>(h + directionsH[direction], w + directionsW[direction])
                                     : -1;

                        costType cost2 = (d2 >= dMin)
                                     ? costs[0][d2 - dMin].at<costType>(h + directionsH[direction + 1], w + directionsW[direction + 1])
                                     : -1;

                        if(cost1 != -1 && cost1 < cost)
                        {
                            disp = d1;
                            cost = cost1;
                        }

                        if(cost2 != -1 && cost2 < cost)
                        {
                            disp = d2;
                        }
                    }

                    dispTemp.at<int>(h, w) = disp;

                }
            }
        }
    }

    dispTemp.copyTo(disparity);
}

// 视差校正—亚像素增强
cv::Mat DisparityRefinement::subpixelEnhancement(cv::Mat &disparity, const vector<vector<cv::Mat> > &costs)
{
    cv::Size dispSize = disparity.size();
    cv::Mat dispTemp(dispSize, CV_32F);

    for(size_t h = 0; h < dispSize.height; h++)
    {
        for(size_t w = 0; w < dispSize.width; w++)
        {
            int disp = disparity.at<int>(h, w);
            float interDisp = disp;

            if(disp > dMin && disp < dMax)
            {
                float cost = costs[0][disp - dMin].at<costType>(h, w) / (float)COST_FACTOR;
                float costPlus = costs[0][disp + 1 - dMin].at<costType>(h, w) / (float)COST_FACTOR;
                float costMinus = costs[0][disp - 1 - dMin].at<costType>(h, w) / (float)COST_FACTOR;

                float diff = (costPlus - costMinus) / (2 * (costPlus + costMinus - 2 * cost));

                if(diff > -1 && diff < 1)
                    interDisp -= diff;
            }

            dispTemp.at<float>(h, w) = interDisp;
        }
    }

	// 中值滤波
    medianBlur(dispTemp, dispTemp, 3);
    return dispTemp;
}

// 视差图转灰度图
cv::Mat DisparityRefinement::convertDisp2Gray(const cv::Mat &disparity)
{
    cv::Size dispSize = disparity.size();
    cv::Mat dispU(dispSize, CV_8U);

    for(size_t h = 0; h < dispSize.height; h++)
    {
        for(size_t w = 0; w < dispSize.width; w++)
        {
            dispU.at<uchar>(h, w) = (disparity.at<int>(h, w) < 0)? 0: (uchar)disparity.at<int>(h, w);
        }
    }

    equalizeHist(dispU, dispU);
    return dispU;
}

