#include "aggregation.h"
using namespace std;

/**********************************************************************
代价聚合
代价聚合分为四步：对所有像素点水平边界计算一次cost，此时累加一次，对所有像素点
垂直边界计算cost，此时累加两次，对所有像素点垂直边界计算cost，此时累加三次，对
所有像素点水平边界计算cost，此时累加四次，完成代价聚合；
**********************************************************************/
Aggregation::Aggregation(const cv::Mat &leftImage, const cv::Mat &rightImage, uint colorThreshold1, uint colorThreshold2,
                         uint maxLength1, uint maxLength2)
{
    this->images[0] = leftImage;
    this->images[1] = rightImage;
    this->imgSize = leftImage.size();
    this->colorThreshold1 = colorThreshold1;
    this->colorThreshold2 = colorThreshold2;
    this->maxLength1 = maxLength1;
    this->maxLength2 = maxLength2;
    this->upLimits.resize(2);
    this->downLimits.resize(2);
    this->leftLimits.resize(2);
    this->rightLimits.resize(2);

    for(uchar imageNo = 0; imageNo < 2; imageNo++)
    {
        upLimits[imageNo] = computeLimits(-1, 0, imageNo);
        downLimits[imageNo] = computeLimits(1, 0, imageNo);
        leftLimits[imageNo] = computeLimits(0, -1, imageNo);
        rightLimits[imageNo] = computeLimits(0, 1, imageNo);
    }
}

int Aggregation::colorDiff(const cv::Vec3b &p1, const cv::Vec3b &p2)
{
    int colorDiff, diff = 0;

    for(uchar color = 0; color < 3; color++)
    {
        colorDiff = std::abs(p1[color] - p2[color]);
        diff = (diff > colorDiff)? diff: colorDiff;
    }

    return diff;
}

int Aggregation::computeLimit(int height, int width, int directionH, int directionW, uchar imageNo)
{
    // reference pixel
    cv::Vec3b p = images[imageNo].at<cv::Vec3b>(height, width);

    // coordinate of p1 the border patch pixel candidate
    int d = 1;
    int h1 = height + directionH;
    int w1 = width + directionW;

    // pixel value of p1 predecessor
    cv::Vec3b p2 = p;

    // test if p1 is still inside the picture
    bool inside = (0 <= h1) && (h1 < imgSize.height) && (0 <= w1) && (w1 < imgSize.width);

    if(inside)
    {
        bool colorCond = true, wLimitCond = true, fColorCond = true;

        while(colorCond && wLimitCond && fColorCond && inside)
        {
            cv::Vec3b p1 = images[imageNo].at<cv::Vec3b>(h1, w1);

            // Do p1, p2 and p have similar color intensities?
            colorCond = colorDiff(p, p1) < colorThreshold1 && colorDiff(p1, p2) < colorThreshold1;

            // Is window limit not reached?
            wLimitCond = d < maxLength1;

            // Better color similarities for farther neighbors?
            fColorCond = (d <= maxLength2) || (d > maxLength2 && colorDiff(p, p1) < colorThreshold2);

            p2 = p1;
            h1 += directionH;
            w1 += directionW;

            // test if p1 is still inside the picture
            inside = (0 <= h1) && (h1 < imgSize.height) && (0 <= w1) && (w1 < imgSize.width);

            d++;
        }

        d--;
    }

    return d - 1;
}

cv::Mat Aggregation::computeLimits(int directionH, int directionW, int imageNo)
{
    cv::Mat limits(imgSize, CV_32S);
    int h, w;
    #pragma omp parallel default (shared) private(w, h) num_threads(omp_get_max_threads())
    #pragma omp for schedule(static)
    for(h = 0; h < imgSize.height; h++)
    {
        for(w = 0; w < imgSize.width; w++)
        {
            limits.at<int>(h, w) = computeLimit(h, w, directionH, directionW, imageNo);
        }
    }

    return limits;
}

cv::Mat Aggregation::aggregation1D(const cv::Mat &costMap, int directionH, int directionW, cv::Mat &windowSizes, uchar imageNo)
{
    cv::Mat tmpWindowSizes = cv::Mat::zeros(imgSize, CV_32S);
    cv::Mat aggregatedCosts(imgSize, CV_32F);
    int dmin, dmax, d;
    int h, w;

    #pragma omp parallel default (shared) private(w, h, d) num_threads(omp_get_max_threads())
    #pragma omp for schedule(static)
    for(h = 0; h < imgSize.height; h++)
    {
        for(w = 0; w < imgSize.width; w++)
        {
            if(directionH == 0)
            {
                dmin = -leftLimits[imageNo].at<int>(h, w);
                dmax = rightLimits[imageNo].at<int>(h, w);
            }
            else
            {
                dmin = -upLimits[imageNo].at<int>(h, w);
                dmax = downLimits[imageNo].at<int>(h, w);
            }

            float cost = 0;
            for(d = dmin; d <= dmax; d++)
            {
                cost += costMap.at<float>(h + d * directionH, w + d * directionW);
                tmpWindowSizes.at<int>(h, w) += windowSizes.at<int>(h + d * directionH, w + d * directionW);
            }
            aggregatedCosts.at<float>(h, w) = cost;
        }
    }

    tmpWindowSizes.copyTo(windowSizes);

    return aggregatedCosts;
}

void Aggregation::aggregation2D(cv::Mat &costMap, bool horizontalFirst, uchar imageNo)
{
    int directionH = 1, directionW = 0;

    if (horizontalFirst)
        std::swap(directionH, directionW);

    cv::Mat windowsSizes = cv::Mat::ones(imgSize, CV_32S);

    for(uchar direction = 0; direction < 2; direction++)
    {
        (aggregation1D(costMap, directionH, directionW, windowsSizes, imageNo)).copyTo(costMap);
        std::swap(directionH, directionW);
    }

    for(size_t h = 0; h < imgSize.height; h++)
    {
        for(size_t w = 0; w < imgSize.width; w++)
        {
            costMap.at<float>(h, w) /= windowsSizes.at<int>(h, w);
        }
    }

}

void Aggregation::getLimits(vector<cv::Mat> &upLimits, vector<cv::Mat> &downLimits,
                            vector<cv::Mat> &leftLimits, vector<cv::Mat> &rightLimits) const
{
    upLimits = this->upLimits;
    downLimits = this->downLimits;
    leftLimits = this->leftLimits;
    rightLimits = this->rightLimits;
}

