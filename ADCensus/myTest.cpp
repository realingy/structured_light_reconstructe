#include <boost/filesystem.hpp>
#include <boost/date_time.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <cstdlib>
#include <iomanip>
#include <opencv2/core/core.hpp>  
#include "stereoprocessor.h"
#include "imageprocessor.h"

using namespace std;
// using namespace cv;

bool loadImageList(string file, vector<string> &list)
{
    bool loadSuccess = false;
    cv::FileNode fNode;
    cv::FileStorage fStorage(file, cv::FileStorage::READ);
	if (!fStorage.isOpened())
    {
        cout << "Could not open the configuration file: \"" << file << "\"" << endl;
        return false;
    }
	else
    {
        fNode = fStorage.getFirstTopLevelNode();

        if (fNode.type() == cv::FileNode::SEQ)
        {
            for(cv::FileNodeIterator iterator = fNode.begin(); iterator != fNode.end(); ++iterator)
            {
                list.push_back((string) *iterator);
            }

            loadSuccess = true;
        }
    }

    return loadSuccess;
}

bool loadImages(vector<string> fileList, vector<cv::Mat> &images)
{
    bool emptyImage = false;
    for (int i = 0; i < fileList.size() && !emptyImage; ++i)
    {
        cv::Mat curImg = cv::imread(fileList[i]);
        if (!curImg.empty())
        {
            images.push_back(curImg);
        }
        else
        {
            emptyImage = true;
        }
    }

    return (!emptyImage && images.size() > 0);
}

bool loadQMatrix(string file, cv::Mat &Q)
{
    bool success = false;
    try
    {
        cv::FileStorage fStorage;
		fStorage.open(file, cv::FileStorage::READ);
        fStorage["Q"] >> Q;
        fStorage.release();
        success = true;
    }
    catch(cv::Exception ex)
    {
    }

    return success;
}

/*
void createAndSavePointCloud(cv::Mat &disparity, cv::Mat &leftImage, cv::Mat &Q, string filename)
{
    pcl::PointCloud<pcl::PointXYZRGB> pointCloud;

    // Read out Q Values for faster access
    double Q03 = Q.at<double>(0, 3);
    double Q13 = Q.at<double>(1, 3);
    double Q23 = Q.at<double>(2, 3);
    double Q32 = Q.at<double>(3, 2);
    double Q33 = Q.at<double>(3, 3);

    for (int i = 0; i < disparity.rows; i++)
    {
        for (int j = 0; j < disparity.cols; j++)
        {
            // Create a new point
            pcl::PointXYZRGB point;

            // Read disparity
            float d = disparity.at<float>(i, j);
            if ( d <= 0 ) continue; //Discard bad pixels

            // Read color
            cv::Vec3b colorValue = leftImage.at<cv::Vec3b>(i, j);
            point.r = static_cast<int>(colorValue[2]);
            point.g = static_cast<int>(colorValue[1]);
            point.b = static_cast<int>(colorValue[0]);

            // Transform 2D -> 3D and normalise to point
            double x = Q03 + j;
            double y = Q13 + i;
            double z = Q23;
            double w = (Q32 * d) + Q33;
            point.x = -x / w;
            point.y = -y / w;
            point.z = z / w;

            // Put point into the cloud
            pointCloud.points.push_back (point);
        }
    }

    // Resize PCL and save to file
    pointCloud.width = pointCloud.points.size();
    pointCloud.height = 1;
    pcl::io::savePCDFileASCII(filename, pointCloud);
}
*/

int main(int argc, char *argv[])
{
    bool readSuccessfully = true;
   // libconfig::Config cfg;

    bool success = false;

    string xmlImages, ymlExtrinsic;
    uint dMin; uint dMax; cv::Size censusWin; float defaultBorderCost;
    float lambdaAD; float lambdaCensus; string savePath; uint aggregatingIterations;
    uint colorThreshold1; uint colorThreshold2; uint maxLength1; uint maxLength2; uint colorDifference;
    float pi1; float pi2; uint dispTolerance; uint votingThreshold; float votingRatioThreshold;
    uint maxSearchDepth; uint blurKernelSize; uint cannyThreshold1; uint cannyThreshold2; uint cannyKernelSize;

    if(readSuccessfully)
	{
        if(readSuccessfully)
        {
				dMin = 0;
                dMax = 60;
                xmlImages = "images.xml";
                ymlExtrinsic ="../sample/extrinsics.yml";
                censusWin.height = 9;
                censusWin.width = 7;
                defaultBorderCost = 0.999;
                lambdaAD = 10.0; // TODO Namen anpassen
                lambdaCensus = 30.0;
                savePath = "results/";
                aggregatingIterations = 4;
                colorThreshold1 = 20;
                colorThreshold2 = 6;
                maxLength1 =34;
                maxLength2 =17;
                colorDifference = 15;
                pi1 = 0.1;
                pi2 = 0.3;
                dispTolerance = 0;
                votingThreshold = 20;
                votingRatioThreshold = 0.4;
                maxSearchDepth = 20;
                blurKernelSize = 3;
                cannyThreshold1 = 20;
                cannyThreshold2 = 60;
                cannyKernelSize = 3;
        }

        if(readSuccessfully)
        {
            vector<string> fileList;
            vector<cv::Mat> images;
            cv::Mat Q(4, 4, CV_64F);

            boost::filesystem::path dir(savePath);
            boost::filesystem::create_directories(dir);

			// bool gotExtrinsic =loadQMatrix(ymlExtrinsic, Q);
			bool gotExtrinsic =true;
			// bool imageList=false;
			// imageList=loadImageList(xmlImages, fileList);

            if(loadImageList(xmlImages, fileList))
            {
                if(loadImages(fileList, images))
                {
                    if (images.size() % 2 == 0)
                    {
                        bool error = false;
                        for (int i = 0; i < (images.size() / 2) && !error; ++i)
                        {
                            stringstream file;
                            file << savePath << i;
                            boost::posix_time::ptime start = boost::posix_time::second_clock::local_time();
                            boost::posix_time::ptime end;
                            boost::posix_time::time_duration diff;

                            ImageProcessor iP(0.1);
                            cv::Mat eLeft, eRight;
                            eLeft = iP.unsharpMasking(images[i * 2], "gauss", 3, 1.9, -1);
                            eRight = iP.unsharpMasking(images[i * 2 + 1], "gauss", 3, 1.9, -1);

                            StereoProcessor sP(dMin, dMax, images[i * 2], images[i * 2 + 1], censusWin, defaultBorderCost, lambdaAD, lambdaCensus, file.str(),
                                               aggregatingIterations, colorThreshold1, colorThreshold2, maxLength1, maxLength2,
                                               colorDifference, pi1, pi2, dispTolerance, votingThreshold, votingRatioThreshold,
                                               maxSearchDepth, blurKernelSize, cannyThreshold1, cannyThreshold2, cannyKernelSize);
                            string errorMsg;
                            error = !sP.init(errorMsg);

                            if(!error && sP.compute())
                            {
                                success = true;
                                if(gotExtrinsic)
                                {
                                    cv::Mat disp = sP.getDisparity();

                                    string dispFile = file.str();
                                    dispFile += "_disp.yml";
                                    cv::FileStorage fs(dispFile, cv::FileStorage::WRITE);
                                    fs << "disp" << disp;
                                    fs.release();

                                   /* file << "_cloud.pcd";
                                    createAndSavePointCloud(disp, images[i * 2], Q, file.str());*/
                                }
                                else
                                {
                                    cerr << "[ADCensusCV] Could not create point cloud (no extrinsic)!" << endl;
                                }
                            }
                            else
                            {
                                cerr << "[ADCensusCV] " << errorMsg << endl;
                            }

                            end = boost::posix_time::second_clock::local_time();

                            diff = end - start;

                            cout << "Finished computation after " << setw(2) << right <<  setfill('0') << ((int)(diff.total_seconds() / 3600)) << ":"
                                                                   << setw(2) << right <<  setfill('0') << ((int)((diff.total_seconds() / 60) % 60)) << ":"
                                                                   << setw(2) << right <<  setfill('0') << (diff.total_seconds() % 60) << " (" << diff.total_seconds() << "s) !" << endl;

                        }
                    }
                    else
                    {
                        cerr << "[ADCensusCV] Not an even image number!" << endl;
                    }
                }
                else
                {
                    cerr << "[ADCensusCV] Could not read images!" << endl;
                }
            }
            else
            {
                cerr << "[ADCensusCV] Could not read image list!" << endl;
            }
        }
    }
    else
    {
        cerr << "[ADCensusCV] ADCensusBM <config file>" << endl;
    }

    return (success)? EXIT_SUCCESS: EXIT_FAILURE;
}

