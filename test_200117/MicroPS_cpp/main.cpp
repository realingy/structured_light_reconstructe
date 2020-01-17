#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

using namespace std;

string dirname = "../Data/WaxBowl/32-08"; // Directory containing the captured input images
string projDirname = "../ProjectedImages/32-08"; // Directory containing the frequency information of the projected images (freqData.mat)

// load([projDirname, '\freqData.mat']);

string imPrefix = "Frame"; // Prefix of the captured images
string imSuffix = ".tiff"; // Suffix of the captured images
int indexLength = 3;       // Length of the image index. For example, if images are numbered Frame1.tiff, Frame2.tiff,..., indexLength is 1. If images are numbered Frame001.tiff, Frame002.tiff,..., indexLength is 3.

cv::Size projDim = cv::Size(768, 1024); // Dimensions of the projector image (numRows, numColumns).
cv::Size camDim = cv::Size(801, 900);  // Dimensions of the camera image (numRows, numColumns).
//int medfiltParam    = [5 5];                                                        % The computed correspondence map is median filtered to mitigate noise. These are the median filter parameters. Usual values are between [1 1] to [7 7], depending on image noise levels, number of images used, and the frequencies. Use smaller values of these parameters for low noise levels, large number of input images, and low frequencies. For example, if the average frequency is 64 pixels, and 15 frequencies are used, use medFiltParam = [1 1]. On the other hand, if the average frequency is 16 pixels, and 5 frequencies are used, use medFiltParam = [7 7].
//中值滤波卷积核尺寸
int medfiltParam = 5; // The computed correspondence map is median filtered to mitigate noise. These are the median filter parameters. Usual values are between [1 1] to [7 7], depending on image noise levels, number of images used, and the frequencies. Use smaller values of these parameters for low noise levels, large number of input images, and low frequencies. For example, if the average frequency is 64 pixels, and 15 frequencies are used, use medFiltParam = [1 1]. On the other hand, if the average frequency is 16 pixels, and 5 frequencies are used, use medFiltParam = [7 7].

cv::Mat IC = MicroPhaseShiftingDecodeFunc(dirname, imPrefix, imSuffix, indexLength, frequencyVec, camDim(1), camDim(2), projDim(2)); // The main decoding function. This function computes the corresponding projector column (sub-pixel) for each camera pixel. IC is the same size as the input images
//中值滤波处理
IC = medfilt2(IC, medfiltParam); // Applying median filtering

// Saving and displaying the results
string outdirname = [dirname, '\Results'];
mkdir(outdirname)
save([outdirname, '\IC.mat'], 'IC');
save([outdirname, '\IC.txt'],'IC','-ASCII')

figure;
imagesc(IC, [500, 950]);
colormap(jet(256));
colorbar