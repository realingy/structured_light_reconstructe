#include <iostream>

using namespace std;

// clear all; clc; 

dirname         = ['..\Data\WaxBowl\32-08'];         // Directory containing the captured input images
projDirname     = ['..\ProjectedImages\32-08'];      // Directory containing the frequency information of the projected images (freqData.mat)
load([projDirname, '\freqData.mat']);

imPrefix        = 'Frame';                                                      % Prefix of the captured images
imSuffix        = '.tiff';                                                      % Suffix of the captured images
indexLength     = 3;                                                            % Length of the image index. For example, if images are numbered Frame1.tiff, Frame2.tiff,..., indexLength is 1. If images are numbered Frame001.tiff, Frame002.tiff,..., indexLength is 3.

projDim         = [768 1024];                                                   % Dimensions of the projector image (numRows, numColumns).
camDim          = [801 900];                                                    % Dimensions of the camera image (numRows, numColumns).
medfiltParam    = [5 5];                                                        % The computed correspondence map is median filtered to mitigate noise. These are the median filter parameters. Usual values are between [1 1] to [7 7], depending on image noise levels, number of images used, and the frequencies. Use smaller values of these parameters for low noise levels, large number of input images, and low frequencies. For example, if the average frequency is 64 pixels, and 15 frequencies are used, use medFiltParam = [1 1]. On the other hand, if the average frequency is 16 pixels, and 5 frequencies are used, use medFiltParam = [7 7].


IC              = MicroPhaseShiftingDecodeFunc(dirname, imPrefix, imSuffix, indexLength, frequencyVec, camDim(1), camDim(2), projDim(2));       % The main decoding function. This function computes the corresponding projector column (sub-pixel) for each camera pixel. IC is the same size as the input images
IC              = medfilt2(IC, medfiltParam);                                                                                                   % Applying median filtering

% Saving and displaying the results
outdirname      = [dirname, '\Results'];
mkdir(outdirname)
save([outdirname, '\IC.mat'], 'IC');
save([outdirname, '\IC.txt'],'IC','-ASCII')
figure;imagesc(IC, [500, 950]);colormap(jet(256));colorbar