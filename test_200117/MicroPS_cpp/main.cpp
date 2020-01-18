#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

using namespace std;

string dirname = "../Data/WaxBowl/32-08"; // Directory containing the captured input images
string projDirname = "../ProjectedImages/32-08"; // Directory containing the frequency information of the projected images (freqData.mat)

cv::Mat MicroPhaseShiftDecode(string dirname, string imPrefix, string imSuffix, int indexLength, vector<float> frequencyVec, cv::Size camDim, cv::Size projDim);

/*
int main()
{
	float x = fmod(12, 1.777777);
	cout << "x: " << x << endl;

	return 0;
}
*/

#if 1
int main()
{
	// load([projDirname, '\freqData.mat']);
	string imPrefix = "Frame"; // Prefix of the captured images
	string imSuffix = ".tiff"; // Suffix of the captured images
	int indexLength = 3;       // Length of the image index. For example, if images are numbered Frame1.tiff, Frame2.tiff,..., indexLength is 1. If images are numbered Frame001.tiff, Frame002.tiff,..., indexLength is 3.

	cv::Size projDim = cv::Size(1024, 768); // Dimensions of the projector image (numRows, numColumns).
	cv::Size camDim = cv::Size(900, 801);   // Dimensions of the camera image (numRows, numColumns).
	// int medfiltParam    = [5 5];         // The computed correspondence map is median filtered to mitigate noise. These are the median filter parameters. Usual values are between [1 1] to [7 7], depending on image noise levels, number of images used, and the frequencies. Use smaller values of these parameters for low noise levels, large number of input images, and low frequencies. For example, if the average frequency is 64 pixels, and 15 frequencies are used, use medFiltParam = [1 1]. On the other hand, if the average frequency is 16 pixels, and 5 frequencies are used, use medFiltParam = [7 7].
	// 中值滤波卷积核尺寸
	int medfiltParam = 5;					// The computed correspondence map is median filtered to mitigate noise. These are the median filter parameters. Usual values are between [1 1] to [7 7], depending on image noise levels, number of images used, and the frequencies. Use smaller values of these parameters for low noise levels, large number of input images, and low frequencies. For example, if the average frequency is 64 pixels, and 15 frequencies are used, use medFiltParam = [1 1]. On the other hand, if the average frequency is 16 pixels, and 5 frequencies are used, use medFiltParam = [7 7].

	cv::Mat IC = MicroPhaseShiftDecode(dirname, imPrefix, imSuffix, indexLength, frequencyVec, camDim, projDim); // The main decoding function. This function computes the corresponding projector column (sub-pixel) for each camera pixel. IC is the same size as the input images
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
	colorbar;

	return 0;
}

// The main decoding function.
// This function computes the corresponding projector column (sub-pixel) for each camera pixel.
// res is the same size as the input images
cv::Mat MicroPhaseShiftDecode(string dirname, string imPrefix, string imSuffix, int indexLength, vector<float> frequencyVec, cv::Size camDim, cv::Size projDim )
{
	/*
	This function computes the corresponding projector column(sub - pixel) for each camera pixel, given the input images, according to Micro Phase Shifting scheme.

	Input parameters:
	dirname        -- directory containing input images
	imPrefix       -- prefix of captured images
	imSuffix       -- suffix of captured images
	indexLength    -- length of index for captured images
	frequencyVec   -- vector containing the projected frequencies(periods in pixels)
	nr, nc         -- number of camera rows, columns
	numProjColumns -- number of total projector columns
	
	Output:
	解相结果
	IC             -- correspondence map(corresponding projector column(sub - pixel) for each camera pixel.Size of IC is the same as input captured images.
	*/

	//function[IC] = MicroPhaseShiftingDecodeFunc(dirname, imPrefix, imSuffix, indexLength, frequencyVec, nr, nc, numProjColumns)

	int numFrequency = frequencyVec.size(); //频率数

	// Making the measurement matrix M(see paper for definition)

	//M = zeros(numFrequency + 2, numFrequency + 2);
	cv::Mat M = cv::Mat::zeros(numFrequency+2, numFrequency+2, CV_32F);

	// Filling the first three rows -- correpsonding to the first frequency
	M(1, :) = [1     cos(2 * pi * 0 / 3) - sin(2 * pi * 0 / 3)    zeros(1, numFrequency - 1)];
	M(2, :) = [1     cos(2 * pi * 1 / 3) - sin(2 * pi * 1 / 3)    zeros(1, numFrequency - 1)];
	M(3, :) = [1     cos(2 * pi * 2 / 3) - sin(2 * pi * 2 / 3)    zeros(1, numFrequency - 1)];

	//Filling the remaining rows - one for each subsequent frequency
	for f = 2:numFrequency
		M(f + 2, :) = [1     zeros(1, f)     1   zeros(1, numFrequency - f)];
	end;

	//Making the observation matrix(captured images)
	int nr = camDim.height;
	int nc = camDim.width;

	cv::Mat R = cv::Mat::zeros(numFrequency+2, nr * nc, CV_32F);
	//R = zeros(numFrequency + 2, nr * nc);

	//Filling the observation matrix(image intensities)
	for i = 1:numFrequency + 2
	{
		IName = [dirname, '\', imPrefix, sprintf([' % 0', num2str(indexLength), 'd'], i), imSuffix];
		Itmp = im2double(imread(IName));
		Itmp = mean(Itmp, 3); //gray-scale intensities are used

		R(i, :) = Itmp(:)';
		// clear Itmp;
	}

	// Solving the linear system
	// The unknowns are[Offset, Amp * cos(phi_1), Amp * sin(phi_1), Amp * cos(phi_2),..., Amp * cos(phi_F)], where F = numFrequency.See paper for details.

	cv::Mat U = M\R;

	// Computing the amplitude
	cv::Mat Amp = sqrt(U(2, :). ^ 2 + U(3, :). ^ 2);

	/*
	% Dividing the amplitude to get the CosSinMat --- matrix containing the sin
	% and cos of the phases corresponding to different frequencies.
	% For the phase of the first frequency, we have both sin and cos.
	% For the phases of the remaining frequencies, we have cos.
	*/

	cv::Mat CosSinMat = U(2:end, : ) ./ repmat(Amp, [numFrequency + 1, 1]);

	// clear M R U Amp

	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	// Converting the CosSinMat into column indices
	cv::Mat IC = PhaseUnwrapCosSinValsToColumnIndex(CosSinMat, frequencyVec, numProjColumns, nr, nc);  // This function converts the CosSinMat into column - correspondence using a linear search

	return IC;
}

/*
% This function converts the CosSinMat into column - correspondence.
%
% CosSinMat is the matrix containing the sin and cos of the phases
% corresponding to different frequencies for each camera pixel.For the
% phase of the first frequency, we have both sinand cos.For the phases of
% the remaining frequencies, we have cos.
%
% The function first performs a linear search on the projector column
% indices.Then, it adds the sub-pixel component.
*/
cv::Mat PhaseUnwrapCosSinValsToColumnIndex(cv::Mat CosSinMat, vector<double> frequencyVec, int numProjColumns, int nr, int nc)
{
	//x0 = [0 : numProjColumns - 1]; // Projector column indices
	//cv::Mat TestMat = repmat(x0, [size(CosSinMat, 1) 1]);
	//cv::Mat TestMat = repmat(x0, [CosSinMat.rows 1]);

	/*
	% Computing the cos and sin values for each projector column.
	% The format is the same as in CosSinMat
	% For the phase of the first frequency, we have both sin and cos.
	% For the phases of the remaining frequencies, we have cos.
	% These will be compared against the values in CosSinMat to find the closest match.
	*/
	cv::Mat TestMat = cv::Mat(CosSinMat.rows, numProjColumns, CV_64F);
	for (size_t i = 0; i < TestMat.rows; i++)
	{
		double * data = TestMat.ptr<double>(i); //得到对i行的首地址
		for (size_t j = 0; j < TestMat.cols; j++)
		{
			data[j] = (double)j; 
		}
	}


	//TestMat(1, :) = cos( (mod(TestMat(1, :), frequencyVec(1)) / frequencyVec(1)) * 2 * pi ); //cos of the phase for the first frequency
	//TestMat(2, :) = sin( (mod(TestMat(2, :), frequencyVec(1)) / frequencyVec(1)) * 2 * pi ); //sin of the phase for the first frequency

	//for i = 3:size(CosSinMat, 1)
		//TestMat(i, :) = cos((mod(TestMat(i, :), frequencyVec(i - 1)) / frequencyVec(i - 1)) * 2 * pi); // cos of the phases of the remaining frequency

	//////////////////////////////////////////////////////////////////////////
	//TestMat.col(0) = TestMat.col(0); //行操作
	//////////////////////////////////////////////////////////////////////////

	//float* data = TestMat.ptr<float>(i); //得到对i行的首地址
	// 处理第一行
	for (int j = 0; j < TestMat.cols; j++)
	{
		float cur = TestMat.at<float>(0, j);
		TestMat.at<float>(0, j) = cos( ( fmod(cur, frequencyVec[0]) / frequencyVec[0]) * 2 * CV_PI);
	}

	// 处理第二行
	for (int j = 0; j < TestMat.cols; j++)
	{
		float cur = TestMat.at<float>(1, j);
		TestMat.at<float>(1, j) = sin( ( fmod(cur, frequencyVec[0]) / frequencyVec[0]) * 2 * CV_PI);
	}

	//处理剩余行
	for (int i = 0; i < TestMat.rows; i++)
	{
		for (int j = 0; j < TestMat.cols; j++)
		{
			float cur = TestMat.at<float>(i, j);
			TestMat.at<float>(i, j) = cos( ( fmod(cur, frequencyVec[i-1]) / frequencyVec[i-1]) * 2 * CV_PI);
		}
	}


	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	//cv::Mat IC = zeros(1, nr * nc); // Vector of column-values
	cv::Mat IC = cv::Mat::zeros(1, nr * nc, CV_64F); // Vector of column-values

	//For each camera pixel, find the closest match
	//This loop can be run in parallel using MATLAB parallel toolbox. The number here is the number of cores on your machine.
	// matlabpool open 8;
	//This loop can be run in parallel using MATLAB parallel toolbox.The number here is the number of cores on your machine.
	// parpool;

	/*
	for i = 1:size(CosSinMat, 2)
	{
		CosSinVec = CosSinMat(:, i);
		ErrorVec = sum(abs(repmat(CosSinVec, [1 numProjColumns]) - TestMat). ^ 2, 1);
		[~, Ind] = min(ErrorVec(:));
		IC(1, i) = Ind;
	}
	*/

	for (int i = 0; i < CosSinMat.cols; i++)
	{
		//[~, Ind] = min(ErrorVec(:)); //i表示最小值所在位置
		//IC(1, i) = Ind;

		cv::Mat CosSinVec = CosSinMat.colRange(i, i+1).clone();
		cv::Mat t1 = cv::Mat( CosSinVec.rows, numProjColumns, CV_64F );
		for (size_t j = 0; j < numProjColumns; j++)
		{
			t1.col(j) = CosSinVec;
		}
		
		cv::Mat t2 = abs(t1 - TestMat)^2;			 //元素求平方
		cv::Mat ErrorVec(1, numProjColumns, CV_64F); //每一列进行求和

		reduce(t2, ErrorVec, 0, CV_REDUCE_SUM);

		double minVal, maxVal;
		int minIdx, maxIdx;

		cv::minMaxIdx( ErrorVec, &minVal, &maxVal, &minIdx, &maxIdx );

		IC.at<double>(0, i) = minIdx; //将最小值的坐标赋给IC矩阵
	}

	// matlabpool close;
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	// Getting the estimates in to the range[0:numProjColumns - 1], because the sinusoids were made in this range.
	IC = IC - 1;

	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	/*
	% Computing the fractional value using phase values of the first frequency
	% since it has both cosand sin values.
	*/

	/*
	PhaseFirstFrequency = acos(CosSinMat(1, :)); // acos returns values in[0, pi] range.There is a 2 way ambiguity.
	PhaseFirstFrequency(CosSinMat(2, :) < 0) = 2 * pi - PhaseFirstFrequency(CosSinMat(2, :) < 0); // Using the sin value to resolve the ambiguity
	ColumnFirstFrequency = PhaseFirstFrequency * frequencyVec(1) / (2 * pi); // The phase for the first frequency, in pixel units.This is equal to mod(trueColumn, frequencyVec(1)).
	*/

	cv::Mat PhaseFirstFrequency = CosSinMat.row(0); //第一行
	for (size_t j = 0; j < PhaseFirstFrequency.cols; j++)
	{
		PhaseFirstFrequency.at<double>(0, j) = acos( PhaseFirstFrequency.at<double>(0, j) ); //acos returns values in[0, pi] range.There is a 2 way ambiguity.
	}

	for (size_t j = 0; j < PhaseFirstFrequency.cols; j++)
	{
		if (CosSinMat.at<double>(1, j) < 0)
		{
			PhaseFirstFrequency.at<double>(0, j) = 2 * CV_PI - PhaseFirstFrequency.at<double>(0, j); //Using the sin value to resolve the ambiguity
		}
	}
	cv::Mat ColumnFirstFrequency = PhaseFirstFrequency * frequencyVec[0] / (2 * CV_PI); //The phase for the first frequency, in pixel units.This is equal to mod(trueColumn, frequencyVec(1)).

	cv::Mat NumCompletePeriodsFirstFreq = IC.clone();
	for ( int i = 0; i < NumCompletePeriodsFirstFreq.rows; i++ )
	{
		for (int j = 0; j < NumCompletePeriodsFirstFreq.cols; j++)
		{
			NumCompletePeriodsFirstFreq.at<double>(i, j) = floor(IC.at<double>(i, j) / frequencyVec[0]); // The number of complete periods for the first frequency
		}
	}

	cv::Mat ICFrac = NumCompletePeriodsFirstFreq * frequencyVec[0] + ColumnFirstFrequency; // The final correspondence, with the fractional component

	//If the difference after fractional correction is large(because of noise), keep the original value.
	for (int i = 0; i < ICFrac.rows; i++)
	{
		for (int j = 0; j < ICFrac.cols; j++)
		{
			if (abs(ICFrac.at<double>(i, j) - IC.at<double>(i, j)) >= 1)
			{
				ICFrac.at<double>(i, j) = IC.at<double>(i, j);
			}
		}
	}
			
	//ICFrac(abs(ICFrac - IC) >= 1) = IC(abs(ICFrac - IC) >= 1);

	IC = ICFrac;

	//Adding back the one to get it in the range[1, numProjColumns]
	IC = IC + 1;

	//IC = reshape(IC, [nr nc]); //Reshaping to make the same size as the captured image
}
#endif