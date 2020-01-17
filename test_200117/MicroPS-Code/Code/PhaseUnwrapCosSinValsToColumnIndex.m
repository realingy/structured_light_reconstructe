
% This function converts the CosSinMat into column-correspondence. 
% 
% CosSinMat is the matrix containing the sin and cos of the phases 
% corresponding to different frequencies for each camera pixel. For the
% phase of the first frequency, we have both sin and cos. For the phases of
% the remaining frequencies, we have cos. 
%
%
% The function first performs a linear search on the projector column
% indices. Then, it adds the sub-pixel component. 



function [IC]   = PhaseUnwrapCosSinValsToColumnIndex(CosSinMat, frequencyVec, numProjColumns, nr, nc)

x0              = [0:numProjColumns-1];                                                                             % Projector column indices

% Coomputing the cos and sin values for each projector column. The format 
% is the same as in CosSinMat - for the phase of the first frequency, we 
% have both sin and cos. For the phases of the remaining frequencies, we 
% have cos. These will be compared against the values in CosSinMat to find 
% the closest match. 

TestMat         = repmat(x0, [size(CosSinMat,1) 1]);

TestMat(1,:)    = cos(  (mod(TestMat(1,:), frequencyVec(1)) / frequencyVec(1)) * 2 * pi);                           % cos of the phase for the first frequency
TestMat(2,:)    = sin(  (mod(TestMat(2,:), frequencyVec(1)) / frequencyVec(1)) * 2 * pi);                           % sin of the phase for the first frequency

for i=3:size(CosSinMat,1)
    TestMat(i,:)    = cos(  (mod(TestMat(i,:), frequencyVec(i-1)) / frequencyVec(i-1)) * 2 * pi);                   % cos of the phases of the remaining frequency
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

IC                  = zeros(1, nr*nc);                                                                              % Vector of column-values

% For each camera pixel, find the closest match
matlabpool open 8;                                                                                                  % This loop can be run in parallel using MATLAB parallel toolbox. The number here is the number of cores on your machine. 
parfor i=1:size(CosSinMat,2)
    CosSinVec   = CosSinMat(:,i);
    ErrorVec    = sum(abs(repmat(CosSinVec, [1 numProjColumns])   -   TestMat).^2, 1);            
    [~, Ind]    = min(ErrorVec(:));
    IC(1,i)     = Ind;
end

matlabpool close;

IC              = IC - 1;                                                                                           % Getting the estimates in to the range [0:numProjColumns-1], because the sinusoids were made in this range. 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




% Computing the fractional value using phase values of the first frequency 
% since it has both cos and sin values. 

PhaseFirstFrequency                     = acos(CosSinMat(1,:));                                                     % acos returns values in [0, pi] range. There is a 2 way ambiguity.
PhaseFirstFrequency(CosSinMat(2,:)<0)   = 2*pi - PhaseFirstFrequency(CosSinMat(2,:)<0);                             % Using the sin value to resolve the ambiguity
ColumnFirstFrequency                    = PhaseFirstFrequency * frequencyVec(1) / (2*pi);                           % The phase for the first frequency, in pixel units. This is equal to mod(trueColumn, frequencyVec(1)). 


NumCompletePeriodsFirstFreq             = floor(IC / frequencyVec(1));                                              % The number of complete periods for the first frequency 
ICFrac                                  = NumCompletePeriodsFirstFreq * frequencyVec(1) + ColumnFirstFrequency;     % The final correspondence, with the fractional component



% If the difference after fractional correction is large (because of noise), keep the original value. 
ICFrac(abs(ICFrac-IC)>=1)               = IC(abs(ICFrac-IC)>=1);
IC                                      = ICFrac;



% Adding back the one to get it in the range [1,numProjColumns]
IC          = IC+1;
IC          = reshape(IC, [nr nc]);                                                                                 % Reshaping to make the same size as the captured image