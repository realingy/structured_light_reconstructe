
% This function computes the corresponding projector column (sub-pixel) for
% each camera pixel, given the input images, according to Micro Phase 
% Shifting scheme. 
%
% Input parameters:
% dirname       -- directory containing input images
% imPrefix      -- prefix of captured images
% imSuffix      -- suffix of captured images
% indexLength   -- length of index for captured images
% frequencyVec  -- vector containing the projected frequencies (periods in pixels)
% nr, nc        -- number of camera rows, columns
% numProjColumns-- number of total projector columns
%
%
% Output:
% IC            -- correspondence map (corresponding projector column
% (sub-pixel) for each camera pixel. Size of IC is the same as input
% captured imgaes.



function [IC]   = MicroPhaseShiftingDecodeFunc(dirname, imPrefix, imSuffix, indexLength, frequencyVec, nr, nc, numProjColumns)


numFrequency    = numel(frequencyVec);



%%%%%%%% Making the measurement matrix M (see paper for definition) %%%%%%%

M           = zeros(numFrequency+2, numFrequency+2);

% Filling the first three rows -- correpsonding to the first frequency
M(1,:)      = [1     cos(2*pi*0/3)     -sin(2*pi*0/3)    zeros(1, numFrequency-1)];
M(2,:)      = [1     cos(2*pi*1/3)     -sin(2*pi*1/3)    zeros(1, numFrequency-1)];
M(3,:)      = [1     cos(2*pi*2/3)     -sin(2*pi*2/3)    zeros(1, numFrequency-1)];

% Filling the remaining rows - one for each subsequent frequency
for f=2:numFrequency
    M(f+2, :)   = [1     zeros(1, f)     1   zeros(1, numFrequency-f)];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%







%%%%%%%%%%%%% Making the observation matrix (captured images) %%%%%%%%%%%%%

R           = zeros(numFrequency+2, nr*nc);

% Filling the observation matrix (image intensities)
for i=1:numFrequency+2
    IName   = [dirname, '\', imPrefix, sprintf(['%0', num2str(indexLength), 'd'], i), imSuffix];
    Itmp    = im2double(imread(IName));
    Itmp    = mean(Itmp,3);                                 % gray-scale intensities are used
    
    R(i,:)  = Itmp(:)';
    clear Itmp
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%%%%%%% Solving the linear system %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% The unknowns are [Offset, Amp*cos(phi_1), Amp*sin(phi_1), Amp*cos(phi_2),
% ..., Amp*cos(phi_F)], where F = numFrequency. See paper for details. 

U   = M\R;



% Computing the amplitude 
Amp                     = sqrt(U(2,:).^2 + U(3,:).^2);


% Dividing the amplitude to get the CosSinMat --- matrix containing the sin
% and cos of the phases corresponding to different frequencies. For the
% phase of the first frequency, we have both sin and cos. For the phases of
% the remaining frequencies, we have cos. 

CosSinMat               = U(2:end, :) ./ repmat(Amp, [numFrequency+1, 1]);

clear M R U Amp

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%





%%%%%%%%%%%%%%% Converting the CosSinMat into column indices %%%%%%%%%%%%%%
IC                      = PhaseUnwrapCosSinValsToColumnIndex(CosSinMat, frequencyVec, numProjColumns, nr, nc);              % This function converts the CosSinMat into column-correspondence using a linear search