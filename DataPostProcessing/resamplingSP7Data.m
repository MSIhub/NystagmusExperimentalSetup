function resampled_sp7_data = resamplingSP7Data(dataIn, desiredFs)
%resamplingSP7Data resampling raw data of sp7
% author: @msihub , mohamedsadiq.ikbal@edu.unige.it

%%--------INPUT------------------------------------------------------------%%
%dataIn:
% sp7
%    ->t [milliseconds]
%    ->yaw [degrees]
%    ->yawSpeed [degrees per second]
% DATA TYPE: Struct

%desiredFs:
% INTEGER: in Hertz, default: 60Hz

%%----------OUTPUT----------------------------------------------------------%%
%resampled_sp7_data: same as dataIn with resampled
%%--------------------------------------------------------------------------%%

if nargin < 2
    desiredFs =60; % in Hz
end

StartTimestamp = dataIn.t(1)*1e-3; 
% -- making relative timestamp and converting milliseconds to seconds
dataIn.t = (dataIn.t - dataIn.t(1))*1e-3;

% calculating the nominal freq of the input data
freqDiff = zeros(length(dataIn.t),1);
for i=2:1:length(dataIn.t)
    freqDiff(i,1) = dataIn.t(i) - dataIn.t(i-1);
end
nominalFs = 1/(mode(freqDiff));

% parameters of antialiasing filter
p = 1;
q = round((nominalFs/desiredFs));
% ensure an odd length filter
% n = 10*q+1;
n = 2*q+1;
% use .25 of Nyquist range of desired sample rate
cutoffRatio = .25;
% construct lowpass filter
lpFilt = p * fir1(n, cutoffRatio * 1/q);

[resampledData,resampledTimestamp] = resample(dataIn.yaw,dataIn.t,desiredFs,p,q,lpFilt);
[resampledDataYawSpeed,resampledTimestamp2] = resample(dataIn.yawSpeed,dataIn.t,desiredFs,p,q,lpFilt);
% Ignoring top and both 5 values as they are prone to error as a result of
% antialiasing filtering
resampled_sp7_data.yaw = resampledData(5:end-5,:);
resampled_sp7_data.yawSpeed = resampledDataYawSpeed(5:end-5,:);
resampled_sp7_data.t = resampledTimestamp(5:end-5);
%offseting back to the start timestamp
resampled_sp7_data.t  = (StartTimestamp) + resampled_sp7_data.t;
end