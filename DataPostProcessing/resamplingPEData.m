function resampled_pe_data_ = resamplingPEData(dataIn, desiredFs)
%resamplingPEData resampling raw data of pupil eye
% author: @msihub , mohamedsadiq.ikbal@edu.unige.it

%%--------INPUT------------------------------------------------------------%%
%dataIn:
%         ->x
%         ->y
%         ->t

% ]
% DATA TYPE: Struct

%desiredFs:
% INTEGER: in Hertz, default: 60Hz

%%----------OUTPUT----------------------------------------------------------%%
%resampled_pe_data_: same as dataIn with resampled
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
freqDiff2 = freqDiff;
freqDiff2(freqDiff==0) = NaN;
nominalFs = 1/(mode(freqDiff2));

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

[resampledDataX,resampledTimestamp] = resample(dataIn.x,dataIn.t,desiredFs,p,q,lpFilt);
[resampledDataY,resampledTimestamp2] = resample(dataIn.y,dataIn.t,desiredFs,p,q,lpFilt);
% Ignoring top and both 5 values as they are prone to error as a result of
% antialiasing filtering
resampled_pe_data_.x = resampledDataX(5:end-5,:);
resampled_pe_data_.y = resampledDataY(5:end-5,:);
resampled_pe_data_.t = resampledTimestamp(5:end-5);
%offseting back to the start timestamp
resampled_pe_data_.t  = (StartTimestamp) + resampled_pe_data_.t;
end