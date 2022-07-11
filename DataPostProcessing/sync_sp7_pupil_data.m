clear; close all; clc;

%% Importing data
filename_pe = "./000/pupil_positions.csv";
data_pe = importPupilData(filename_pe);

filename_sp7 = "SP7MotionData-20220601183017.log";
data_sp7 = importSP7Data(filename_sp7);
%% DATA PREPROCESSING
%%--------PUPIL EYE DATA ------------------ %%
%[data confidence > data_confidence_threshold]
% [norm_pos
%    ->right
%         ->x
%         ->y
%         ->t
%     ->left
%         ->x
%         ->y
%         ->t
% ]
data_confidence_threshold = 0.8;
%Offseting the timestamp to match the system clock
timestamp_offset  = 1653857113.4873507; % value measurement using script and from sync_info_*.info
data_pe.pupil_timestamp = (data_pe.pupil_timestamp + timestamp_offset) *1000; % seconds to millisecond 

%extracting right eye info [eye_id = 0]
right_eye_condition = data_pe.eye_id == 0 & data_pe.confidence > data_confidence_threshold;
norm_pos.right.x = data_pe.norm_pos_x(find(right_eye_condition));
norm_pos.right.y = data_pe.norm_pos_y(find(right_eye_condition));
norm_pos.right.t = data_pe.pupil_timestamp(find(right_eye_condition));

%extracting left eye info [eye_id = 1]
left_eye_condition = data_pe.eye_id == 1& data_pe.confidence > data_confidence_threshold;
norm_pos.left.x = data_pe.norm_pos_x(find(left_eye_condition));
norm_pos.left.y = data_pe.norm_pos_y(find(left_eye_condition));
norm_pos.left.t = data_pe.pupil_timestamp(find(left_eye_condition));

%%-------------------- SP7 DATA ------------------%
% sp7
%    ->t
%    ->yaw
%    ->yawSpeed
% sp7_ref
%     ->t
%     ->yaw
%     ->yawSpeed
%%SYNCRONIZING THE DATA BASED ON THE TIMEDATA
index = 1;
while(data_sp7.EpochTimeStampMS(index) <= data_pe.pupil_timestamp(1))
    index = index +1;
end
data_sp7(1:index, :) = []; %discarding the previous additional recordings
data_sp7(end, :) = []; %discarding the last NaN entry


%data of motion acutally performed
sp7.t = data_sp7.EpochTimeStampMS;
sp7.yaw = data_sp7.q7;
sp7.yawSpeed = (data_sp7.qdot7);

%data of motion input sent to platform
sp7_ref.t = data_sp7.EpochTimeStampMS;
sp7_ref.yaw = data_sp7.qref7;
sp7_ref.yawSpeed = data_sp7.qdotref7;

%% RESAMPLING THE DATA     
% PARAM
resamplingRate = 60;
resampled_sp7_data = resamplingSP7Data(sp7, resamplingRate);
resampled_pe_data.right = resamplingPEData(norm_pos.right, resamplingRate);
resampled_pe_data.left = resamplingPEData(norm_pos.left, resamplingRate);

%% REMOVING THE EXCESS DATA
end_length = size(resampled_pe_data.right.t,1);
if (size(resampled_sp7_data.t) < size(resampled_pe_data.right.t))
    end_length = size(resampled_sp7_data.t,1);
end
% SP7
sp7_out = struct();
fields = fieldnames(sp7);
for itr1 = 1:1:length(fields)
    sp7_out.(fields{itr1})=resampled_sp7_data.(fields{itr1})(1:end_length,:);
end

%PE
pe_out = struct();
fields = fieldnames(norm_pos.right);
for itr1 = 1:1:length(fields)
    pe_out.right.(fields{itr1})=resampled_pe_data.right.(fields{itr1})(1:end_length,:);
end
fields = fieldnames(norm_pos.right);
for itr1 = 1:1:length(fields)
    pe_out.left.(fields{itr1})=resampled_pe_data.left.(fields{itr1})(1:end_length,:);
end

%% DATA VISUALIZATION

%Right eye
figure()
subplot(2,2,1);
plot(pe_out.right.t-pe_out.right.t(1), pe_out.right.x, 'r');
xlabel('Timestamp [relative] [seconds]')
ylabel('Norm pos x [Right]');
grid on;


subplot(2,2,2);
plot(pe_out.right.t-pe_out.right.t(1), pe_out.right.y,'r');
xlabel('Timestamp [relative] [seconds]')
ylabel('Norm pos y [Right]');
grid on;

subplot(2,2,[3,4]);
plot(pe_out.right.x, pe_out.right.y,'r.');
xlabel('Norm pos x [Right]')
ylabel('Norm pos y [Right]');
grid on;
sgtitle( 'Right Eye [Pupil lab]' );


%Left eye
figure()
subplot(2,2,1);
plot(pe_out.left.t-pe_out.left.t(1), pe_out.left.x, 'g');
xlabel('Timestamp [relative] [seconds]')
ylabel('Norm pos x [left]');
grid on;

subplot(2,2,2);
plot(pe_out.left.t-pe_out.left.t(1), pe_out.left.y,'g');
xlabel('Timestamp [relative] [seconds]')
ylabel('Norm pos y [left]');
grid on;

subplot(2,2,[3,4]);
plot(pe_out.left.x, pe_out.left.y,'g.');
xlabel('Norm pos x [left]')
ylabel('Norm pos y [left]');
grid on;
sgtitle( 'Left Eye [Pupil lab]' );


%SP7
%Left eye
figure()
subplot(1,2,1);
plot(sp7_out.t-sp7_out.t(1), sp7_out.yaw, 'b');
xlabel('Timestamp [relative] [seconds]')
ylabel('Yaw [degrees]');
grid on;

subplot(1,2,2);
plot(sp7_out.t-sp7_out.t(1), sp7_out.yawSpeed,'b');
xlabel('Timestamp [relative] [seconds]')
ylabel('Yaw speed [degrees/second]');
grid on;
sgtitle( 'SP7 actual motion' );

%COMPARSION
figure()
subplot(1,2,1)
plot(sp7_out.yawSpeed, pe_out.right.x, 'r')
xlabel('Yaw speed [degrees/second]')
ylabel('Norm pos x [Right]');
grid on;
axis([-inf inf 0 1])
subplot(1,2,2)
plot(sp7_out.yawSpeed, pe_out.right.y, 'r')
xlabel('Yaw speed [degrees/second]')
ylabel('Norm pos y [Right]');
grid on;
axis([-inf inf 0 1])
sgtitle( 'Right Eye vs Yaw Speed' );


figure()
subplot(1,2,1)
plot(sp7_out.yawSpeed, pe_out.left.x, 'g')
xlabel('Yaw speed [degrees/second]')
ylabel('Norm pos x [Left]');
grid on;
axis([-inf inf 0 1])
subplot(1,2,2)
plot(sp7_out.yawSpeed, pe_out.left.y, 'g')
xlabel('Yaw speed [degrees/second]')
ylabel('Norm pos y [Left]');
grid on;
axis([-inf inf 0 1])
sgtitle( 'Left Eye vs Yaw Speed' );


%if interested in 3D representation, uncomment the following
figure()
plot3(pe_out.right.x,pe_out.right.y, sp7_out.yawSpeed, 'r.')
grid on
sgtitle('Right EYE x and y vs YAW speed')
xlabel('Norm pos x [Right]')
ylabel('Norm pos y [Right]');
zlabel('Yaw speed [degrees/second]')

