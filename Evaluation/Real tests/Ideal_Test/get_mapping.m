clc; clear variables; close all
addpath '../../helper_functions'
addpath 'logs'
addpath 'Model_error_Test'

deffilem5sub
% addpath('model_tests')
% file_to_read = "DL_20_battery_test9.csv";
% file_to_read = 'LQR_28_battery_test5.csv';
% file_to_read = 'LQR_+7_roll_test10.csv';
% file_to_read = 'DL_+7_roll_test10.csv';
% file_to_read = 'DL_14_battery_test5.csv';
% file_to_read = 'LQR_14_battery_test4.csv';
% file_to_read = '14_dl_10.csv';
% file_to_read = '14_lqr_10.csv';
% file_to_read = 'DL_modelerror_9_test10.csv';
% file_to_read = 'LQR_modelerror_9_test10.csv';
file_to_read = 'LQR_modelerror_pitch_29_test4.csv';
% file_to_read = 'DL_modelerror_pitch_21_test10.csv';


tst_data = csvread(file_to_read,1,0);
% plot(tst_data(:,9))
% tst_data=tst_data(6000:end,:);
% plot(tst_data(:,9))
enable_data = tst_data(2:end,10);
% enable_idx = find(enable_data-1,1,'last');
enable_idx = find(enable_data,1);
enable_idx = enable_idx + 1;
% enable_idx = 1;
pitch_data = tst_data(:,9);
roll_data = tst_data(:,8);
% disable_idx = find(abs(pitch_data)<40,1,'last');
disable_idx = length(tst_data);

pitch_data = pitch_data(enable_idx:disable_idx);
roll_data = roll_data(enable_idx:disable_idx);
pitchvel_data = tst_data(enable_idx:disable_idx,7);
rollvel_data = tst_data(enable_idx:disable_idx,6);
dalpha_w_data = tst_data(enable_idx:disable_idx,4);
dalpha_d_data = tst_data(enable_idx:disable_idx,5);

fprintf("roll: %.4f\tdroll: %.4f\tpitch: %.4f\tdpitch: %.4f\n",roll_data(1),rollvel_data(1),pitch_data(1),pitchvel_data(1))

figure(1)
hold on
% plot(t(1:end-2),states(6,1:end-2)*180/pi)
plot(pitch_data)
plot(roll_data)
% figure(2)
% plot(dalpha_w_data)
% hold on
% plot(dalpha_d_data)

%%
load 'disable_map.mat'
disable_map(file_to_read) = enable_idx + 500
save('disable_map','disable_map')

