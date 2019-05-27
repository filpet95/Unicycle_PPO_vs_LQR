clc; clear variables; close all
addpath '../../helper_functions'
addpath 'logs'
load 'disable_map.mat'
deffilem5sub
% addpath('model_tests')
file_to_read = 'LQR_28_battery_test3.csv';
% file_to_read = "DL_20_battery_test1.csv";
tst_data = csvread(file_to_read,1,0);
% plot(tst_data(:,9))
% tst_data=tst_data(6000:end,:);
% plot(tst_data(:,9))
enable_data = tst_data(2:end,10);
% enable_idx = find(enable_data-1,1,'last');
enable_idx = find(enable_data,1);
pitch_data = tst_data(2:end,9);
roll_data = tst_data(2:end,8);
pitchvel_data = tst_data(2:end,7);
rollvel_data = tst_data(2:end,6);
disable_idx = disable_map(file_to_read);
% disable_idx = find(abs(pitch_data)<40,1,'last');
% disable_idx = find(abs(pitch_data)>40,1);
% if isempty(disable_idx)
%     disable_idx = length(tst_data);
% end

pitch_data = pitch_data(enable_idx:disable_idx);
roll_data = roll_data(enable_idx:disable_idx);
pitchvel_data = pitchvel_data(enable_idx:disable_idx);
rollvel_data = rollvel_data(enable_idx:disable_idx);

t = 0.01*(0:disable_idx-enable_idx);

figure(1)
hold on
% plot(t(1:end-2),states(6,1:end-2)*180/pi)
plot(t,pitchvel_data,'r-')
plot(t,rollvel_data,'b-')
plot([0 t(end)],[-35 -35],'black--')
plot([0 t(end)],[35 35],'black--')

figure(2)
hold on
% plot(t(1:end-2),states(6,1:end-2)*180/pi)
plot(t,pitch_data,'r-')
plot(t,roll_data,'b-')
plot([0 t(end)],[-3 -3],'black--')
plot([0 t(end)],[3 3],'black--')
figure(1)


failed_tests_LQR = [2,3,5];
failed_tests_DL = [5,7,9];

init_rad_LQR = zeros(10,2);
init_radvel_LQR = zeros(10,2);
for i = 1:10
    file_to_read = 'LQR_28_battery_test' + string(i) + '.csv';
    tst_data = csvread(file_to_read,1,0);
    enable_data = tst_data(2:end,10);
    enable_idx = find(enable_data,1);
    pitch_data = tst_data(2:end,9);
    roll_data = tst_data(2:end,8);
    pitchvel_data = tst_data(2:end,7);
    rollvel_data = tst_data(2:end,6);
    disable_idx = disable_map(file_to_read);
    pitch_data = pitch_data(enable_idx:disable_idx);
    roll_data = roll_data(enable_idx:disable_idx);
    pitchvel_data = pitchvel_data(enable_idx:disable_idx);
    rollvel_data = rollvel_data(enable_idx:disable_idx);
    
    init_rad_LQR(i,1) = roll_data(1)*pi/180;
    init_rad_LQR(i,2) = pitch_data(1)*pi/180;
    init_radvel_LQR(i,1) = rollvel_data(1)*pi/180;
    init_radvel_LQR(i,2) = pitchvel_data(1)*pi/180;
    
    t = 0.01*(0:disable_idx-enable_idx);
    figure(3)
    hold on
    subplot(5,2,i)
    hold on
    % plot(t(1:end-2),states(6,1:end-2)*180/pi)
    plot(t,pitchvel_data,'r-')
    plot(t,rollvel_data,'b-')
    plot([0 t(end)],[-20 -20],'black--')
    plot([0 t(end)],[20 20],'black--')
    figure(4)
    hold on
    subplot(5,2,i)
    hold on
    % plot(t(1:end-2),states(6,1:end-2)*180/pi)
    plot(t,pitch_data,'r-')
    plot(t,roll_data,'b-')
    plot([0 t(end)],[-3 -3],'black--')
    plot([0 t(end)],[3 3],'black--')
    
end
init_rad_DL = zeros(10,2);
init_radvel_DL = zeros(10,2);
for i = 1:10
    file_to_read = 'DL_20_battery_test' + string(i) + '.csv';
    tst_data = csvread(file_to_read,1,0);
    enable_data = tst_data(2:end,10);
    enable_idx = find(enable_data,1);
    pitch_data = tst_data(2:end,9);
    roll_data = tst_data(2:end,8);
    pitchvel_data = tst_data(2:end,7);
    rollvel_data = tst_data(2:end,6);
    disable_idx = disable_map(file_to_read);
    pitch_data = pitch_data(enable_idx:disable_idx);
    roll_data = roll_data(enable_idx:disable_idx);
    pitchvel_data = pitchvel_data(enable_idx:disable_idx);
    rollvel_data = rollvel_data(enable_idx:disable_idx);
    
    init_rad_DL(i,1) = roll_data(1)*pi/180;
    init_rad_DL(i,2) = pitch_data(1)*pi/180;
    init_radvel_DL(i,1) = rollvel_data(1)*pi/180;
    init_radvel_DL(i,2) = pitchvel_data(1)*pi/180;
    t = 0.01*(0:disable_idx-enable_idx);
    figure(5)
    hold on
    subplot(5,2,i)
    hold on
    % plot(t(1:end-2),states(6,1:end-2)*180/pi)
    plot(t,pitchvel_data,'r-')
    plot(t,rollvel_data,'b-')
    plot([0 t(end)],[-20 -20],'black--')
    plot([0 t(end)],[20 20],'black--')
    figure(6)
    hold on
    subplot(5,2,i)
    hold on
    % plot(t(1:end-2),states(6,1:end-2)*180/pi)
    plot(t,pitch_data,'r-')
    plot(t,roll_data,'b-')
    plot([0 t(end)],[-3 -3],'black--')
    plot([0 t(end)],[3 3],'black--')
    
end

table(abs(init_rad_LQR*180/pi - [zeros(10,1),28*ones(10,1)])<=0.3,abs(init_rad_DL*180/pi - [zeros(10,1),20*ones(10,1)])<=0.3,...
    abs(init_radvel_LQR*180/pi)<=3+3,abs(init_radvel_DL*180/pi)<=3+3,'VariableNames',{'LQR_angles','DL_angles','LQR_dangles','DL_dangles'})
