clc; clear variables; close all
addpath '../../helper_functions'
addpath 'logs'
load 'disable_map.mat'
deffilem5sub

stable_lqr = {'Yes','No','No','Yes','-','Yes','Yes','Yes','Yes','-',};
stable_dl = {'Yes','Yes','-','Yes','No','Yes','No','Yes','-','Yes',};
stabl = {stable_lqr;stable_dl};

% file_to_read = 'LQR_28_battery_test9.csv';
test = 4;
meth = 'DL';
avg = 0;

method = 2;
% file_to_read = 'DL_14_battery_test' + string(test) + '.csv';
% if isequal('LQR',meth)
%     method = 1;
%     file_to_read = 'LQR_14_battery_test'+ string(test) + '.csv';
% end
file_to_read = '14_dl_' + string(test) + '.csv';
if isequal('LQR',meth)
    method = 1;
    file_to_read = '14_lqr_'+ string(test) + '.csv';
end

% 
tst_data = csvread(file_to_read,1,0);
enable_data = tst_data(2:end,10);
% enable_idx = find(enable_data-1,1,'last');
enable_idx = find(enable_data,1);

pitch_data = tst_data(2:end,9);
roll_data = tst_data(2:end,8);
pitchvel_data = tst_data(2:end,7);
rollvel_data = tst_data(2:end,6);
disk_data = tst_data(2:end,5);
wheel_data = tst_data(2:end,4);
inp_w_data = tst_data(2:end,2);
inp_d_data = tst_data(2:end,3);

disable_idx = disable_map(file_to_read);
% disable_idx = enable_idx+200;

pitch_data = pitch_data(enable_idx:disable_idx);
roll_data = roll_data(enable_idx:disable_idx);
pitchvel_data = pitchvel_data(enable_idx:disable_idx);
rollvel_data = rollvel_data(enable_idx:disable_idx);
disk_data = disk_data(enable_idx:disable_idx);
wheel_data = wheel_data(enable_idx:disable_idx);
inp_w_data = inp_w_data(enable_idx:disable_idx);
inp_d_data = inp_d_data(enable_idx:disable_idx);
t = 0.01*(0:disable_idx-enable_idx);

init_roll = roll_data(1);
init_pitch = pitch_data(1);
init_rollvel = rollvel_data(1);
init_pitchvel = pitchvel_data(1);

roll_limit = 2;
ptch_limit = 2;
ptchvel_limit = 15;
rollvel_limit = 15;
disk_limit = 130;
wheel_limit = 20;
time_stable = 10;
max_oversht = -min(pitch_data);
for i = 1:length(pitch_data)
%     if abs(pitch_data(i)) < ptch_limit && abs(roll_data(i)) < roll_limit && abs(pitchvel_data(i)) < rollvel_limit && abs(rollvel_data(i)) < rollvel_limit
    if abs(pitch_data(i)) < ptch_limit && abs(wheel_data(i)) < wheel_limit
%         fprintf("test %d stabilized after %f seconds with overshoot = %f \n",test,i*0.01,max_oversht)
        time_stable = i*0.01;
        break
    end
end

fprintf("%s\t&%d\t&%.4f\t&%.4f\t&%.4f\t&%.4f\t\\\\ \\hline \n " ,...
    meth ,test,time_stable,max_oversht,init_pitch,init_pitchvel)


f1 = figure(1);
hold on
plot(t,roll_data,'b-')
plot(t,pitch_data,'r-')
% title({'Roll, $\varphi$'},'interpreter','latex')
xlabel({'[\textit{s}]'},'interpreter','latex')
ylabel({'[\textit{deg}]'},'interpreter','latex')
plot([time_stable time_stable],[-10,20],'black--')
plot([0 t(end)],[-max_oversht -max_oversht],'black-.')
% if method == 1
%    plot([0 t(end)],[-2*1.4831 -2*1.4831],'black--')
%    plot([0 t(end)],[2*1.4831 2*1.4831],'black--','HandleVisibility','off')
% else
%    plot([0 t(end)],[-2*1.4858 -2*1.4858],'black--')
%    plot([0 t(end)],[2*1.4858 2*1.4858],'black--','HandleVisibility','off') 
% end
legend({'$\varphi$','$\theta$','$t_s$','$M_p$'},'interpreter','latex')
f1.Position = f1.Position.*[1 1 1 0.5];

figure(2)
hold on
plot(t,disk_data,'b-')
plot(t,wheel_data,'r-')
% title({'Disk speed, $\dot{\alpha}_d$'},'interpreter','latex')
xlabel({'[\textit{s}]'},'interpreter','latex')
ylabel({'[\textit{RPM}]'},'interpreter','latex')
legend({'$\dot{\alpha}_d$','$\dot{\alpha}_w$'},'interpreter','latex')

f2 = figure(3);
hold on
plot(t,rollvel_data,'b-')
plot(t,pitchvel_data,'r-')
plot([time_stable time_stable],[-100,100],'black--')
% title({'LQR Angular Velocities'},'interpreter','latex')
xlabel({'[\textit{s}]'},'interpreter','latex')
ylabel({'[$\frac{\textit{deg}}{\textit{s}}$]'},'interpreter','latex')
% plot([0 t(end)],[-15 -15],'black--')
% plot([0 t(end)],[15 15],'black--','HandleVisibility','off')
legend({'$\dot{\varphi}$','$\dot{\theta}$','$t_s$'},'interpreter','latex')
f2.Position = f2.Position.*[1 1 1 0.5];
figure(4)
hold on
plot(t,saturate(inp_d_data,12),'b-')
plot(t,saturate(inp_w_data,12),'r-')
% title({'LQR Angular Velocities'},'interpreter','latex')
xlabel({'[\textit{s}]'},'interpreter','latex')
ylabel({'[\textit{V}]'},'interpreter','latex')
legend({'$u_d$','$u_w$'},'interpreter','latex')

fprintf("%s test %d:\n Stabilized = %s \n Pitch = %f \n PitchVel = %f\n Roll = %f \n Rollvel = %f \n ",...
    meth, test,stabl{method}{test},init_pitch,init_pitchvel,init_roll,init_rollvel)

% roll_limit = 2;
% ptch_limit = 2;
% ptchvel_limit = 15;
% rollvel_limit = 15;
% for i = 1:length(pitch_data)
%     if abs(pitch_data(i)) < ptch_limit && abs(roll_data(i)) < roll_limit && abs(pitchvel_data(i)) < rollvel_limit && abs(rollvel_data(i)) < rollvel_limit
%         fprintf("stabilized after %f seconds",i*0.01)
%         break
%     end
% end

% 
% fprintf("%d\t&%s\t&%.4f\t&%.4f\t&%.4f\t&%.4f\t\\\\ \\hline \n " ,...
%     test,stabl{method}{test},init_pitch,init_pitchvel,init_roll,init_rollvel)
% 
% failed_tests_LQR = [2,3,5];
% failed_tests_DL = [5,7,9];
% 
% init_rad_LQR = zeros(10,2);
% init_radvel_LQR = zeros(10,2);
% for i = 1:9
%     file_to_read = 'LQR_28_battery_test' + string(i) + '.csv';
%     tst_data = csvread(file_to_read,1,0);
%     enable_data = tst_data(2:end,10);
%     enable_idx = find(enable_data,1);
%     pitch_data = tst_data(2:end,9);
%     roll_data = tst_data(2:end,8);
%     pitchvel_data = tst_data(2:end,7);
%     rollvel_data = tst_data(2:end,6);
%     disable_idx = disable_map(file_to_read);
%     pitch_data = pitch_data(enable_idx:disable_idx);
%     roll_data = roll_data(enable_idx:disable_idx);
%     pitchvel_data = pitchvel_data(enable_idx:disable_idx);
%     rollvel_data = rollvel_data(enable_idx:disable_idx);
%     
%     init_rad_LQR(i,1) = roll_data(1)*pi/180;
%     init_rad_LQR(i,2) = pitch_data(1)*pi/180;
%     init_radvel_LQR(i,1) = rollvel_data(1)*pi/180;
%     init_radvel_LQR(i,2) = pitchvel_data(1)*pi/180;
%     
%     t = 0.01*(0:disable_idx-enable_idx);
%     figure(3)
%     hold on
%     subplot(5,2,i)
%     hold on
%     % plot(t(1:end-2),states(6,1:end-2)*180/pi)
%     plot(t,pitchvel_data,'r-')
%     plot(t,rollvel_data,'b-')
%     plot([0 t(end)],[-20 -20],'black--')
%     plot([0 t(end)],[20 20],'black--')
%     figure(4)
%     hold on
%     subplot(5,2,i)
%     hold on
%     % plot(t(1:end-2),states(6,1:end-2)*180/pi)
%     plot(t,pitch_data,'r-')
%     plot(t,roll_data,'b-')
%     plot([0 t(end)],[-3 -3],'black--')
%     plot([0 t(end)],[3 3],'black--')
%     
% end
% init_rad_DL = zeros(10,2);
% init_radvel_DL = zeros(10,2);
% for i = 1:10
%     file_to_read = 'DL_20_battery_test' + string(i) + '.csv';
%     tst_data = csvread(file_to_read,1,0);
%     enable_data = tst_data(2:end,10);
%     enable_idx = find(enable_data,1);
%     pitch_data = tst_data(2:end,9);
%     roll_data = tst_data(2:end,8);
%     pitchvel_data = tst_data(2:end,7);
%     rollvel_data = tst_data(2:end,6);
%     disable_idx = disable_map(file_to_read);
%     pitch_data = pitch_data(enable_idx:disable_idx);
%     roll_data = roll_data(enable_idx:disable_idx);
%     pitchvel_data = pitchvel_data(enable_idx:disable_idx);
%     rollvel_data = rollvel_data(enable_idx:disable_idx);
%     
%     init_rad_DL(i,1) = roll_data(1)*pi/180;
%     init_rad_DL(i,2) = pitch_data(1)*pi/180;
%     init_radvel_DL(i,1) = rollvel_data(1)*pi/180;
%     init_radvel_DL(i,2) = pitchvel_data(1)*pi/180;
%     t = 0.01*(0:disable_idx-enable_idx);
%     figure(5)
%     hold on
%     subplot(5,2,i)
%     hold on
%     % plot(t(1:end-2),states(6,1:end-2)*180/pi)
%     plot(t,pitchvel_data,'r-')
%     plot(t,rollvel_data,'b-')
%     plot([0 t(end)],[-20 -20],'black--')
%     plot([0 t(end)],[20 20],'black--')
%     figure(6)
%     hold on
%     subplot(5,2,i)
%     hold on
%     % plot(t(1:end-2),states(6,1:end-2)*180/pi)
%     plot(t,pitch_data,'r-')
%     plot(t,roll_data,'b-')
%     plot([0 t(end)],[-3 -3],'black--')
%     plot([0 t(end)],[3 3],'black--')
%     
% end
% 
% table(abs(init_rad_LQR*180/pi - [zeros(10,1),28*ones(10,1)])<=0.3,abs(init_rad_DL*180/pi - [zeros(10,1),20*ones(10,1)])<=0.3,...
%     abs(init_radvel_LQR*180/pi)<=3+3,abs(init_radvel_DL*180/pi)<=3+3,'VariableNames',{'LQR_angles','DL_angles','LQR_dangles','DL_dangles'})
%TESTS THAT NEED TO BE REDONE: 
% LQR - 10, 5
% DL - 9, 3