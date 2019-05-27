clc; clear variables; close all
addpath '../../helper_functions'
addpath 'logs'
% load 'disable_map.mat'
deffilem5sub
% addpath('model_tests')
test = 1;
file_to_read = 'LQR_'+string(test)+'.csv';
% file_to_read = "DL_20_battery_test1.csv";
tst_data = csvread(file_to_read,1,0);

enable_data = tst_data(2:end,10);

enable_idx_lqr = find(enable_data,1)+100;
% disable_idx_lqr = 8442;
disable_idx_lqr = enable_idx_lqr + 1000;
pitch_data = tst_data(:,9);
roll_data = tst_data(:,8);
enable_idx = enable_idx_lqr;
disable_idx = disable_idx_lqr;
pitch_data = pitch_data(enable_idx:disable_idx);
mean(pitch_data)
% pitch_data = pitch_data - mean(pitch_data);
roll_data = roll_data(enable_idx:disable_idx);
pitchvel_data = tst_data(enable_idx:disable_idx,7);
rollvel_data = tst_data(enable_idx:disable_idx,6);
wheel_data = tst_data(enable_idx:disable_idx,4);
disk_data = tst_data(enable_idx:disable_idx,5);

t = 0.01*(0:disable_idx-enable_idx);

std_pitch = std(pitch_data);
std_roll = std(roll_data);
% corr(pitch_data,roll_data)
% mean(pitch_data)
%% 
% addpath('model_tests')
file_to_read = 'DL_'+string(test)+'.csv';
% file_to_read = "DL_20_battery_test1.csv";
tst_data = csvread(file_to_read,1,0);

enable_data = tst_data(2:end,10);

enable_idx_dl = find(enable_data,1)+100;
% disable_idx_lqr = 8442;
disable_idx_dl = enable_idx_dl + 1000;
pitch_data_dl = tst_data(:,9);
roll_data_dl = tst_data(:,8);
enable_idx = enable_idx_dl;
disable_idx = disable_idx_dl;
pitch_data_dl = pitch_data_dl(enable_idx:disable_idx);
% pitch_data_dl = pitch_data_dl - mean(pitch_data_dl);
roll_data_dl = roll_data_dl(enable_idx:disable_idx);
pitchvel_data_dl = tst_data(enable_idx:disable_idx,7);
rollvel_data_dl = tst_data(enable_idx:disable_idx,6);
wheel_data_dl = tst_data(enable_idx:disable_idx,4);
disk_data_dl = tst_data(enable_idx:disable_idx,5);

t_dl = 0.01*(0:disable_idx-enable_idx);

%%
figure(1)
subplot(2,1,1)
hold on
plot(t,roll_data,'b-')
plot(t_dl,roll_data_dl,'b--')
title({'Roll, $\varphi$'},'interpreter','latex')
xlabel({'[\textit{s}]'},'interpreter','latex')
ylabel({'[\textit{deg}]'},'interpreter','latex')
legend({'LQR','DL'},'interpreter','latex')
subplot(2,1,2)
hold on
plot(t,pitch_data,'r-')
plot(t_dl,pitch_data_dl,'r--')
title({'Pitch, $\theta$'},'interpreter','latex')
xlabel({'[\textit{s}]'},'interpreter','latex')
ylabel({'[\textit{deg}]'},'interpreter','latex')
legend({'LQR','DL'},'interpreter','latex')


% figure(1)
% subplot(2,2,1)
% hold on
% % plot(t(1:end-2),states(6,1:end-2)*180/pi)
% plot(t,pitchvel_data,'r-')
% plot(t,rollvel_data,'b-')
% plot([0 t(end)],[-35 -35],'black--')
% plot([0 t(end)],[35 35],'black--')
% title('LQR')
% % figure(2)
% subplot(2,2,3)
% hold on
% % plot(t(1:end-2),states(6,1:end-2)*180/pi)
% plot(t,pitch_data,'r-')
% plot(t,roll_data,'b-')
% plot([0 t(end)],[-2*std_both -2*std_both],'black--')
% plot([0 t(end)],[2*std_both 2*std_both],'black--')
% title('LQR')

figure(2)
subplot(2,1,1)
hold on
plot(t,disk_data,'b-')
plot(t_dl,disk_data_dl,'b--')
title({'Disk speed, $\dot{\alpha}_d$'},'interpreter','latex')
xlabel({'[\textit{s}]'},'interpreter','latex')
ylabel({'[\textit{RPM}]'},'interpreter','latex')
legend({'LQR','DL'},'interpreter','latex')
subplot(2,1,2)
hold on
plot(t,wheel_data,'r-')
plot(t_dl,wheel_data_dl,'r--')
title({'Wheel speed, $\dot{\alpha}_w$'},'interpreter','latex')
xlabel({'[\textit{s}]'},'interpreter','latex')
ylabel({'[\textit{RPM}]'},'interpreter','latex')
legend({'LQR','DL'},'interpreter','latex')

figure(3)
subplot(2,1,1)
hold on
plot(t,rollvel_data,'b-')
plot(t_dl,pitchvel_data,'r-')
title({'LQR Angular Velocities'},'interpreter','latex')
xlabel({'[\textit{s}]'},'interpreter','latex')
ylabel({'[$\frac{\textit{deg}}{\textit{s}}$]'},'interpreter','latex')
legend({'$\dot{\varphi}$','$\dot{\theta}$'},'interpreter','latex')
subplot(2,1,2)
hold on
plot(t,rollvel_data_dl,'b-')
plot(t_dl,pitchvel_data_dl,'r-')
title({'DL Angular Velocities'},'interpreter','latex')
xlabel({'[\textit{s}]'},'interpreter','latex')
ylabel({'[$\frac{\textit{deg}}{\textit{s}}$]'},'interpreter','latex')
legend({'$\dot{\varphi}$','$\dot{\theta}$'},'interpreter','latex')
% figure(2)
% figure(3)
% plot(wheel_data)
fprintf("LQR test %d:\n roll std = %f \n roll mean = %f \n pitch std = %f \n pitch mean = %f \n" + ...
    " dalpha_d std = %f \n dalpha_d mean = %f \n dalpha_w std = %f \n dalpha_w mean = %f \n ",...
    test,std(roll_data),mean(roll_data),std(pitch_data),mean(pitch_data),std(disk_data),mean(disk_data),...
    std(wheel_data),mean(wheel_data))
fprintf("\n")
fprintf("DL test %d:\n roll std = %f \n roll mean = %f \n pitch std = %f \n pitch mean = %f \n" + ...
    " dalpha_d std = %f \n dalpha_d mean = %f \n dalpha_w std = %f \n dalpha_w mean = %f \n ",...
    test,std(roll_data_dl),mean(roll_data_dl),std(pitch_data_dl),mean(pitch_data_dl),std(disk_data_dl),mean(disk_data_dl),...
    std(wheel_data_dl),mean(wheel_data_dl))



fprintf("\\textsc{lqr}\t&%d\t&%.4f\t&%.4f\t&%.4f\t&%.4f\t&" + ...
    "%.4f\t&%.4f\t&%.4f\t&%.4f\t\\\\ \\hline \n ",...
    test,std(roll_data),mean(roll_data),std(pitch_data),mean(pitch_data),std(disk_data),mean(disk_data),...
    std(wheel_data),mean(wheel_data))

fprintf("\\textsc{dl}\t&%d\t&%.4f\t&%.4f\t&%.4f\t&%.4f\t&" + ...
    "%.4f\t&%.4f\t&%.4f\t&%.4f\t\\\\ \\hline \n ",...
    test,std(roll_data_dl),mean(roll_data_dl),std(pitch_data_dl),mean(pitch_data_dl),std(disk_data_dl),mean(disk_data_dl),...
    std(wheel_data_dl),mean(wheel_data_dl))

% [rho,pval] = corr(rollvel_data,pitchvel_data)
% 
% 
% % figure(3)
% figure(1)
% subplot(2,2,2)
% hold on
% % plot(t(1:end-2),states(6,1:end-2)*180/pi)
% plot(t_dl,pitchvel_data_dl,'r-')
% plot(t_dl,rollvel_data_dl,'b-')
% plot([0 t_dl(end)],[-35 -35],'black--')
% plot([0 t_dl(end)],[35 35],'black--')
% title('DL')
% % figure(4)
% subplot(2,2,4)
% hold on
% % plot(t_dl(1:end-2),states(6,1:end-2)*180/pi)
% plot(t_dl,pitch_data_dl,'r-')
% plot(t_dl,roll_data_dl,'b-')
% plot([0 t_dl(end)],[-3 -3],'black--')
% plot([0 t_dl(end)],[3 3],'black--')
% title('DL')
% 
% figure(2)
% hold on
% subplot(2,1,1)
% hold on
% % plot(t_dl(1:end-2),states(6,1:end-2)*180/pi)
% plot(t_dl,wheel_data_dl,'r--')
% subplot(2,1,2)
% plot(t_dl,disk_data_dl,'b--')
% % plot([0 t_dl(end)],[-35 -35],'black--')
% % plot([0 t_dl(end)],[35 35],'black--')
% % title('DL')
% 
% corr(pitch_data_dl,roll_data_dl)
% fprintf("DL test %d:\n roll mean = %f \n pitch mean = %f \n" + ...
%     " roll std = %f \n pitch std = %f \n",test,mean(roll_data_dl),mean(pitch_data_dl),std(roll_data_dl),std(pitch_data_dl))
