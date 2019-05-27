clc; clear variables; close all

addpath('Ideal_Test')
addpath('../../Modelling_v3')
addpath('../Simulation/LQR_models')
addpath '../helper_functions'
deffilem5sub
tst_data_LQR = csvread('LQR_14_battery_test1.csv',1,0);
tst_data_DL = csvread('DL_14_battery_test3.csv',1,0);
LQR_gain = 'K_Simulation.mat';
load(LQR_gain)

enable_data_LQR = tst_data_LQR(2:end,10);
enable_data_DL = tst_data_DL(2:end,10);

enable_idx_LQR = find(enable_data_LQR,1);
enable_idx_DL = find(enable_data_DL,1);
pitch_data_LQR = tst_data_LQR(:,2);
roll_data_LQR = tst_data_LQR(:,3);
pitch_data_DL = tst_data_DL(:,2);
roll_data_DL = tst_data_DL(:,3);

disable_idx_LQR = find(abs(pitch_data_LQR)>40,1);
if isempty(disable_idx_LQR)
    disable_idx_LQR = length(tst_data_LQR);
end
pitch_data_LQR = pitch_data_LQR(enable_idx_LQR:disable_idx_LQR);
pitchvel_data_LQR = tst_data_LQR(enable_idx_LQR:disable_idx_LQR,7);
roll_data_LQR = roll_data_LQR(enable_idx_LQR:disable_idx_LQR);
rollvel_data_LQR = tst_data_LQR(enable_idx_LQR:disable_idx_LQR,6);


disable_idx_DL = find(abs(pitch_data_DL)>40,1);
if isempty(disable_idx_DL)
    disable_idx_DL = length(tst_data_DL);
end
pitch_data_DL = pitch_data_DL(enable_idx_DL:disable_idx_DL);
pitchvel_data_DL = tst_data_DL(enable_idx_DL:disable_idx_DL,7);
roll_data_DL = roll_data_DL(enable_idx_DL:disable_idx_DL);
rollvel_data_DL = tst_data_DL(enable_idx_DL:disable_idx_DL,6);


Ts = 0.01;
t = 0:Ts:3;



figure(1)
subplot(2,1,1)
hold on
if length(t)>length(pitch_data_LQR)
    plot(t(1:length(pitch_data_LQR)),pitch_data_LQR,'r-')
    plot(t(1:length(pitch_data_LQR)),roll_data_LQR,'b-')
else 
    plot(t(1:end-2),pitch_data_LQR(3:length(t)),'r-')
    plot(t(1:end-2),roll_data_LQR(3:length(t)), 'b-')
end
legend({"$\theta$",'$\varphi$'},'interpreter','latex','location','northeast')
xlabel('Time [\textit{s}]','interpreter','latex')
ylabel('Angle [\textit{deg}]','interpreter','latex')
title('LQR','interpreter','latex')
hold off

subplot(2,1,2)
hold on
if length(t)>length(pitch_data_DL)
    plot(t(1:length(pitch_data_DL)),pitch_data_DL,'r-')
    plot(t(1:length(pitch_data_DL)),roll_data_DL,'b-')
else 
    plot(t(1:end-2),pitch_data_DL(3:length(t)),'r-')
    plot(t(1:end-2),roll_data_DL(3:length(t)),'b-')
end

legend({'$\theta$','$\varphi$'},'interpreter','latex','location','northeast')
xlabel('Time [\textit{s}]','interpreter','latex')
ylabel('Angle [\textit{deg}]','interpreter','latex')
title('DL','interpreter','latex')


