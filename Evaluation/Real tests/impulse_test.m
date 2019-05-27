%% model error tests 25.04.19
clear all; close all; clc
%%

addpath('Impulse_Test')
addpath ('../helper_functions')
deffilem5sub
% for roll figures - model error
<<<<<<< HEAD
tst_data_LQR = csvread('_5_LQR_t4.csv',1,0); %best so fer = 1 
=======
tst_data_LQR = csvread('_5_LQR_t1.csv',1,0); %best so fer = 1 
>>>>>>> 42b7e94b63ea4eb1b74ede943111d33dd58335d8
tst_data_DL = csvread('_5_DL_t4.csv',1,0); %best so far = 4
%for pitch figures - model error
%tst_data_LQR = csvread('LQR_modelerror_pitch_29_test1.csv',1,0);
%tst_data_DL = csvread('DL_modelerror_pitch_21_test1.csv',1,0);


enable_data_LQR = tst_data_LQR(1:end,10);
enable_data_DL = tst_data_DL(1:end,10);
enable_idx_LQR = find(enable_data_LQR,1);
enable_idx_DL = find(enable_data_DL,1);

%pitch of DL and LQR
pitch_data_LQR = tst_data_LQR(:,9);
pitch_data_DL = tst_data_DL(:,9);

%LQR data
disable_idx_LQR = find(abs(pitch_data_LQR)>40,1);
if isempty(disable_idx_LQR)
    disable_idx_LQR = length(tst_data_LQR);
end
%pitch_data_LQR = pitch_data_LQR(enable_idx_LQR:disable_idx_LQR);

wheelinput_data_LQR = tst_data_LQR(enable_idx_LQR:disable_idx_LQR,2);
diskinput_data_LQR = tst_data_LQR(enable_idx_LQR:disable_idx_LQR,3);
dalphaw_data_LQR = tst_data_LQR(enable_idx_LQR:disable_idx_LQR,4);
dalphad_data_LQR = tst_data_LQR(enable_idx_LQR:disable_idx_LQR,5);
dphi_data_LQR = tst_data_LQR(enable_idx_LQR:disable_idx_LQR,6);
dtheta_data_LQR = tst_data_LQR(enable_idx_LQR:disable_idx_LQR,7);
roll_data_LQR = tst_data_LQR(enable_idx_LQR:disable_idx_LQR,8);
pitch_data_LQR = tst_data_LQR(enable_idx_LQR:disable_idx_LQR,9);

fprintf('Inital values for LQR: \n u_w = %f \n u_d = %f \n dalpha_w = %f \n dalpa_d = %f \n dphi = %f \n dtheta = %f \n roll = %f \n pitch = %f \n',...
        wheelinput_data_LQR(1),diskinput_data_LQR(1),dalphaw_data_LQR(1),dalphad_data_LQR(1),dphi_data_LQR(1),dtheta_data_LQR(1),roll_data_LQR(1),pitch_data_LQR(1))


%DL data
disable_idx_DL = find(abs(pitch_data_DL)>40,1);
if isempty(disable_idx_DL)
    disable_idx_DL = length(tst_data_DL);
end

wheelinput_data_DL = tst_data_DL(enable_idx_DL:disable_idx_DL,2);
diskinput_data_DL = tst_data_DL(enable_idx_DL:disable_idx_DL,3);
dalphaw_data_DL = tst_data_DL(enable_idx_DL:disable_idx_DL,4);
dalphad_data_DL = tst_data_DL(enable_idx_DL:disable_idx_DL,5);
dphi_data_DL = tst_data_DL(enable_idx_DL:disable_idx_DL,6);
dtheta_data_DL = tst_data_DL(enable_idx_DL:disable_idx_DL,7);
roll_data_DL = tst_data_DL(enable_idx_DL:disable_idx_DL,8);
pitch_data_DL = tst_data_DL(enable_idx_DL:disable_idx_DL,9);

fprintf('Inital values for DL: \n u_w = %f \n u_d = %f \n dalpha_w = %f \n dalpa_d = %f \n dphi = %f \n dtheta = %f \n roll = %f \n pitch = %f \n',...
        wheelinput_data_DL(1),diskinput_data_DL(1),dalphaw_data_DL(1),dalphad_data_DL(1),dphi_data_DL(1),dtheta_data_DL(1),roll_data_DL(1),pitch_data_DL(1))

Ts = 0.01;
<<<<<<< HEAD
t = 0:Ts:2;
=======
t = 0:Ts:3;
>>>>>>> 42b7e94b63ea4eb1b74ede943111d33dd58335d8
% PLotting ------------

f = figure
%subplot(2,1,1)
hold on
if length(t)>length(pitch_data_LQR)
    plot(t(1:length(pitch_data_LQR)),pitch_data_LQR,'r-')
    plot(t(1:length(pitch_data_LQR)),roll_data_LQR,'b-')
    %plot(t(1:length(pitch_data_LQR)),zeros(1,length(roll_data_LQR)),'--')
<<<<<<< HEAD
    %plot(t(1:length(pitch_data_LQR)),2*ones(1,length(roll_data_LQR)),'--')
    %plot(t(1:length(pitch_data_LQR)),-2*ones(1,length(roll_data_LQR)),'--')
=======
    plot(t(1:length(pitch_data_LQR)),2*ones(1,length(roll_data_LQR)),'--')
    plot(t(1:length(pitch_data_LQR)),-2*ones(1,length(roll_data_LQR)),'--')
>>>>>>> 42b7e94b63ea4eb1b74ede943111d33dd58335d8
 
else 
    plot(t(1:end-2),pitch_data_LQR(3:length(t)),'r-')
    plot(t(1:end-2),roll_data_LQR(3:length(t)), 'b-')
    %plot(t(1:end-2),zeros(1,length(t)-2), '--')
<<<<<<< HEAD
    %plot(t(1:end-2),2*ones(1,length(t)-2), '--')
    %plot(t(1:end-2),-2*ones(1,length(t)-2), '--')
=======
    plot(t(1:end-2),2*ones(1,length(t)-2), '--')
    plot(t(1:end-2),-2*ones(1,length(t)-2), '--')
>>>>>>> 42b7e94b63ea4eb1b74ede943111d33dd58335d8

end
legend({"$\theta$",'$\varphi$','$\delta_{\alpha}$'},'interpreter','latex','location','northeast')
xlabel('[\textit{s}]','interpreter','latex')
ylabel('[\textit{deg}]','interpreter','latex')
%title('LQR','interpreter','latex')
hold off
f.Position = f.Position.*[1 1 1 0.5]

f = figure
%subplot(2,1,2)
hold on
if length(t)>length(pitch_data_DL)
    plot(t(1:length(pitch_data_DL)),pitch_data_DL,'r-')
    plot(t(1:length(pitch_data_DL)),roll_data_DL,'b-')
    %plot(t(1:length(pitch_data_DL)),zeros(1,length(roll_data_DL)),'--')
<<<<<<< HEAD
    %plot(t(1:length(pitch_data_DL)),2*ones(1,length(roll_data_DL)),'--')
    %plot(t(1:length(pitch_data_DL)),-2*ones(1,length(roll_data_DL)),'--')
=======
    plot(t(1:length(pitch_data_DL)),2*ones(1,length(roll_data_DL)),'--')
    plot(t(1:length(pitch_data_DL)),-2*ones(1,length(roll_data_DL)),'--')
>>>>>>> 42b7e94b63ea4eb1b74ede943111d33dd58335d8
else 
    plot(t(1:end-2),pitch_data_DL(3:length(t)),'r-')
    plot(t(1:end-2),roll_data_DL(3:length(t)),'b-')
   % plot(t(1:end-2),zeros(1,length(t)-2), '--')
<<<<<<< HEAD
    %plot(t(1:end-2),2*ones(1,length(t)-2), '--')
    %plot(t(1:end-2),-2*ones(1,length(t)-2), '--')
=======
    plot(t(1:end-2),2*ones(1,length(t)-2), '--')
    plot(t(1:end-2),-2*ones(1,length(t)-2), '--')
>>>>>>> 42b7e94b63ea4eb1b74ede943111d33dd58335d8
    
end

legend({'$\theta$','$\varphi$','$\delta_{\alpha}$'},'interpreter','latex','location','northeast')
xlabel('[\textit{s}]','interpreter','latex')
ylabel('[\textit{deg}]','interpreter','latex')
%title('DL','interpreter','latex')
hold off
f.Position = f.Position.*[1 1 1 0.5]
%% plotting inputs

figure
hold on
if length(t)>length(pitch_data_LQR)
    plot(t(1:length(pitch_data_LQR)),wheelinput_data_LQR,'r-')
    plot(t(1:length(pitch_data_LQR)),diskinput_data_LQR,'b-')
else 
    plot(t(1:end-2),wheelinput_data_LQR(3:length(t)),'r-')
    plot(t(1:end-2),diskinput_data_LQR(3:length(t)),'b-')
end

legend({'$u_w$','$u_d$'},'interpreter','latex','location','northeast')
xlabel('Time [\textit{s}]','interpreter','latex')
ylabel('Voltage [v]','interpreter','latex')
title('LQR','interpreter','latex')
hold off

figure
hold on
if length(t)>length(pitch_data_DL)
    plot(t(1:length(pitch_data_DL)),wheelinput_data_DL,'r-')
    plot(t(1:length(pitch_data_DL)),diskinput_data_DL,'b-')
else 
    plot(t(1:end-2),wheelinput_data_DL(3:length(t)),'r-')
    plot(t(1:end-2),diskinput_data_DL(3:length(t)),'b-')
end

legend({'$u_w$','$u_d$'},'interpreter','latex','location','northeast')
xlabel('Time [\textit{s}]','interpreter','latex')
ylabel('Voltage [v]','interpreter','latex')
title('DL','interpreter','latex')
hold off

%% plotting angular velocities of disk and wheel

figure
subplot(2,1,1)
hold on
if length(t)>length(pitch_data_LQR)
    plot(t(1:length(pitch_data_LQR)),dalphaw_data_LQR,'r-')
    plot(t(1:length(pitch_data_LQR)),dalphad_data_LQR,'b-')
    plot(t(1:length(pitch_data_DL)),130*ones(1,length(roll_data_DL)),'--')
    plot(t(1:length(pitch_data_DL)),-130*ones(1,length(roll_data_DL)),'--')
else 
    plot(t(1:end-2),dalphaw_data_LQR(3:length(t)),'r-')
    plot(t(1:end-2),dalphad_data_LQR(3:length(t)),'b-')
    plot(t(1:end-2),130*ones(1,length(t)-2), '--')
    plot(t(1:end-2),-130*ones(1,length(t)-2), '--')
end

legend({'$\dot{\varphi}$','$\dot{\theta}$', '$\delta_{\omega}$'},'interpreter','latex','location','northeast')
xlabel('Time [\textit{s}]','interpreter','latex')
ylabel('[$\frac{\textit{deg}}{s}$]','interpreter','latex')
title('LQR','interpreter','latex')
hold off

subplot(2,1,2)
hold on
if length(t)>length(pitch_data_DL)
    plot(t(1:length(pitch_data_DL)),dalphaw_data_DL,'r-')
    plot(t(1:length(pitch_data_DL)),dalphad_data_DL,'b-')
    plot(t(1:length(pitch_data_DL)),2*ones(1,length(roll_data_DL)),'--')
    plot(t(1:length(pitch_data_DL)),-2*ones(1,length(roll_data_DL)),'--')
else 
    plot(t(1:end-2),dalphaw_data_DL(3:length(t)),'r-')
    plot(t(1:end-2),dalphad_data_DL(3:length(t)),'b-')
    plot(t(1:end-2),130*ones(1,length(t)-2), '--')
    plot(t(1:end-2),-130*ones(1,length(t)-2), '--')
end

legend({'$\dot{\varphi}$','$\dot{\theta}$', '$\delta_{\omega}$'},'interpreter','latex','location','northeast')
xlabel('Time [\textit{s}]','interpreter','latex')
ylabel('[$\frac{\textit{deg}}{s}$]','interpreter','latex')
title('DL','interpreter','latex')
hold off

%% plotting angular velocity 

figure
subplot(2,1,1)
hold on
if length(t)>length(pitch_data_LQR)
    plot(t(1:length(pitch_data_LQR)),dphi_data_LQR,'b-')
    plot(t(1:length(pitch_data_LQR)),dtheta_data_LQR,'r-')
    %plot(t(1:length(pitch_data_DL)),130*ones(1,length(roll_data_DL)),'--')
    %plot(t(1:length(pitch_data_DL)),-130*ones(1,length(roll_data_DL)),'--')
else 
    plot(t(1:end-2),dphi_data_LQR(3:length(t)),'b-')
    plot(t(1:end-2),dtheta_data_LQR(3:length(t)),'r-')
    %plot(t(1:end-2),130*ones(1,length(t)-2), '--')
    %plot(t(1:end-2),-130*ones(1,length(t)-2), '--')
end

legend({'$\dot{\varphi}$','$\dot{\theta}$', '$\delta_{\alpha}$'},'interpreter','latex','location','northeast')
xlabel('Time [\textit{s}]','interpreter','latex')
ylabel('[$\frac{\textit{deg}}{s}$]','interpreter','latex')
title('LQR','interpreter','latex')
hold off

subplot(2,1,2)
hold on
if length(t)>length(pitch_data_DL)
    plot(t(1:length(pitch_data_DL)),dphi_data_DL,'b-')
    plot(t(1:length(pitch_data_DL)),dtheta_data_DL,'r-')
    %plot(t(1:length(pitch_data_DL)),2*ones(1,length(roll_data_DL)),'--')
    %plot(t(1:length(pitch_data_DL)),-2*ones(1,length(roll_data_DL)),'--')
else 
    plot(t(1:end-2),dphi_data_DL(3:length(t)),'b-')
    plot(t(1:end-2),dtheta_data_DL(3:length(t)),'r-')
    %plot(t(1:end-2),130*ones(1,length(t)-2), '--')
    %plot(t(1:end-2),-130*ones(1,length(t)-2), '--')
end

legend({'$\dot{\varphi}$','$\dot{\theta}$', '$\delta_a$'},'interpreter','latex','location','northeast')
xlabel('Time [\textit{s}]','interpreter','latex')
ylabel('[$\frac{\textit{deg}}{s}$]','interpreter','latex')
title('DL','interpreter','latex')
hold off

