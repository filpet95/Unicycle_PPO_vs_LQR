
clc; clear variables; close all

%% ----------------------Simulation setup----------------------------------%
%roll_force = 19; %mAX DL
%roll_force = 20; %mAX LQR
roll_force = 0;
%pitch_force = 0;
<<<<<<< HEAD
pitch_force = 34; %MAX DL
%pitch_force = 49; %max LQR
=======
%pitch_force = 34; %MAX DL
pitch_force = 49; %max LQR
>>>>>>> 42b7e94b63ea4eb1b74ede943111d33dd58335d8

t_start = 1;
t_dur = 0.02;

% Sample time
Ts = 0.01;

% Simulation time
<<<<<<< HEAD
total_time = 5;
=======
total_time = 10;
>>>>>>> 42b7e94b63ea4eb1b74ede943111d33dd58335d8

% observation size, if last input -> 8. 
obs_size = 6;

% Initial state to be Simulated. 
init_state = [0 0 0 0 0*pi/180 0*pi/180]';

% The LQR gain to be used
% LQR_gain = 'K_1e-3_1e-3_200_1_1e2_10_1e-3_1e-2.mat';
% LQR_gain = 'K_filtered_.9_1_e-3_1e-3_200_1_1e2_10_1e-3_12e-3_12e-3.mat';
% LQR_gain = 'K_Simulation.mat';
LQR_gain = 'K_Hardware.mat';
% DL_model = 'Simulation_model.csv';
DL_model = 'DL_hardware.csv';


lambda_LQR = 0.9;
lambda_DL = 0.9;

% save('DL_model_wb2.mat','M')
%% ----------------------Loading, initiation-------------------------------
% Add the paths
addpath '../LQR_models'
addpath '../DL_models'
% addpath '../UNI_models'
addpath '../../helper_functions'
M = csvread(DL_model);

% Load the LQR gain
load(LQR_gain)

% Load The DL matrices
% load DL_model_wb2.mat
pi_fc0_w = M(1,1:384);
% pi_fc0_w = table2array(valuesfromDLtestsv2(1,1:384));
pi_fc0_b = M(2,1:64);
pi_fc1_w = M(3,1:4096);
pi_fc1_b = M(4,1:64);
pi_w = M(5,1:128);
pi_b = M(6,1:2);
   
new_pi_fc0_w = reshape(pi_fc0_w, [6,64]);
new_pi_fc1_w = reshape(pi_fc1_w, [64,64]);
new_pi_w = reshape(pi_w, [64,2]);


% time vector
t = 0:Ts:total_time;

% states for DL, LQR for plotting
states_LQR = zeros(6,length(t)+1);
states_DL = zeros(6,length(t)+1);

% inputs for DL, LQR for plotting
input_LQR = zeros(2,length(t));
input_DL = zeros(2,length(t));

% initiate first state
states_LQR(:,1) = init_state;
states_DL(:,1) = init_state;

%initiate weights
w_LQR = 0;
w_DL = 0;
old_LQR = 0;
old_DL = 0;

% initiate first observation 
obs_LQR = [init_state;0];
obs_DL = init_state;
if obs_size == 8
    obs_LQR = [zeros(2,1);obs_LQR];
    obs_DL = [zeros(2,1);obs_DL];
    
    %Make sure that K_d mult works if obs space is 8
    K_d = [zeros(2,2) K_d];
end


%% ---------------------Simulation-----------------------------------------
for i = 1:length(t)
    %Derive input for LQR and DL
    input_LQR(:,i) = predict_LQR(obs_LQR,K_d);
    input_DL(:,i) = predict_DL(obs_DL,new_pi_fc0_w, new_pi_fc1_w, new_pi_w,pi_fc0_b, pi_fc1_b, pi_b );
    w_LQR = lambda_LQR*w_LQR + 1;
    w_DL = lambda_DL*w_DL + 1;
    input_LQR(:,i) = (1-1/w_LQR)*old_LQR + (1/w_LQR)*input_LQR(:,i);
    input_DL(:,i) = (1-1/w_DL)*old_DL + (1/w_DL)*input_DL(:,i);
    old_LQR = input_LQR(:,i);
    old_DL = input_DL(:,i);
    
    if t(i) > t_start && t(i) < t_start+t_dur
        %Apply input on the system
        [c_statesLQR,~] = NL_update_discrete_myode45_external_forces(states_LQR(:,i),input_LQR(:,i),pitch_force,roll_force,Ts,1e-3);
        [c_statesDL,~] = NL_update_discrete_myode45_external_forces(states_DL(:,i),input_DL(:,i),pitch_force,roll_force,Ts,1e-3);
    else
        [c_statesLQR,~] = NL_update_discrete_myode45_external_forces(states_LQR(:,i),input_LQR(:,i),0,0,Ts,1e-3);
        [c_statesDL,~] = NL_update_discrete_myode45_external_forces(states_DL(:,i),input_DL(:,i),0,0,Ts,1e-3);
    end
    %save the state and observation
    states_LQR(:,i+1) = c_statesLQR(:,end);
    states_DL(:,i+1) = c_statesDL(:,end);
    obs_LQR = [c_statesLQR(:,end);old_LQR(1)];
%     obs_LQR = c_statesLQR(:,end);
    obs_DL = c_statesDL(:,end);
    if obs_size == 8
        obs_LQR = [input_LQR(:,i);obs_LQR];
        obs_DL = [input_DL(:,i);obs_DL];
    end
end

%% ----------------------Results-------------------------------------------
f = figure

hold on
plot(t,180*states_LQR(5,1:end-1)/pi,'b-')
plot(t,180*states_LQR(6,1:end-1)/pi,'r-')
legend({'$\varphi$','$\theta$'},'interpreter','latex','location','northeast')
xlabel(' [\textit{s}]','interpreter','latex')
ylabel('[\textit{deg}]','interpreter','latex')
%title('LQR','interpreter','latex')
f.Position = f.Position.*[1 1 1 0.5]
f = figure

hold on
plot(t,180*states_DL(5,1:end-1)/pi,'b-')
plot(t,180*states_DL(6,1:end-1)/pi,'r-')
legend({'$\varphi$','$\theta$'},'interpreter','latex','location','northeast')
xlabel('[\textit{s}]','interpreter','latex')
ylabel('[\textit{deg}]','interpreter','latex')
%title('DL','interpreter','latex')
f.Position = f.Position.*[1 1 1 0.5]
%%
legend({'$\dot{\varphi}$','$\dot{\theta}$'},'interpreter','latex','location','northeast')
xlabel('Time [\textit{s}]','interpreter','latex')
ylabel('[$\frac{\textit{deg}}{s}$]','interpreter','latex')
<<<<<<< HEAD
%title('LQR','interpreter','latex')
=======
title('LQR','interpreter','latex')
>>>>>>> 42b7e94b63ea4eb1b74ede943111d33dd58335d8
%%
legend({'$\varphi$','$\theta$'},'interpreter','latex','location','northeast')
xlabel('Time [\textit{s}]','interpreter','latex')
ylabel('[\textit{deg}]','interpreter','latex')
%%
figure(3)
subplot(2,1,1)
hold on
title('Motor vels, LQR')
plot(t,60*states_LQR(1:2,1:end-1)/(2*pi))
subplot(2,1,2)
hold on
title('Motor vels, DL')
plot(t,60*states_DL(1:2,1:end-1)/(2*pi))

figure(4)
subplot(2,1,1)
hold on
title('Ang Vels, LQR')
plot(t,180*states_LQR(3:4,1:end-1)/pi)
subplot(2,1,2)
hold on
title('Ang vels, DL')
plot(t,180*states_DL(3:4,1:end-1)/pi)
% figure(2)
% plot(t,input_DL)

%legend({'$\dot{\alpha_w}$','$\dot{\alpha_d}$'},'interpreter','latex','location','northeast')
%xlabel('Time [\textit{s}]','interpreter','latex')
%ylabel('Angular velocity [\textit{deg} / s]','interpreter','latex')
%title('LQR','interpreter','latex')
%hold off

%% ---------------------Helping functions ---------------------------------

function action = predict_LQR(obs,K_d)
    action = saturate(-K_d*obs,12);
end

function action = predict_DL(obs,new_pi_fc0_w, new_pi_fc1_w, new_pi_w,pi_fc0_b, pi_fc1_b, pi_b )
    out1 = tanh(new_pi_fc0_w' * obs + pi_fc0_b');
    out2 = tanh(new_pi_fc1_w' * out1 + pi_fc1_b');
    out3 = new_pi_w' * out2 + pi_b';
    action = saturate(out3,12);
end


