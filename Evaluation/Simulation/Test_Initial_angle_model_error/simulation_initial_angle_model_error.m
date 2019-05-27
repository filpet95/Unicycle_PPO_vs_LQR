clc; clear variables; close all

%% ----------------------Simulation setup----------------------------------
% Sample time
Ts = 0.01;

% Simulation time
total_time = 5;

% observation size, if last input -> 8. 
obs_size = 6;

% Initial state to be Simulated. 
% init_state = [0 0 0 0 6.55*pi/180 6.55*pi/180]';
init_state_LQR = [0 0 0 0 0*pi/180 28*pi/180]'; %max -6 til 9 (DL) og 11 (LQR)
init_state_DL = [0 0 0 0 0*pi/180 21*pi/180]';
% init_state_LQR = [0 0 0 0 11*pi/180 0*pi/180]'; %max -6 til 9 (DL) og 11 (LQR)
% init_state_DL = [0 0 0 0 9*pi/180 0*pi/180]'
% init_state = [0 0 0 0 12*pi/180 12*pi/180]';


% The LQR gain to be used
% LQR_gain = 'K_1e-3_1e-3_200_1_1e2_10_1e-3_1e-2.mat';
% LQR_gain = 'K_filtered_.9_1_e-3_1e-3_200_1_1e2_10_1e-3_12e-3_12e-3.mat';
% LQR_gain = 'K_Simulation.mat';
LQR_gain = 'K_Hardware.mat';
% DL_model = 'Simulation_model.csv';
DL_model = 'DL_hardware.csv';
% DL_model = 'noise+filters+inputpenalty.csv';

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
deffilem5sub
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
states_LQR(:,1) = init_state_LQR;
states_DL(:,1) = init_state_DL;

%initiate weights
w_LQR = 0;
w_DL = 0;
old_LQR = 0;
old_DL = 0;

% initiate first observation 
obs_LQR = [init_state_LQR;0];
obs_DL = init_state_DL;
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
    
    
    %Apply input on the system
    [c_statesLQR,~] = NL_update_discrete_myode45_model_error(states_LQR(:,i),input_LQR(:,i),Ts,1e-3);
    [c_statesDL,~] = NL_update_discrete_myode45_model_error(states_DL(:,i),input_DL(:,i),Ts,1e-3);
    
    %save the state and observation
    states_LQR(:,i+1) = c_statesLQR(:,end);
    states_DL(:,i+1) = c_statesDL(:,end);
%     obs_LQR = [c_statesLQR(:,end);old_LQR(1)];
    obs_LQR = [c_statesLQR(:,end);old_LQR(1)];
    obs_DL = c_statesDL(:,end);
    if obs_size == 8
        obs_LQR = [input_LQR(:,i);obs_LQR];
        obs_DL = [input_DL(:,i);obs_DL];
    end
end

%% ----------------------Results-------------------------------------------
% figure(1)
% subplot(2,1,1)
% hold on
% title('Angles, LQR')
% plot(t,180*states_LQR(5:6,1:end-1)/pi)
% subplot(2,1,2)
% hold on
% title('Angles, DL')
% plot(t,180*states_DL(5:6,1:end-1)/pi)
% 
% % figure(2)
% % plot(t,input_DL)


f = figure

hold on
plot(t,180*states_LQR(5,1:end-1)/pi,'b-')
plot(t,180*states_LQR(6,1:end-1)/pi,'r-')
% plot(t,2.8828*ones(1,length(t)),'m--')
plot(t,zeros(1,length(t)),'black--','HandleVisibility','off')
plot([0 5],[2.883 2.883],'m--')

legend({"$\dot{\alpha_d}$",'$\dot{\alpha_w}$','$\mu_{LQR}$'},'interpreter','latex','location','northeast')
xlabel('[\textit{s}]','interpreter','latex')
ylabel('[\textit{deg}]','interpreter','latex')
ylim([-15 29])
%title('LQR','interpreter','latex')
hold off
f.Position = f.Position.*[1 1 1 0.5]

f = figure
hold on
plot(t,180*states_DL(5,1:end-1)/pi,'b-')
plot(t,180*states_DL(6,1:end-1)/pi,'r-')
% plot(t,2.8828*ones(1,length(t)),'m--')
plot(t,zeros(1,length(t)),'black--','HandleVisibility','off')
plot([0 5],[2.883 2.883],'m--')

legend({"$\dot{\alpha_d}$",'$\dot{\alpha_w}$','$\mu_{DL}$'},'interpreter','latex','location','northeast')
xlabel('[\textit{s}]','interpreter','latex')
ylabel('[\textit{deg}]','interpreter','latex')
ylim([-15 29])
%title('DL','interpreter','latex')
hold off
f.Position = f.Position.*[1 1 1 0.5]
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


