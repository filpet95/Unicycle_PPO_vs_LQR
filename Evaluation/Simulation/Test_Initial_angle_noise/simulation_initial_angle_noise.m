
clc; clear variables; close all

%% ----------------------Simulation setup----------------------------------

max_angle = 45*pi/180;
max_angvel = 245*pi/180;
max_disk_vel = 440*2*pi/60;
max_wheel_vel = 90*2*pi/60;

noise_percent = .003;
nb_simulations = 100;
cov_noise = noise_percent*1e-2*[max_wheel_vel;max_disk_vel;max_angvel;max_angvel;max_angle;max_angle];
cov_noise = 5e-6.*ones(6,1);

% Sample time
Ts = 0.01;

% Simulation time
total_time = 15;

% observation size, if last input -> 8. 
obs_size = 6;

% Initial state to be Simulated. 
% init_state = [0 0 0 0 3*pi/180 3*pi/180]'; %DL 97% using 0.003% noise
init_state = [0 0 0 0 0*pi/180 19*pi/180]'; %DL 96% using 0.003% noise

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
addpath '../UNI_models'
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


LQR_fail = zeros(1,nb_simulations);
DL_fail = zeros(1,nb_simulations);
% time vector
t = 0:Ts:total_time;
bar = waitbar(0,"progress");
for j = 1:nb_simulations
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


% ---------------------Simulation-----------------------------------------


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
        [c_statesLQR,~] = NL_update_discrete_myode45(states_LQR(:,i),input_LQR(:,i),Ts,1e-3);
        [c_statesDL,~] = NL_update_discrete_myode45(states_DL(:,i),input_DL(:,i),Ts,1e-3);
        if any(abs(c_statesLQR(5:6)) > 45*pi/180)
            LQR_fail(j) = 1;
        end
        if any(abs(c_statesDL(5:6)) > 45*pi/180)
            DL_fail(j) = 1;
        end
        %save the state and observation
        noise_v = randn([6,1]).*sqrt(cov_noise);
        states_LQR(:,i+1) = c_statesLQR(:,end) + noise_v;
        states_DL(:,i+1) = c_statesDL(:,end) + noise_v;
        obs_LQR = [c_statesLQR(:,end);old_LQR(1)];
%         obs_LQR = c_statesLQR(:,end);
        obs_DL = c_statesDL(:,end);
        if obs_size == 8
            obs_LQR = [input_LQR(:,i);obs_LQR];
            obs_DL = [input_DL(:,i);obs_DL];
        end
    end
    waitbar(j/nb_simulations,bar,"Progress")
end
%% ----------------------Results-------------------------------------------
figure(1)
subplot(2,1,1)
hold on
title('Angles, LQR')
plot(t,180*states_LQR(5:6,1:end-1)/pi)
subplot(2,1,2)
hold on
title('Angles, DL')
plot(t,180*states_DL(5:6,1:end-1)/pi)

% figure(2)
% plot(t,input_DL)
fprintf("LQR failed %d times, which corresponds to %d%% accuracy \n",sum(LQR_fail),100-100*sum(LQR_fail)/length(LQR_fail))
fprintf("DL failed %d times, which corresponds to %d%% accuracy \n",sum(DL_fail),100-100*sum(DL_fail)/length(DL_fail))
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

