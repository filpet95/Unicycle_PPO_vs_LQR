
clc; clear variables; close all
addpath '../../helper_functions'
addpath 'report_models'
deffilem5sub
%% ----------------------Simulation setup----------------------------------
% Sample time
Ts = 0.01;

% Simulation time
total_time = 5;

% observation size, if last input -> 8. 
obs_size = 6;

% which angle and angular velocity the system is regarded stabilized in
stable_angle = 0.5;

% Initial state to be Simulated. 
% init_state = [0 0 0 0 8.15*pi/180 8.15*pi/180]'; % MAX DL
% init_state = [0 0 0 0 8.59*pi/180 8.59*pi/180]'; % MAX LQR

 init_state = [0 0 0 0 0*pi/180 0*pi/180]'; % MAX DL
% init_state = [0 0 0 0 0*pi/180 -28*pi/180]'; % MAX LQR

% init_state = [0 0 0 0 8*pi/180 0*pi/180]'; % MAX DL,LQR
% init_state = [0 0 0 0 9*pi/180 0*pi/180]'; % Both fail

%init_state = [0 0 0 0 0*pi/180 14*pi/180]'; 

%init_state = [0 0 0 0 -7.8*pi/180 0*pi/180]'; 

% The models to be used
LQR_gain = 'K_Hardware.mat';
% LQR_gain = 'K_Simulation_filtered.mat';
% LQR_gain = 'K_d.mat';
% LQR_gain = 'K_Simulation.mat';

DL_model = 'DL_hardware.csv';
%DL_model = 'base_model.csv' %init_state = [0 0 0 0 2*pi/180 8*pi/180]';
%DL_model = 'base_model_with_noise_right_model_inputpelaisation.csv'
%DL_model = 'base_model_with_noise_right_model.csv';
%DL_model = 'base_model_with_noise_first_right_model_inputpelaisation_s7.csv';
% DL_model = 'Simulation_model.csv';
%DL_model = 'new_base_model_no_noise.csv';
%DL_model = 'new_base_model_with_noise.csv';
%DL_model = 'new_base_model_with_noise4.csv';
% DL_model = 'noise+filters.csv';
% DL_model = 'noise+filters+inputpenalty.csv'; %CORRESPONDS TO UNI_v25
% DL_model = 'model_adjustment_modelv39.csv';
DL_model = 'reward_function_nr2.pkl.csv';

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
% obs_LQR = init_state;
obs_DL = init_state;
if obs_size == 8
    obs_LQR = [zeros(2,1);obs_LQR];
    obs_DL = [zeros(2,1);obs_DL];
    
    %Make sure that K_d mult works if obs space is 8
    K_d = [zeros(2,2) K_d];
end

DL_stable = false;
LQR_stable = false;
succ_lqr = true;
succ_dl = true;
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
    if succ_lqr
        [c_statesLQR,~] = NL_update_discrete_myode45(states_LQR(:,i),input_LQR(:,i),Ts,1e-3);
    else
        c_statesLQR = [0;0;0;0;states_LQR(5:6,i)];
        input_LQR(:,i) = [0;0];
    end
    if succ_dl
        [c_statesDL,~] = NL_update_discrete_myode45(states_DL(:,i),input_DL(:,i),Ts,1e-3);
    else
        c_statesDL = [0;0;0;0;states_DL(5:6,i)];
        input_DL(:,i) = [0;0];
    end
%     if all(abs(c_statesLQR(3:6,end)) < stable_angle*pi/180)
%         if LQR_stable == false
%             fprintf("LQR stabilized after %f seconds!\n",t(i))
%             LQR_stable = true;
%         end
%     end
    %if any(abs(c_statesLQR(5:6,end)) >= 90*pi/180)
    %    succ_lqr = false;
    %end
%     if all(abs(c_statesDL(3:6,end)) < stable_angle*pi/180)
%         if DL_stable == false
%             fprintf("DL stabilized after %f seconds!\n",t(i))
%             DL_stable = true;
%         end
%     end
    %if any(abs(c_statesDL(5:6,end)) >= 90*pi/180)
    %    succ_dl = false;
    %end
    %save the state and observation
    states_LQR(:,i+1) = c_statesLQR(:,end);
    states_DL(:,i+1) = c_statesDL(:,end);
    obs_LQR = [c_statesLQR(:,end);old_LQR(1)];
%     obs_LQR = c_statesLQR(:,end);
    obs_DL = c_statesDL(:,end);
end

roll_limit = 2;
ptch_limit = 2;
ptchvel_limit = 15;
rollvel_limit = 15;
disk_limit = 130;
wheel_limit = 20;
time_stable = 10;

% for i = 1:length(states_LQR)
% %     if abs(pitch_data(i)) < ptch_limit && abs(roll_data(i)) < roll_limit && abs(pitchvel_data(i)) < rollvel_limit && abs(rollvel_data(i)) < rollvel_limit
%     if abs(states_LQR(5,i))*180/pi < roll_limit && abs(states_LQR(2,i))*30/pi < disk_limit
% %         fprintf("test %d stabilized after %f seconds with overshoot = %f \n",test,i*0.01,max_oversht)
%         time_stable_LQR = i*0.01;
%         break
%     end
% end
% 
% for i = 1:length(states_DL)
% %     if abs(pitch_data(i)) < ptch_limit && abs(roll_data(i)) < roll_limit && abs(pitchvel_data(i)) < rollvel_limit && abs(rollvel_data(i)) < rollvel_limit
%     if abs(states_DL(5,i))*180/pi < roll_limit && abs(states_DL(2,i))*30/pi < disk_limit
% %         fprintf("test %d stabilized after %f seconds with overshoot = %f \n",test,i*0.01,max_oversht)
%         time_stable_DL = i*0.01;
%         break
%     end
% end
for i = 1:length(states_LQR)
%     if abs(pitch_data(i)) < ptch_limit && abs(roll_data(i)) < roll_limit && abs(pitchvel_data(i)) < rollvel_limit && abs(rollvel_data(i)) < rollvel_limit
    if abs(states_LQR(6,i))*180/pi < ptch_limit && abs(states_LQR(1,i))*30/pi < wheel_limit
%         fprintf("test %d stabilized after %f seconds with overshoot = %f \n",test,i*0.01,max_oversht)
        time_stable_LQR = i*0.01;
        break
    end
end

for i = 1:length(states_DL)
%     if abs(pitch_data(i)) < ptch_limit && abs(roll_data(i)) < roll_limit && abs(pitchvel_data(i)) < rollvel_limit && abs(rollvel_data(i)) < rollvel_limit
    if abs(states_DL(6,i))*180/pi < ptch_limit && abs(states_DL(1,i))*30/pi < wheel_limit
%         fprintf("test %d stabilized after %f seconds with overshoot = %f \n",test,i*0.01,max_oversht)
        time_stable_DL = i*0.01;
        break
    end
end

% max_oversht_LQR = -min(states_LQR(5,:))*180/pi;
% max_oversht_DL = -min(states_DL(5,:))*180/pi;
max_oversht_LQR = -min(states_LQR(6,:))*180/pi;
max_oversht_DL = -min(states_DL(6,:))*180/pi;

%fprintf("Time of stabilization,\t LQR:%.4f,\t DL:%.4f\nOvershoot,\t LQR:%.4f,\t DL:%.4f\n:",time_stable_LQR,time_stable_DL,max_oversht_LQR,max_oversht_DL)
%% ----------------------Results-------------------------------------------
f1 = figure;
hold on
plot(t,180*states_DL(5,1:end-1)/pi,'b-')
plot(t,180*states_DL(6,1:end-1)/pi,'r-')
%plot([time_stable_DL time_stable_DL],[-5,14],'black--')
%plot([0 t(end)],[-max_oversht_DL -max_oversht_DL],'black-.')
legend({'$\varphi$','$\theta$','$t_s$','$M_p$'},'interpreter','latex')
xlabel({'[\textit{s}]'},'interpreter','latex')
ylabel({'[\textit{deg}]'},'interpreter','latex')
% legend({'$\varphi$','$\theta$'},'interpreter','latex')
axis([0,t(end),-10 10])
f1.Position = f1.Position.*[1 1 1 0.5];

% 
f = figure
hold on
 %title('DL','interpreter','latex')
 plot(t,input_DL(2,1:end),'b-')
 plot(t,input_DL(1,1:end),'r-')
 xlabel({'[\textit{s}]'},'interpreter','latex')
 ylabel({'[$\textit{V}$]'},'interpreter','latex')
 legend({'$u_d$','$u_w$'},'interpreter','latex')
 axis([0,t(end),-13,13])
 f.Position = f.Position.*[1 1 1 0.5];
% 




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


