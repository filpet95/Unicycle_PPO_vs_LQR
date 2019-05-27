clc; clear variables; close all
% addpath('C:\Users\filpe\Dropbox\Chalmers_stuff\SSYX04 Master Thesis\Unicycle_Project\Modelling')
load 'linear_model.mat'
tic
GAMMA = 0.99;
sim_init = [zeros(4,1);0*pi/180;25*pi/180];
t = 0:Ts:3;
states = zeros(6,length(t));
states(:,1) = sim_init;

succ = 1;   
alpha_w = 0;
enable = 1; %enable control
inp_v = zeros(2,length(t));
phi = 0;
theta = 0;
for i = 1:length(t)
    if succ ~= -1
        inp_v(:,i) = -K_d*states(:,i);
        inp_v(:,i) = inp_v(:,i)*enable;
        if inp_v(1,i) > 12
            inp_v(1,i) = 12;a
        elseif inp_v(1,i) < -12
            inp_v(1,i) = -12;
        end
        if inp_v(2,i) > 12
            inp_v(2,i) = 12;
        elseif inp_v(2,i) < -12
            inp_v(2,i) = -12;
        end
        if abs(inp_v(1,i)) < 1
            inp_v(1,i) = 0;
        end
        if abs(inp_v(2,i))< 1
            inp_v(2,i) = 0;
        end
%         inp_v(:,i) = [-10;0];
        [states_myode,t_my_ode] = NL_update_discrete_myode45(states(:,i),inp_v(:,i),Ts,1e-3);
        states(:,i+1) = states_myode(:,end);
        alpha_w = alpha_w + states(1,i)*Ts;
    end
end
figure(1)
hold on
plot(t+Ts,states(5:6,2:end)*180/pi)
legend("phi","theta")
% figure(2)
% hold on
% plot(t+Ts,states(2,1:end-1),'r')
% figure(3)
% plot(t(1:end-1)+Ts,inp_v(1,2:end),'*')
%generate_cpp_vector(inp_v(2,:),'inp_v')
toc 
