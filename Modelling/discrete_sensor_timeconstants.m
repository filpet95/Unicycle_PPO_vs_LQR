clc; clear variables; close all
% addpath('C:\Users\filpe\Dropbox\Chalmers_stuff\SSYX04 Master Thesis\Unicycle_Project\Modelling')
% load 'linear_model.mat'
Ts = 0.01;
sim_init = [zeros(4,1);0*pi/180;0*pi/180];
t = 0:Ts:3;
states = zeros(6,length(t));
states(:,1) = sim_init;
states_filt = zeros(6,length(t));
states_filt(:,1) = sim_init;
states_obsv = zeros(6,length(t));
states_obsv(:,1) = sim_init;
succ = 1;   
alpha_w = 0;
enable = 1; %enable control
inp_v = zeros(2,length(t));
inp_v_filt = zeros(2,length(t));
phi = 0;
theta = 0;
weights = zeros(6,1);
xp_phi = [0;0];
Pp_phi = 1*eye(2);
xp_theta = [0;0];
Pp_theta = 1*eye(2);
obsv = sim_init;
for i = 1:length(t)
    if succ ~= -1
%         inp_v(:,i) = -K_d*states(:,i);
%         inp_v(:,i) = inp_v(:,i)*enable;
%         inp_v_filt(:,i) = -K_d*obsv;
%         inp_v_filt(:,i) = inp_v_filt(:,i)*enable;
        inp_v(:,i) = 12;
        inp_v_filt(:,i) = 12;
%         if inp_v(1,i) > 12
%             inp_v(1,i) = 12;
%         elseif inp_v(1,i) < -12
%             inp_v(1,i) = -12;
%         end
%         if inp_v(2,i) > 12
%             inp_v(2,i) = 12;
%         elseif inp_v(2,i) < -12
%             inp_v(2,i) = -12;
%         end
%         if abs(inp_v(1,i)) < 1
%             inp_v(1,i) = 0;
%         end
%         if abs(inp_v(2,i))< 1
%             inp_v(2,i) = 0;
%         end
        [states_myode,t_my_ode] = NL_update_discrete_myode45(states(:,i),inp_v(:,i),Ts,1e-3);
        states(:,i+1) = states_myode(:,end);
        [states_myode,t_my_ode] = NL_update_discrete_myode45(states_filt(:,i),inp_v_filt(:,i),Ts,1e-3);
        states_filt(:,i+1) = states_myode(:,end);

        [obsv_dalpha,weights] = filter_dalpha(states_filt(1:2,i+1),obsv(1:2),weights,~[true true]);        
        [x_phi,P_phi,xp_phi,Pp_phi] = filter_dangles(xp_phi,Pp_phi,states_filt(3,i+1),true);
        [x_theta,P_theta,xp_theta,Pp_theta] = filter_dangles(xp_theta,Pp_theta,states_filt(4,i+1),true);
        
        obsv_angles = filter_angles(states_filt(5:6,i+1),obsv(5:6),states_filt(3:4,i+1),~[true,true]);
        obsv(1:2) = obsv_dalpha;
        obsv(3) = x_phi(1);
        obsv(4) = x_theta(1);
        obsv(5:end) = obsv_angles;
        states_obsv(:,i+1) = obsv;
    end
end
figure(1)
hold on
plot(t+Ts,states_obsv(5,2:end)*180/pi,'b-')
plot(t+Ts,states_obsv(6,2:end)*180/pi,'r-')
plot(t+Ts,states(5,2:end)*180/pi,'b--')
plot(t+Ts,states(6,2:end)*180/pi,'r--')
legend({'Observed $\varphi$','Observed $\theta$','True $\varphi$','True $\theta$'},'interpreter','latex','location','best')
xlabel('Time [$s$]','interpreter','latex')
ylabel('Angle [\textit{deg}]','interpreter','latex')
figure(2)
hold on
plot(t+Ts,states_obsv(1,1:end-1)*30/pi,'r-')
plot(t+Ts,states_obsv(2,1:end-1)*30/pi,'b-')
plot(t+Ts,states(1,1:end-1)*30/pi,'r--')
plot(t+Ts,states(2,1:end-1)*30/pi,'b--')
legend({'Observed $\dot\alpha_w$','Observed $\dot\alpha_d$','True $\dot\alpha_w$','True $\dot\alpha_d$'},'interpreter','latex','location','best')
xlabel('Time [$s$]','interpreter','latex')
ylabel('Motor Velocities [\textit{RPM}]','interpreter','latex')
figure(3)

hold on
plot(t+Ts,states_obsv(3,1:end-1)*180/pi,'b-')
plot(t+Ts,states_obsv(4,1:end-1)*180/pi,'r-')
plot(t+Ts,states(3,1:end-1)*180/pi,'b--')
plot(t+Ts,states(4,1:end-1)*180/pi,'r--')
legend({'Observed $\dot\varphi$','Observed $\dot\theta$','True $\dot\varphi$','True $\dot\theta$'},'interpreter','latex','location','best')
xlabel('Time [$s$]','interpreter','latex')
ylabel('Angular Velocities [$\frac{deg}{s}$]','interpreter','latex')
% figure(4)
% hold on
% plot(t(1:end-1)+Ts,inp_v_filt(1:2,2:end),'b')
% plot(t(1:end-1)+Ts,inp_v(1:2,2:end),'r--')
%generate_cpp_vector(inp_v(2,:),'inp_v')

function [obsv,weights] = filter_dalpha(states,old_states,weights,enable)
    
obsv = states;
if enable(1)%dalpha_w
    weights(1) = 0.45*weights(1) + 1;
    obsv(1) = (1-1/weights(1))*old_states(1) + states(1)/weights(1);
end
if enable(2)%dalpha_d
    weights(2) = 0.45*weights(2) + 1;
    obsv(2) = (1-1/weights(2))*old_states(2) + states(2)/weights(2);
end
end
function [x,P,xp,Pp] = filter_dangles(xp,Pp,meas,enable)
x = [meas;0];
P = zeros(2,2);
if(enable)
    Ts = 0.01;
    A = [1 Ts;0 1];
    H = [1 0];
    Q = [0 0;0 5e-4];
    R = 5e-6;
    %innovation covariance:
    S = H*Pp*H.'+R;
    %innovation: 
    v = meas-H*xp;
    %Kalman gain:
    K = Pp*H.'/S;
    
    x = xp + K*v;
    P = Pp-K*S*K.';
    
    xp = A*x;
    Pp = A*P*A.' + Q;
end
end
function obsv = filter_angles(states,old_states,vel,enable)
%     phi = (1-GAMMA)*phi + GAMMA*states(5,i+1);
obsv = states;
if enable(1)%phi
    obsv(1) = (1-0.99)*states(1) + 0.99*(old_states(1)+0.01*vel(1));
end
if enable(2)%theta
    obsv(2) = (1-0.99)*states(2) + 0.99*(old_states(2)+0.01*vel(2));
end
end