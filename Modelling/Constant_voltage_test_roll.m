clc; clear variables; close all
addpath('model_tests')
% -8 -> test2
% -5 -> test4
% -2 -> test5
tst_data = csvread('roll_-8_10v_test2.csv',1,0);
% plot(tst_data(:,9))
% tst_data=tst_data(6000:end,:);
% plot(tst_data(:,9))
enable_data = tst_data(2:end,10);
% enable_idx = find(enable_data-1,1,'last');
enable_idx = find(enable_data,1);
pitch_data = tst_data(:,9);
roll_data = tst_data(:,8);
% disable_idx = find(abs(pitch_data)<40,1,'last');
disable_idx = find(abs(pitch_data)>40,1);
if isempty(disable_idx)
    disable_idx = length(tst_data);
end
pitch_data = pitch_data(enable_idx:disable_idx);
roll_data = roll_data(enable_idx:disable_idx);
pitchvel_data = tst_data(enable_idx:disable_idx,7);

sim_init = [0;0;tst_data(enable_idx,6)*pi/180;tst_data(enable_idx,7)*pi/180;tst_data(enable_idx,8)*pi/180;tst_data(enable_idx,9)*pi/180];


% plot(pitch_data)
constant_input = [0;-10];
% sim_init = [zeros(4,1);-7*pi/180;0*pi/180];
Ts = 0.01;
t = 0:Ts:3;
states = zeros(6,length(t));
states(:,1) = sim_init;

succ = 1;   
alpha_w = 0;
inp_v = zeros(2,length(t));
for i = 1:length(t)
    if succ ~= -1
        inp_v(:,i) = constant_input;
%         inp_v(:,i) = [-10;0];
        [states_myode,t_my_ode] = NL_update_discrete_myode45(states(:,i),inp_v(:,i),Ts,1e-3);
        states(:,i+1) = states_myode(:,end);
        if any(abs(states(5:6,i+1)) > [13*pi/180 13*pi/180])
            succ = -1;
            break;
        end
        alpha_w = alpha_w + states(1,i)*Ts;
    end
end
states = states(:,1:i+1);
t = t(1:i+1);
figure(1)
hold on
plot(t(1:end),states(5,1:end)*180/pi,'b--')
if length(t)>length(pitch_data)
    plot(t(1:length(pitch_data)),roll_data,'b-')
else 
    plot(t(1:end),roll_data(1:length(t)),'b-')
end

legend({"Simulated System",'Real System'},'interpreter','latex','location','southwest')
xlabel('Time [\textit{s}]','interpreter','latex')
ylabel('Angle [\textit{deg}]','interpreter','latex')


% figure(2)
% hold on
% plot(t+Ts,states(4,1:end)*180/pi)
% if length(t)>length(pitch_data)
%     plot(t(1:length(pitch_data)),pitchvel_data)
% else 
%     plot(t,pitchvel_data(1:length(t)))
% end
% 
% legend("simulation",'test','location','best')


% figure(2)
% hold on
% plot(t+Ts,states(2,1:end-1),'r')
% figure(3)
% plot(t(1:end-1)+Ts,inp_v(1,2:end),'*')
%generate_cpp_vector(inp_v(2,:),'inp_v')
% toc 
