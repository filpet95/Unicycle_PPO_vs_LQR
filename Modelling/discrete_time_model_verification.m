clc; clear variables; close all

load 'linear_model.mat'

sim_init = [zeros(4,1);0.2*pi/180;0.2*pi/180];
t = 0:Ts:1;
states = zeros(6,length(t));
l_states = zeros(6,length(t));
states(:,1) = sim_init;
l_states(:,1) = sim_init;


inp_v = 0*ones(2,length(t));

for i = 1:length(t)
    [states_myode,t_my_ode] = NL_update_discrete_myode45(states(:,i),inp_v(:,i),Ts,1e-3);
    states(:,i+1) = states_myode(:,end);
    l_states(:,i+1) = disc_sys.A*l_states(:,i) + disc_sys.B*inp_v(:,i);
end

figure(1)
hold on
plot(t+Ts,states(5,2:end)*180/pi,'b-')
plot(t+Ts,states(6,2:end)*180/pi,'r-')
plot(t+Ts,l_states(5,2:end)*180/pi,'b--')
plot(t+Ts,l_states(6,2:end)*180/pi,'r--')

xlabel({'[\textit{s}]'},'interpreter','latex')
ylabel({'[\textit{deg}]'},'interpreter','latex')
xlim([0 1])

% if method == 1
%     plot([0 t(end)],[-2*0.1879 -2*0.1879],'black--')
%     plot([0 t(end)],[2*0.1879 2*0.1879],'black--','HandleVisibility','off')
% else
%     plot([0 t(end)],[-2*1.6110 -2*1.6110],'black--')
%     plot([0 t(end)],[2*1.6110 2*1.6110],'black--','HandleVisibility','off')
% end
legend({'$\varphi$, NL','$\theta$, NL','$\varphi$, LTI','$\theta$, LTI'},'interpreter','latex','location','nw')
% f1.Position = f1.Position.*[1 1 1 0.5];
