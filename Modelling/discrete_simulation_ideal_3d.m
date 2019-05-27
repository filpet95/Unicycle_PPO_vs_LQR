clc; clear variables; close all
% addpath('C:\Users\filpe\Dropbox\Chalmers_stuff\SSYX04 Master Thesis\Unicycle_Project\Modelling')
load 'linear_model.mat'

% body->world rotations:
Rx = @(phi) [1 0 0;0 cos(phi) -sin(phi);0 sin(phi) cos(phi)];
Ry = @(theta)[cos(theta) 0 sin(theta);0 1 0;-sin(theta) 0 cos(theta)];

% constants for 3d simulations:
rw = 0.072;
Lwb = 0.129214;
Lwd = 0.300609;
rd = 0.2;

% Create a unit cylinder object: 
[x_u,y_u,z_u] = cylinder(1); % Unit cylinder with centre in origo.
z_u = z_u-0.5;

%create wheel:
xw = rw*x_u;
yw = rw*y_u;
zw = 29e-3*z_u;
w1 = Rx(pi/2)*[xw(1,:);yw(1,:);zw(1,:)];
w2 = Rx(pi/2)*[xw(2,:);yw(2,:);zw(2,:)];
wheel.w1 = w1;
wheel.w2 = w2;

%create disk:
xd = rd*x_u;
yd = rd*y_u;
zd = 1.5e-3*z_u;
d1 = Ry(pi/2)*[xd(1,:);yd(1,:);zd(1,:)];
d2 = Ry(pi/2)*[xd(2,:);yd(2,:);zd(2,:)];
disk.d1 = d1;
disk.d2 = d2;

%create Body
xb = [0 0 0 0];
yb = [-0.5 0.5 0.5 -0.5];
zb = [0.5 0.5 -0.5 -0.5];
yb = 0.153*yb;
zb = 0.35*zb;
body = [xb;yb;zb];

% Simulation
sim_init = [zeros(4,1);180*pi/180;0*pi/180];
vw = [45 5]; %View
sc = 1; %Scale
t = 0:Ts:2;  % Simulation time

% predef
states = zeros(6,length(t));
states(:,1) = sim_init;
succ = 1;   
alpha_w = 0;
enable = 1; %enable control
inp_v = zeros(2,length(t));

for i = 1:length(t)
    if succ ~= -1 %Stop simulation if uni doesnt stabilize
        inp_v(:,i) = -K_d*states(:,i);
        inp_v(:,i) = inp_v(:,i)*enable;
%         Saturate
        if inp_v(1,i) > 12
            inp_v(1,i) = 12;
        elseif inp_v(1,i) < -12
            inp_v(1,i) = -12;
        end
        if inp_v(2,i) > 12
            inp_v(2,i) = 12;
        elseif inp_v(2,i) < -12
            inp_v(2,i) = -12;
        end
%         Motors does not spin under a certain voltage:
        if abs(inp_v(1,i)) < 1
            inp_v(1,i) = 0;
        end
        if abs(inp_v(2,i))< 1
            inp_v(2,i) = 0;
        end
%         Take a step
        [states_myode,t_my_ode] = NL_update_discrete_myode45(states(:,i),inp_v(:,i),Ts,1e-3);
        states(:,i+1) = states_myode(:,end);
        
        alpha_w = alpha_w + states(1,i)*Ts;
%         Plot result
        [w,b,d,succ1,p_d] = get_3d_objects(states(5,i),states(6,i),alpha_w,wheel,body,disk);
        figure(1)
        clf
        hold on
        surf(w.x,w.y,w.z)
        patch(b.x,b.y,b.z,'r')
        surf(d.x,d.y,d.z)
        patch(1*[1 -1 -1 1], 1*[1 1 -1 -1], [0 0 0 0], 1*[1 1 -1 -1])
        plot3([0 1],[0 0],[0 0],'r')
        plot3([0 0],[0 1],[0 0],'g')
        plot3([0 0],[0 0],[0 1],'b')
        A_d = [Rx(states(5,i))*Ry(states(6,i)) p_d];
        ex = A_d*[1;0;0;1];
        ey = A_d*[0;1;0;1];
        ez = A_d*[0;0;1;1];
        plot3([p_d(1) ex(1)],[p_d(2) ex(2)],[p_d(3) ex(3)],'r')
        plot3([p_d(1) ey(1)],[p_d(2) ey(2)],[p_d(3) ey(3)],'g')
        plot3([p_d(1) ez(1)],[p_d(2) ez(2)],[p_d(3) ez(3)],'b')
        axis(sc*[-1 1 -1 1 -.4 1])
        view(vw(1),vw(2))
        xlabel(t(i))
        ylabel(inp_v(:,i))
        %zlabel(180*states2(5:6,i)/pi)
        if t(i) > 2.15e9
            pause
        end
    end
end
figure(2)
plot(t+Ts,states(5:6,2:end)*180/pi,'r')
figure(3)
plot(t+Ts,states(2,1:end-1),'r')
figure(4)
plot(t(1:end-1)+Ts,inp_v(2,2:end),'*')
%generate_cpp_vector(inp_v(2,:),'inp_v')
