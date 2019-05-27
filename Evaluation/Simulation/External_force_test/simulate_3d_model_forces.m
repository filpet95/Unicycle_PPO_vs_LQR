clc; clear variables;

%load 'linear_model.mat'
Rx = @(phi) [1 0 0;0 cos(phi) -sin(phi);0 sin(phi) cos(phi)];
Ry = @(theta)[cos(theta) 0 sin(theta);0 1 0;-sin(theta) 0 cos(theta)];
%Ideal K_d for a model wothout the forces
K_d = [   -1.8363   -0.0003   -0.0549    9.7754   -0.1385   52.8178;
           0.0003   -0.4236  -29.4253   -0.0051  -89.4770   -0.0203];
rw = 0.072;
Lwb = 0.139554;
Lwd = 0.3;
rd = 0.2;

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

sim_init = [zeros(4,1);0*pi/180;0*pi/180];
% sim_init = [zeros(4,1);30*pi/180;30*pi/180;zeros(2,1)];
Ts = 0.01;
% t = linspace(0,10,1/Ts);
t = 0:Ts:4;
states1 = zeros(6,length(t));
states1(:,1) = sim_init;
states2 = zeros(6,length(t));
states2(:,1) = sim_init;
succ1 = 1;
succ2 = 1;
alpha_w1 = 0;
alpha_w2 = 0;

vw = [0 0];
sc = 1.5;
% inp = [-1;1];
enable = 1;
inp_v = zeros(4,length(t));

for i = 1:length(t)
    i
    if succ2 ~= -1
        %inp_v(:,i) = -[K_d(1:2,:); ones(2,6)]*states2(:,i);
        %inp_v(:,i) = inp_v(:,i)*enable;
        if i > 100 && i < 110
            %inp_v(3:4, i)= [10; 10];
            %inp_v(:,i) = -[K_d(1:2,:); ones(2,6)]*states2(:,i);
            inp_v(:,i) = -[K_d; zeros(2,6)]*states2(:,i);
            %inp_v(3:4,i) = inp_v(3:4,i) + [100; 100];
            inp_v(3:4,i) = inp_v(3:4,i) + [10; 0];
            inp_v(:,i) = inp_v(:,i)*enable;
        else
            %inp_v(3:4,i) = [0;0];
            inp_v(:,i) = -[K_d(1:2,:); zeros(2,6)]*states2(:,i);
            inp_v(:,i) = inp_v(:,i)*enable;
            %inp_v(3:4,i) = [0,0];
        end
        
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
        if abs(inp_v(1,i)) < 3
            inp_v(1,i) = 0;
        end
        if abs(inp_v(2,i))< 3
            inp_v(2,i) = 0;
        end

        %         states2(:,i+1) = states2(:,i) + Ts*NL_model_V3(states2(1,i),states2(2,i),states2(3,i),states2(4,i),states2(5,i),states2(6,i),states2(7,i),states2(8,i),inp_v(1,i),inp_v(2,i));
        %states2(:,i+1) = (disc_sys.A*states2(:,i)+disc_sys.B*inp_v(:,i));
        %if i <100 || i >170
        %    states2(:,i+1) = NL_update_discrete_forces(states2(:,i),[inp_v(1:2,i);0;0],Ts);
        %else 
        %    fprintf('halló')
        %    states2(:,i+1) = NL_update_discrete_forces(states2(:,i),[inp_v(1:2,i);1;1],Ts);
        %end
        states2(:,i+1) = NL_update_discrete_forces(states2(:,i),inp_v(:,i),Ts);
        alpha_w2 = alpha_w2 + states2(1,i)*Ts;
        %         succ2 = animate_V1(states2(5,i),states2(6,i),alpha_w2,vw,2);
        [w,b,d,succ2,p_d] = get_3d_objects(states2(5,i),states2(6,i),alpha_w2,wheel,body,disk);
        figure(2)
        clf
        hold on
        surf(w.x,w.y,w.z)
        patch(b.x,b.y,b.z,'r')
        surf(d.x,d.y,d.z)
        patch(1*[1 -1 -1 1], 1*[1 1 -1 -1], [0 0 0 0], 1*[1 1 -1 -1])
        plot3([0 1],[0 0],[0 0],'r')
        plot3([0 0],[0 1],[0 0],'g')
        plot3([0 0],[0 0],[0 1],'b')
        A_d = [Rx(states2(5,i))*Ry(states2(6,i)) p_d];
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
        zlabel(180*states2(5:6,i)/pi)
        if t(i) > 2.15e9
            pause
        end
    end
    %     if succ2 ~= -1
    %         states2(:,i+1) = states2(:,i) + Ts*NL_model_V2(states2(1,i),states2(2,i),states2(3,i),states2(4,i),states2(5,i),states2(6,i),inp(1),inp(2));
    %         alpha_w2 = alpha_w2 + states2(1,i)*Ts;
    %         %         succ2 = animate_V1(states2(5,i),states2(6,i),alpha_w2,vw,2);
    %         [w,b,d,succ2,p_d] = get_3d_objects(states2(5,i),states2(6,i),alpha_w2,wheel,body,disk);
    %         figure(2)
    %         clf
    %         hold on
    %         surf(w.x,w.y,w.z)
    %         patch(b.x,b.y,b.z,'r')
    %         surf(d.x,d.y,d.z)
    %         patch(1*[1 -1 -1 1], 1*[1 1 -1 -1], [0 0 0 0], 1*[1 1 -1 -1])
    %         plot3([0 1],[0 0],[0 0],'r')
    %         plot3([0 0],[0 1],[0 0],'g')
    %         plot3([0 0],[0 0],[0 1],'b')
    %         A_d = [Rx(states2(5,i))*Ry(states2(6,i)) p_d];
    %         ex = A_d*[1;0;0;1];
    %         ey = A_d*[0;1;0;1];
    %         ez = A_d*[0;0;1;1];
    %         plot3([p_d(1) ex(1)],[p_d(2) ex(2)],[p_d(3) ex(3)],'r')
    %         plot3([p_d(1) ey(1)],[p_d(2) ey(2)],[p_d(3) ey(3)],'g')
    %         plot3([p_d(1) ez(1)],[p_d(2) ez(2)],[p_d(3) ez(3)],'b')
    %         axis(sc*[-1 1 -1 1 -.4 1])
    %         view(vw(1),vw(2))
    %         xlabel(t(i))
    %         if t(i) > 1.15
    %             pause
    %         end
    %     end
    pause(0.01)
end
