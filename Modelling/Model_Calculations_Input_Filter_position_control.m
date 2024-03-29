clc; clear variables; close all
%%
%---------------Position vectors--------------------------------------

syms phi theta Lwb Lwd alpha_w alpha_d rw real


R_roll = [1 0 0;0 cos(phi) sin(phi);0 -sin(phi) cos(phi)]; %World -> Body
R_pitch = [cos(theta) 0 -sin(theta);0 1 0;sin(theta) 0 cos(theta)]; %world -> Body

R_rp = R_roll.'*R_pitch.';

%position vector of wheel center
p_w0 = [-rw*alpha_w;-rw*sin(phi);rw*cos(phi)];

%Transition matrix from wheel frame to 0 frame
A_0w = [R_rp p_w0];

%position of body center in 0 frame
p_b0 = simplify(A_0w*[0;0;Lwb;1]);

%position of disk center in 0 frame
p_d0 = simplify(A_0w*[0;0;Lwd;1]);

%--------------Velocity vectors------------------------------------------
%general coordinates
q = [alpha_w;alpha_d;phi;theta];

%angles and derivatives
syms alpha_w alpha_d phi theta dalpha_w dalpha_d dphi dtheta ddalpha_w ddalpha_d ddphi ddtheta 
[I,m] = get_parameters_v5();
Mw = m.m_wheel;
Mb = m.m_body;
Md = m.m_disk;
%velocity vector
dq = [dalpha_w; dalpha_d; dphi; dtheta];
ddq = [ddalpha_w; ddalpha_d; ddphi; ddtheta];

%velocity vectors
dq_w = jacobian(p_w0,q)*dq;
dq_b = jacobian(p_b0,q)*dq;
dq_d = jacobian(p_d0,q)*dq;

%---------------Translational kinetic energy------------------------------
T_t = 0.5*Mw*(dq_w.'*dq_w) + 0.5*Mb*(dq_b.'*dq_b) + 0.5*Md*(dq_d.'*dq_d);

%---------------Rotational Kinetic energy --------------------------------
%Wheel frame rotational kinetic energy
omega_w = R_roll*[dphi; 0; 0] + [0; -dalpha_w; 0];
I_wheel = I.I_wheel;
T_w = 0.5*omega_w.'*I_wheel*omega_w;

%body frame rotational kinetic energy
omega_b = R_pitch*R_roll*[dphi; 0; 0] + R_pitch*[0; dtheta; 0];
I_body = I.I_body;
T_b = 0.5*omega_b.'*I_body*omega_b;

%Disk frame rotational kinetic energy
omega_d = omega_b + [dalpha_d; 0; 0];
I_disk = I.I_disk;
T_d = 0.5*omega_d.'*I_disk*omega_d;

T_r = T_w + T_b + T_d;
T = T_t + T_r;
T = simplify(T);
%---------------potential energy-------------------------------------------
M = [Mw Mb Md];
g = 9.81;
U = M*g*[p_w0(3); p_b0(3); p_d0(3)];

%----------------Lagrange's equation --------------------------------------
%Lagrange 
L = simplify(T - U);

%----------------Lagrange's dynamics---------------------------------------
dL_ddq = simplify(jacobian(T,dq).');
dL_ddq_dot = simplify(jacobian(dL_ddq,q)*dq + jacobian(dL_ddq,dq)*ddq);
dL_dq = simplify(jacobian(L,q).');

%----------------External Torques------------------------------------------
syms Km_w Ku_w Kg_w Km_d Ku_d Kg_d R_w L_w R_d L_d u_w u_d di_w di_d

tau_w = Km_w*Kg_w*(u_w-Ku_w*dalpha_w)/R_w;
tau_d = Km_d*Kg_d*(u_d-Ku_d*dalpha_d)/R_d;

%define external forces: 
tau = [tau_w;tau_d;-tau_d;tau_w];

%----------------Dynamics -------------------------------------------------
dynam_Uni = dL_ddq_dot-dL_dq-tau;

dynam = [dynam_Uni];

%--------------------Define on state space form----------------------------
[A_dyn,B_dyn] = equationsToMatrix(simplify(dynam),[ddalpha_w;ddalpha_d;ddphi;ddtheta]);
dyn = linsolve(A_dyn,B_dyn);

f1 = dyn(1);
f2 = dyn(2);
f3 = dyn(3);
f4 = dyn(4);

states = [dalpha_w;dalpha_d;dphi;dtheta;phi;theta];
fsym = [f1;f2;f3;f4;dphi;dtheta];
%--------------------Substitute parameters---------------------------------
params = [Lwb Lwd rw Km_w Ku_w Kg_w Km_d Ku_d Kg_d R_w R_d];

%Mechanical parameters
Lwb_val = 0.129214;
Lwd_val = 0.300609;
rw_val = 0.072;

%Motor parameters
rpm2rads = 2*pi/60;
Km_w_val = 1.5298374/2.5;
Ku_w_val = 12/(107*rpm2rads);
Kg_w_val = 1; %derived Ku,Km with gear 
R_w_val = 12/2.5;

Ku_d_val = 12/(437*rpm2rads);
% Km_d_val = 2.1537733091666564/20;
Km_d_val = 0.5/20;
Kg_d_val = 1;
R_d_val = 12/20;

param_vals = [Lwb_val Lwd_val rw_val Km_w_val Ku_w_val Kg_w_val Km_d_val Ku_d_val Kg_d_val R_w_val R_d_val];

fsym = simplify(subs(fsym,params,param_vals));

%--------------------Create Function---------------------------------------
f = symfun(simplify(fsym),[states;u_w;u_d]);
f = matlabFunction(f,'File','NL_model_V3');

%% Linearization
A = double(subs(jacobian(fsym,states),[states;u_w;u_d].',zeros(1,size(states,1)+2)));
B = double(subs(jacobian(fsym,[u_w;u_d]),[states;u_w;u_d].',zeros(1,size(states,1)+2)));
C = [1 zeros(1,5);0 1 zeros(1,4);zeros(1,4) 1 0; zeros(1,5) 1];
D = zeros(size(C,1),size(B,2));
% for i = 1:size(A,1)
%     for j = 1:size(A,2)
%         if abs(A(i,j)) < 0.5 
%             A(i,j) = 0;
%         end
%     end
% end
% for i = 1:size(B,1)
%     for j = 1:size(B,2)
%         if abs(B(i,j)) < 1 
%             B(i,j) = 0;
%         end
%     end
% end
cont_sys = ss(A,B,C,D);
Ts = 1e-2;
disc_sys = c2d(cont_sys,Ts);
Ad = disc_sys.A;
Bd = disc_sys.B;
Cd = disc_sys.C;

input_gamma_w = 1/10
% input_gamma_w = 1/10
% input_gamma_w = 1-input_gamma_w;
% input_gamma_d = 0.9;
Atilde = [Ad Bd(:,1);zeros(1,6) 1-input_gamma_w];
Adtilde = [Atilde zeros(7,1);Ts zeros(1,6) 1];
% Atilde = [Ad Bd;zeros(1,6) 1-input_gamma_w 1-input_gamma_d];
Btilde = [zeros(6,1) Bd(:,2);input_gamma_w 0];
Bdtilde = [Btilde;zeros(1,2)];
disc_sys = ss(Adtilde,Bdtilde,eye(8),zeros(8,2),Ts);
%check if controllable or not:
if rank(ctrb(cont_sys)) == size(cont_sys.A,1)
    fprintf("Controllable!\n")
else
    fprintf("Not controllable!\n")
end
[stable_part,not_stable_part] = stabsep(cont_sys);
if rank(ctrb(not_stable_part)) < size(not_stable_part.A,1)
    fprintf("not stabilizable\n")
else
    fprintf("stabilizable!\n")
end

if rank(obsv(cont_sys)) == size(cont_sys.A,1)
    fprintf("Observable!\n")
else
    fprintf("Not observable!\n")
end
[stable_part,not_stable_part] = stabsep(cont_sys);
if rank(obsv(not_stable_part)) < size(not_stable_part.A,1)
    fprintf("not detectable\n")
else
    fprintf("detectable!\n")
end

%% Controller Design

% %---------------- K SIMULATION --------------------------------------------
% Q = diag([1e3 1e3 1e-3 1e-3 1e4 1e4 1e-3]);
% R = 12*diag([1e-4 1e-4]);



%-----------------K HARDWARE OLD ----------------------------------------------
% Q = diag([1e-3 1e-3 200 1 1e2 10 1e-3]);
% R = 12*diag([1e-2 1e-2]);
%-----------------K HARDWARE ----------------------------------------------
% Q = diag([1e-3 1e-3 200 1e-1 1e2 1 1e-5]);
% R = 12*diag([1e-2 1e-2]);
% ----------------K HARDWARE ----------------------------------------------
% Q = diag([1e-3 1e-3 1e-1 1e-1 1 1 eps]);
% R = 12*diag([1e-2 1e-2]);
Q = diag([1e-3 1e-3 1e-1 1e-1 1 1 eps 1e-2]);
R = 12*diag([1e-2 1e-2]);

%K = lqr(cont_sys,Q,R);
K_d = lqr(disc_sys,Q,R)
% save('K_d','K_d')
