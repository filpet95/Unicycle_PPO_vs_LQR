clc; clear variables; close all
%%
%---------------SETUP-------------------------------------------------
% Activate_J_Disk_Motor = false;
% Activate_J_Wheel_Motor = false;

%---------------Position vectors--------------------------------------

syms phi theta Lwb Lwd alpha_w alpha_d rw real
syms force1 force2 m_pitch m_roll

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

%position of expernal forces
p_force_pitch = p_d0;
p_force_roll = p_d0 + [0; 0.055; 0];

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

%force vector
dp_force_pitch = dq_d;
dp_force_roll = jacobian(p_force_roll,q)*dq;
%ddp_force_pitch = simplify(jacobian(dp_force_roll,q)*dq + jacobian(dp_force_roll,dq)*ddq);
%ddp_force_roll = simplify(jacobian(dp_force_pitch,q)*dq + jacobian(dp_force_pitch,dq)*ddq);
%magnitude of force 
mag_pitch = 1;
mag_roll = 1;
% force_pitch = mag_pitch * dp_force_pitch(1); %force in x direction
% force_roll = mag_roll * dp_force_roll(2); %force in y direction
%---------------Translational kinetic energy------------------------------
%Tt = 0.5*Mw*dq_w'*dq_w + 0.5*Mb*dq_b'*dq_b + 0.5*Md*dq_d'*dq_d;
T_t = 0.5*Mw*(dq_w.'*dq_w) + 0.5*Mb*(dq_b.'*dq_b) + 0.5*Md*(dq_d.'*dq_d);
%T_force = 0.5*force1*(dp_force_pitch.'*dp_force_pitch) + 0.5*force2*(dp_force_roll.'*dp_force_roll);
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
T = T_t + T_r;% + T_force;
T = simplify(T);
%---------------potential energy-------------------------------------------
M = [Mw Mb Md];

g = 9.81;
U = M*g*[p_w0(3); p_b0(3); p_d0(3)];

U_forces = m_pitch*p_force_pitch(1) + m_roll*p_force_roll(2);
U = U + U_forces
external_torque = simplify(jacobian(U_forces,q))
%----------------Lagrange's equation --------------------------------------
%Lagrange 
L = simplify(T - U);


%----------------Lagrange's dynamics---------------------------------------
dL_ddq = simplify(jacobian(T,dq).');
dL_ddq_dot = simplify(jacobian(dL_ddq,q)*dq + jacobian(dL_ddq,dq)*ddq);
dL_dq = simplify(jacobian(L,q).');


%%

%----------------External Torques------------------------------------------
% syms tau_w tau_d 
%define external forces: 

syms Km_w Ku_w Kg_w Km_d Ku_d Kg_d R_w L_w R_d L_d u_w u_d di_w di_d


tau_w = Km_w*Kg_w*(u_w-Ku_w*dalpha_w)/R_w;
tau_d = Km_d*Kg_d*(u_d-Ku_d*dalpha_d)/R_d;

tau = [tau_w;tau_d;-tau_d;tau_w];

%define dynamics = 0
dynam_Uni = dL_ddq_dot-dL_dq-tau; 


dynam = [dynam_Uni];



%solve for each state in the dynamics
%f1 = solve(dynam(1),ddalpha_w);
%f2 = solve(dynam(2),ddalpha_d);
%f3 = solve(dynam(3),ddphi);
%f4 = solve(dynam(4),ddtheta);

[A_dyn,B_dyn] = equationsToMatrix(simplify(dynam),[ddalpha_w;ddalpha_d;ddphi;ddtheta]);
dyn = linsolve(A_dyn,B_dyn);

f1 = dyn(1);
f2 = dyn(2);
f3 = dyn(3);
f4 = dyn(4);

 %%
% tau_w = Km_w*Kg_w*f5;
% tau_d = Km_d*Kg_d*f6;
% 
% tau = [tau_w;-tau_d;tau_d;-tau_w];
% 
% %define dynamics = 0
% dynam_Uni = dL_ddq_dot-dL_dq-tau;
% 
% dynam_Motor = [-u_w + R_w*f5 + Ku_w*Kg_w*dalpha_w
%                -u_d + R_d*f6 + Ku_d*Kg_d*dalpha_d];
% dynam = [dynam_Uni;dynam_Motor];
% 

%--------------------Define on state space form---------------------------
states = [dalpha_w;dalpha_d;dphi;dtheta;phi;theta];
fsym = [f1;f2;f3;f4;dphi;dtheta];
%--------------------Substitute parameters---------------------------------
params = [Lwb Lwd rw Km_w Ku_w Kg_w Km_d Ku_d Kg_d R_w R_d];

rpm2rads = 2*pi/60;
Lwb_val = 0.129214;
Lwd_val = 0.300609;
rw_val = 0.072;

Km_w_val = 1.5298374/2.5;
Ku_w_val = 12/(107*rpm2rads);
Kg_w_val = 1; %derived Ku,Km with gear (efficiency of gearbox?).
R_w_val = 12/2.5;
% L_w_val = 1e-3; % ????

%437 RPM version of same motor works better (7.5deg)
Ku_d_val = 12/(437*rpm2rads);
% Km_d_val = 2.1537733091666564/20;
Km_d_val = 0.5/20;
Kg_d_val = 1;
R_d_val = 12/20;
% L_d_val = 1e-3;



param_vals = [Lwb_val Lwd_val rw_val Km_w_val Ku_w_val Kg_w_val Km_d_val Ku_d_val Kg_d_val R_w_val R_d_val];


fsym = simplify(subs(fsym,params,param_vals));
%--------------------Create Function---------------------------------------
f = symfun(simplify(fsym),[states;u_w;u_d;m_pitch;m_roll]);
f = matlabFunction(f,'File','NL_model_forces');

