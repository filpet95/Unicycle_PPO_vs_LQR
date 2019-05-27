function [Inertias,masses] = get_parameters_model_error()
%GET PARAMETERS
% % % % % WHEEL, HUB, SHAFT, ADAPTER
Ixx = 3.882E+05;
Ixy = -1.808E-07;
Ixz = -3.483E-10;
Iyx = -1.808E-07;
Iyy = 6.889E+05;
Iyz = 0.00;
Izx = -3.483E-10;
Izy = 0.00;
Izz = 3.882E+05;
I_wheel = [Ixx Ixy Ixz;Iyx Iyy Iyz;Izx Izy Izz];
% I_wheel = [Ixx 0 0;0 Iyy 0;0 0 Izz];
M_wheel = 304.656;

Inertias.I_wheel = I_wheel*1e-9;
masses.m_wheel = M_wheel*1e-3;


% % % % DISK, DISKHUB
Ixx = 3.948E+06;
Ixy = -3.502;
Ixz = -77.208;
Iyx = -3.502;
Iyy = 1.980E+06;
Iyz = -436.686;
Izx = -77.208;
Izy = -436.686;
Izz = 1.969E+06;
M_disk = 204.514;
I_disk = [Ixx Ixy Ixz;Iyx Iyy Iyz;Izx Izy Izz];
% I_disk = [Ixx 0 0;0 Iyy 0;0 0 Izz];

Inertias.I_disk = I_disk*1e-9;
masses.m_disk = M_disk*1e-3;

% % % % BODY, disk motor, bearing, battery, wheel-motor, box, weight,
% electronic box, electronics
Ixx = 2.674E+07;
Ixy = -4.495E+04;
Ixz = 1.178E+06;
Iyx = -4.495E+04;
Iyy = 2.078E+07;
Iyz = 5.171E+06;
Izx = 1.178E+06;
Izy = 5.171E+06;
Izz = 7.229E+06;
%
% Ixx = 2.890E+07;
% Ixy = 2.519E+04;
% Ixz = 1.186E+06;
% Iyx = 2.519E+04;
% Iyy = 2.102E+07;
% Iyz = 4.958E+06;
% Izx = 1.186E+06;
% Izy = 4.958E+06;
% Izz = 9.176E+06;

I_body = [Ixx Ixy Ixz;Iyx Iyy Iyz;Izx Izy Izz];
% I_body = [Ixx 0 0;0 Iyy 0;0 0 Izz];
M_body = 1596.134;
Inertias.I_body = I_body*1e-9;
masses.m_body = M_body*1e-3;

end