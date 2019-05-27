function [upd_states,t] = NL_update_discrete_myode45_external_forces(states,input,pitch_force,roll_force,Ts,tol)
%Update the continious time nonlinear model with variables step size
%until the next discrete input after Ts seconds.
% BRK4tilde = [35/384      0           500/1113    125/192     -2187/6784  11/84   0];
% ARK5 = [0           0           0           0           0           0       0
%     1/5         0           0           0           0           0       0
%     3/40        9/40        0           0           0           0       0
%     44/45       -56/15      32/9        0           0           0       0
%     19372/6521   -25369/2187 64448/6561  -212/729   0           0       0
%     -9017/3168  -355/33     46732/5247  49/176      -5103/18656 0       0
%     35/384      0           500/1113    125/192     -2187/6784  11/84   0];
% BRK5 = [5179/57600  0   7571/16695  393/640 -92097/339200   187/2100    1/40];

BRK4tilde = [25/216 0   1408/2565   2197/4101   -1/5    0];
BRK5 = [16/135      0           6656/12825  28561/56430 -9/50   2/55];
ARK5 = [0           0           0           0           0       0
        1/4         0           0           0           0       0
        3/32        9/32        0           0           0       0
        1932/2197   -7200/2197  7296/2197   0           0       0
        439/216     -8          3680/513    -845/4104   0       0
        -8/27       2           -3544/2565   1859/4104  -11/40 0];

odefun = @(t,y)NL_model_forces(y(1),y(2),y(3),y(4),y(5),y(6),input(1),input(2),pitch_force,roll_force);
t = 0;

my_upd_states = zeros(6,1e5);
my_t = zeros(1,1e5);
i = 1;
my_upd_states(:,1) = states;
while t < Ts
    i = i+1;
    Ts_try = Ts - t;
    xr45 = general_RK_step(ARK5,[BRK4tilde;BRK5],Ts_try,odefun,my_upd_states(:,i-1)); %first attempt
    d45 = norm(xr45(1,:)-xr45(2,:));
    while  d45 > tol
%         Ts_try = Ts_try * abs(tol/d45)^(1/3);
        Ts_try = Ts_try*(tol * Ts_try/(2*abs(d45)))^(1/4);
        %Ts_try = Ts_try*0.01;
        xr45 = general_RK_step(ARK5,[BRK4tilde;BRK5],Ts_try,odefun,my_upd_states(:,i-1));
        d45 = norm(xr45(1,:)-xr45(2,:));
    end
    t = t+Ts_try;
    my_upd_states(:,i) = xr45(2,:)';
    my_t(i) = t;
end
my_upd_states = my_upd_states(:,1:i);
my_t = my_t(1:i);
upd_states = my_upd_states;
t = my_t;
%     T_dbg = 4.186477385849301e-6;
%     [tr4,xr4] = general_RKv3(ARK4,BRK4,T_dbg,T_dbg,odefun,states);
%     [tr5,xr5] = general_RKv3(ARK5,BRK5,T_dbg,T_dbg,odefun,states);
%     xr45 = general_RK_step(ARK5,[BRK4tilde;BRK5],T_dbg,odefun,states);
%     xr4 = xr4(2,:);
%     xr5 = xr5(2,:);
%     norm(xr4-xr5)
%     norm(xr45(1,:)-xr45(2,:))
%     [t,upd_states] = ode45(odefun,[0 Ts],states);
%     upd_states = upd_states';
%     norm(upd_states(:,end)-my_upd_states(:,end))

end