function [states,inputs] = simulation_LQR(initial_state,K,time_vec,Ts)
% - returns simulation vectors states, input and the corresponding time vector t
% given initial state, LQR gain and time vector.

% inital_state = 6x1 vector
% K = 2x6 vector
% time vec = 1xn vector

states = zeros(6,length(time_vec)+1);
inputs = zeros(2,length(time_vec));
states(:,1) = initial_state;

for i = 1:length(time_vec)
    inputs(:,i) = saturate(-K*states(:,i),12);
    [c_states,~] = NL_update_discrete_myode45(states(:,i),inputs(:,i),Ts,1e-3);
    states(:,i+1) = c_states(:,end);   
end
states = states(:,1:end-1);

end

function sat_value = saturate(inp,sat_val)
    sat_value = zeros(size(inp));
    for i = 1:length(inp)
        sat_value(i) = sign(inp(i))*min(abs(inp(i)),sat_val);
    end
end