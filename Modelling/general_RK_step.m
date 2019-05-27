function xRK = general_RK_step (A,B,dt,f,init)
%general RK method for f dependent only on x. f(t,x) x can be vector.
%init is nx1 initial state. 
xRK= zeros(size(B,1),length(init));
stage_max = size(A,2);
K = zeros(length(init),stage_max);

for j = 1:stage_max
    K(:,j) = f(0,init' + dt*A(j,:)*K');
end
for i = 1:size(B,1)
    xRK(i,:) = init' + dt*B(i,:)*K';
end

end