clc; clear variables; close all

cov_ax = 5e-6;
cov_ay = 5e-6;
cov_az = 1e-5;

dat_acc = randn([3,1e5]).*sqrt([cov_ax;cov_ay;cov_az]) + [0;0;1];

phi = atan2(dat_acc(2,:),(sqrt(dat_acc(1,:).^2 + dat_acc(3,:).^2)));
theta = atan2(-dat_acc(3,:),dat_acc(1,:));
figure(1)
histogram(phi)
figure(2)
histogram(theta)
cov(theta);
cov(phi);


cov_gx = 5e-6;
cov_gy = 5e-6;
cov_gz = 5e-6;

c2r_wheel = 2*pi/(32*26.9);
c2r_disk = 2*pi/(24*19.203);

ctr = 0;
for i = 1:10000
    if get_motor_noise(20*2*pi/60,c2r_wheel,0.01) == 2
        ctr = ctr + 1;
    end
end
function obsv = get_motor_noise(vel,c2r,Ts)
    counts = vel*Ts/c2r;
    if rand() > abs(counts)-abs(floor(counts))
        obsv = floor(counts);
    else 
        obsv = ceil(counts);
    end
end