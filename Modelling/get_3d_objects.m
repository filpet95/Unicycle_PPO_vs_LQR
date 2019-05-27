function [wheel,body,disk,succ,p_d] = get_3d_objects(roll,pitch,alpha_w,wheel_in,body_in,disk_in)
succ = 1;
rw = 0.072;
Lwb = 0.129214;
Lwd = 0.300609;

Rx = [1 0 0;0 cos(roll) sin(roll);0 -sin(roll) cos(roll)];
Ry = [cos(pitch) 0 -sin(pitch);0 1 0;sin(pitch) 0 cos(pitch)];
p_w = [-rw*alpha_w;-rw*sin(roll);rw*cos(roll)];
p_d = [Lwd*sin(pitch) - alpha_w*rw;-sin(roll)*(rw + Lwd*cos(pitch));cos(roll)*(rw + Lwd*cos(pitch))];
p_b = [Lwb*sin(pitch) - alpha_w*rw;-sin(roll)*(rw + Lwb*cos(pitch));cos(roll)*(rw + Lwb*cos(pitch))];

w1 = wheel_in.w1;
w2 = wheel_in.w2;
d1 = disk_in.d1;
d2 = disk_in.d2;

if p_d(3) < 0
    succ = -1;
end

%rotate wheel around x
w1 = Rx.'*w1;
w2 = Rx.'*w2;

xw = [w1(1,:);w2(1,:)];
yw = [w1(2,:);w2(2,:)];
zw = [w1(3,:);w2(3,:)];

%rotate disk around x,y
d1 = Rx.'*Ry.'*d1;
d2 = Rx.'*Ry.'*d2;

xd = [d1(1,:);d2(1,:)];
yd = [d1(2,:);d2(2,:)];
zd = [d1(3,:);d2(3,:)];

%rotate body around x,y
pb = body_in;
pb = Rx.'*Ry.'*pb;

wheel.x = xw+p_w(1);
wheel.y = yw+p_w(2);
wheel.z = zw+p_w(3);

disk.x = xd+p_d(1);
disk.y = yd+p_d(2);
disk.z = zd+p_d(3);

body.x = pb(1,:)+p_b(1);
body.y = pb(2,:)+p_b(2);
body.z = pb(3,:)+p_b(3);

end