clear
clc

%Link Lengths, [cm]
r1 = 53.69;
r2 = 21.25;
r3 = 51.84;
r4 = 23.25;

%Initial and Final Angles
theta2_i = -106;
theta2_f = -145;

%Rotational Velocity and Acceleration of Steering Link
omega_2 = -1; %[rad/s]
alpha_2 = 0; %[rad/s/s]
cx = [];
cy = [];
jx = [];
jy = [];
dx = [];
dy = [];
ax = [];
ay = [];

omega_3 = [];
omega_4 = [];
alpha_3 = [];
alpha_4 = [];

i = 1;
for theta2 = theta2_i:-1:theta2_f   
    theta2_list(i) = theta2;
    sx = -r1 - r2 * cosd(theta2);
    sy = -r2 * sind(theta2);
    
   cos_gamma = (r4^2 + r3^2 - r2^2 - r1^2 - 2*r1*r2*cosd(theta2)) / (2*r3*r4);
   
   theta3_1(i) = 2*atand((-sy + sqrt(r4^2 * (1 - cos_gamma^2))) / (-r3 + r4*cos_gamma - sx));
   theta3_2(i) = 2*atand((-sy - sqrt(r4^2 * (1 - cos_gamma^2))) / (-r3 + r4*cos_gamma - sx)); 
   
   theta4_1(i) = 2*atand((-sy + sqrt(r3^2 * (1-cos_gamma^2))) / (r4 - r3*cos_gamma - sx));
   theta4_2(i) = 2*atand((-sy - sqrt(r3^2 * (1-cos_gamma^2))) / (r4 - r3*cos_gamma - sx));
   
   omega_3(end+1) = omega_2 * (r2*sind(theta2 - theta4_2(i))) / (r3*sind(theta4_2(i) - theta3_2(i)));
   omega_4(end+1) = omega_2 * (r2*sind(theta2 - theta3_2(i))) / (r4*sind(theta4_2(i) - theta3_2(i))); 
   
   alpha_3(end+1) = (omega_2^2*r2*cosd(theta2 - theta4_2(i)) - alpha_2*r2*sind(theta2 - theta4_2(i)) - (omega_3(i))^2*r3*cosd(theta3_2(i) - theta4_2(i)) - (omega_4(i))^2*r4 )...
       / (r3*sind(theta3_2(i) - theta4_2(i)));
   alpha_4(end+1) = (omega_2^2*r2*cosd(theta2 - theta3_2(i)) + alpha_2*r2*sind(theta2 - theta3_2(i)) + (omega_3(i))^2*r3 - (omega_4(i))^2*r4*cosd(theta4_2(i) - theta3_2(i)))...
       / (r4*sind(theta4_2(i) - theta3_2(i)));
   
   cx(end+1) = 0;
   cy(end+1) = 0;
   
   jx(end+1) = r2 * cosd(theta2);
   jy(end+1) = r2 * sind(theta2);
   
   ax(end+1) = -53.69 + r4 * cosd(theta4_2(i));
   ay(end+1) = r4 * sind(theta4_2(i));
   
   dx(end+1) = -53.69;
   dy(end+1) = 0;
   
   i = i + 1;
end
%Array of time values for speed and accel, cacluated from omega = 1 rad/s =
%1.469 seconds/degree for 39 degrees of rotation
time_arr = 0:1.469:39*1.469;  

subplot(3,2,[1,2])
hold on
for ii = 1:1:length(cx)
    quiver(cx(ii),cy(ii), jx(ii)-cx(ii),jy(ii)-cy(ii), 'b', 'ShowArrowHead', 'off', 'AutoScale', 0)
    quiver(cx(ii),cy(ii), dx(ii)-cx(ii),dy(ii)-cy(ii), 'k', 'ShowArrowHead', 'off', 'AutoScale', 0)
    quiver(jx(ii),jy(ii), ax(ii)-jx(ii),ay(ii)-jy(ii), 'g', 'ShowArrowHead', 'off', 'AutoScale', 0)
    quiver(dx(ii),dy(ii), ax(ii)-dx(ii),ay(ii)-dy(ii), 'r', 'ShowArrowHead', 'off', 'AutoScale', 0)
end
hold off
title('Linkage Positions')
axis equal

subplot(3,2,3)
plot(time_arr, omega_3)
title('Angular Velocity of Tie Rod')
xlabel('Time [s]')
ylabel('Omega 3 [rad/s]')

subplot(3,2,4)
plot(time_arr, omega_4)
title('Angular Velocity of Knuckle Arm')
xlabel('Time [s]')
ylabel('Omega 4 [rad/s]')

subplot(3,2,5)
plot(time_arr, alpha_3)
title('Angular Acceleration of Tie Rod')
xlabel('Time [s]')
ylabel('Alpha 3 [rad/s/s]')

subplot(3,2,6)
plot(time_arr, alpha_4)
title('Angular Acceleration of Knuckle Arm')
xlabel('Time [s]')
ylabel('Alpha 4 [rad/s/s]')



