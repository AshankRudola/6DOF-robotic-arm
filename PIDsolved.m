close all
clear all
clc
%% Initilization
th_int=[-pi/2 pi/2 pi/3]; %initial positions
ths=[pi/2 -pi/2 pi/4]; %set-points
x0=[0 0 th_int 0 0 0 0]; %states initial values %%%%%%%%
Ts=[0 20]; %time span
%% Robot Specifications
L1=1; %link 1
L2=1; %link 2
L3=1; %Link 3
M1=1; %mass 1
M2=1; %mass 2
M3=1; %mass 3
spec=[L1 L2 L3 M1 M2 M3];
%% PID Parameters
% PID parameters for theta 1
Kp1=15;
Kd1=7;
Ki1=10;
% PID parameters for theta 2
Kp2=15;
Kd2=10;
Ki2=10;
% PID parameters for theta 3
Kp3=15;
Kd3=10;
Ki3=10;
Kpid=[Kp1 Kd1 Ki1 Kp2 Kd2 Ki2 Kp3 Kd3 Ki3];
%% ODE solving
% opt1=odeset('RelTol',1e-10,'AbsTol',1e-20,'NormControl','off');
[T,X] = ode45(@(t,x) r2dof(t,x,ths,spec,Kpid),Ts,x0);
%% Output
th1=X(:,4); %theta1 wavwform
th2=X(:,5); %theta2 wavwform
th3=X(:,6); %theta2 wavwform
%torque inputs computation from the 7th,8th states inside ODE
F1=diff(X(:,7))./diff(T); %%%%
F2=diff(X(:,8))./diff(T); %%%%
F3=diff(X(:,8))./diff(T); %%%%
tt=0:(T(end)/(length(F1)-1)):T(end);
%xy
x1=L1.*sin(th1); % X1
y1=L1.*cos(th1); % Y1
x2=L1.*sin(th1)+L2.*sin(th1+th2)+L3.*sin(th1+th2+th3); % X2
y2=L1.*cos(th1)+L2.*cos(th1+th2)+L3.*cos(th1+th2+th3); % Y2
%theta1 error plot
plot(T,ths(1)-th1)
grid
title('Theta-1 error')
ylabel('theta1 error (rad)')
xlabel('time (sec)')
%theta2 error plot
figure
plot(T,ths(2)-th2)
grid
title('Theta-2 error')
ylabel('theta2 error (rad)')
xlabel('time (sec)')
%theta3 error plot
figure
plot(T,ths(3)-th3)
grid
title('Theta-3 error')
ylabel('theta3 error (rad)')
xlabel('time (sec)')
% %torque1 plot
% figure
% plot(tt,F1)
% grid
% title('Torque of theta 1')
% ylabel('theta1 torque')
% xlabel('time (sec)')
% %torque2 plot
% figure
% plot(tt,F2)
% grid
% title('Torque of theta 2')
% ylabel('theta2 torque')
% xlabel('time (sec)')