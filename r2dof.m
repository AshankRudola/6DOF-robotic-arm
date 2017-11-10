function xdot=r2dof(t,x,ths,spec,Kpid)
xdot=zeros(8,1);
%% set-points
th1s=ths(1);
th2s=ths(2);
th3s=ths(3);
%% Robot Specifications
M1=spec(4);
M2=spec(5);
M3=spec(6);
L1=spec(1);
L2=spec(2);
L3=spec(3);
g=9.8;
%% Inertia Matrix %%%%%%%%%
b11=M1*L1^2;
b12=0;
b21=0;
b22=M2*L2^2+M3*L2^2+M3*L3^2+M3*L2*L3*cos(x(4));
b31=0;
b13=0;
b23=M3*L3^2+M3*L2^2+M3*L2*L3*cos(x(4));
b33=M3*L3^2;
b32=M3*L3^2+M3*L2^2+M3*L2*L3*cos(x(4));
Bq=[b11 b12 b13;b21 b22 b23;b31 b32 b33];
%% C Matrix %%%%%%%%%
c1=-M2*L1*L2*sin(x(4))*(2*x(5)*x(6)+x(6)^2);
c2=-M2*L1*L2*sin(x(4))*x(5)*x(6);
c3=M3*L2*L3*sin(x(4));
Cq=[c1;c2;c3];

%% Gravity Matrix %%%%%%%%%
g1=-(M1+M2)*g*L1*sin(x(3))-M2*g*L2*sin(x(3)+x(4));
g2=-M2*g*L2*sin(x(3)+x(4));
g3= M3*g*L3*cos(x(5));
Gq=[g1;g2;g3];
%% PID Control
% PID parameters for theta 1
Kp1=Kpid(1);
Kd1=Kpid(2);
Ki1=Kpid(3);
% PID parameters for theta 2
Kp2=Kpid(4);
Kd2=Kpid(5);
Ki2=Kpid(6);
% PID parameters for theta 3
Kp3=Kpid(7);
Kd3=Kpid(8);
Ki3=Kpid(9);
%decoupled control input  
f1=Kp1*(th1s-x(7))-Kd1*x(4)+Ki1*(x(1));
f2=Kp2*(th2s-x(8))-Kd2*x(5)+Ki2*(x(2));
f3=Kp3*(th3s-x(9))-Kd3*x(6)+Ki3*(x(3)); 
Fhat=[f1;f2;f3]; 
F=Bq*Fhat; % actual input to the system
%% System states %%%%%%%%%%
xdot(1)=(th1s-x(4)); %dummy state of theta1 integration
xdot(2)=(th2s-x(5)); %dummy state of theta2 integration
xdot(3)=(th2s-x(6)); %dummy state of theta2 integration

xdot(4)=x(5); %theta1-dot
xdot(5)=x(6); %theta2-dot
q2dot=inv(Bq)*(-Cq-Gq+F);
xdot(6)=q2dot(1); %theta1-2dot
xdot(7)=q2dot(2); %theta1-2dot
%control input function output to outside computer program
xdot(8)=F(1);
xdot(9)=F(2);