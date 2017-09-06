clear all
syms q1 q2 q4 a1 a2 d3 d4 t
% q1,q2,d3,q4 variable angle
%a1, a2, d4 length of robot
assume(a1,'real');assume(a1>0);
assume(a2,'real');assume(a2>0);
assume(d3,'real');assume(d3>0);
assume(d4,'real');assume(d4>0);
assume(q1,'real');
assume(q2,'real');
assume(q2,'real');
assume(q4,'real');
%INPUT MATRIX Denavit hartenberg
A_01 = [ cos(q1) -sin(q1) 0 a1*cos(q1); sin(q1) cos(q1) 0 a1*sin(q1); 0 0  1 0;0 0 0 1]
A_12 = [ cos(q2) sin(q2) 0 a2*cos(q2); sin(q2) -cos(q2) 0 a2*sin(q2); 0 0 -1 0;0 0 0 1]
A_23 = [ 1 0 0 0; 0 1 0 0; 0 0 1 d3; 0 0 0 1]
A_34 = [ cos(q4) -sin(q4) 0 0; sin(q4) cos(q4) 0 0; 0 0  1 d4;0 0 0 1]
% MATRIX LAST Denavit hartenberg
A_04=A_01*A_12*A_23*A_34;
A_02=simplify(A_01*A_12);
A_03=simplify(A_02*A_23);
disp('Matrix Last')
A_04 = simplify(A_04)
rE = A_04(1:3,4) % Final point for activities

%Robotics System Toolbox - Peter Corke
L(1)=Link([0,0,0.2,0,1]) %P
L(2)=Link([0,0,0.4,0,0]) %R
L(3)=Link([0,0,0.4,0,0]) %R
L(4)=Link([0,0,0.2,0,0]) %R
L(1).qlim=[0 3];
L(2).qlim=[-pi 5*pi/6];
robot=SerialLink(L)
robot.plotopt = {'workspace' [-1,1,-1,1,-1,1]};
robot.plot([0.1,pi/2,pi/2,pi/2])

t=[0:0.05:4]; %time
j= length(t);
for i=1:j
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
xE(i) = 0.6*cos(t(i));
yE(i) = 0.3*sin(t(i));
zE(i)= -0.3;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
A = 2*0.5*xE(i);
B = 2*0.5*yE(i);
D = -0.4^2 + xE(i).^2 + yE(i).^2 + 0.5^2;
s1 = (D.*B-A.*sqrt(A.^2+B.^2-D.^2))./(A.^2+B.^2);
c1 = (D.*A+B.*sqrt(A.^2+B.^2-D.^2))./(A.^2+B.^2);
q1_num(:,i) = atan2(s1,c1);
s2 = (yE(i)-0.5.*sin(q1_num(:,i)));
c2 = (xE(i)-0.5.*cos(q1_num(:,i)));
q2_num(:,i) = atan2(s2,c2)- q1_num(:,i);
d3_num(:,i) = -zE(i) - 0.1;
q4_num(:,i)=q1_num(:,i)+q2_num(:,i);
end
figure
clf
hold on
grid on
axis([-1, 1, -1, 1 ,-1, 1])
for i=1:length(t)
    plot(robot,[q1_num(1,i),q2_num(1,i),d3_num(1,i),q4_num(1,i)])
    plot3(xE(i),yE(i),zE(i),'b.')
    pause(1/1000) %video
end
