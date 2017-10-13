%Generalised and commented code- A Rudola
% Search this link for detailed explaination,whenever referred to in this code :
%www.petercorke.com/RTB/r9/html/


L(1)=Link([0 0 0 pi/2 0 -pi/2]);% the links describing the length ,offsets,etc Search "Link" in the above mentioned site for details
                                %we are concerned with 3rd value,i.e. the length of the link in 'mm'.
L(2)=Link([0 0 -105 -pi/2 ]);
L(3)=Link([0 0 -105 pi/2 ]);
L(4)=Link([0 0 -105 -pi/2 0 0]);

R= SerialLink(L);%concatenates the links to create a arm.Detailed info on the above mentioned link

q0 = [21 12 -34 0];% initial values of joint angles of the arm at pickup position in degrees

R.plot(q0)%plotting initial arrangement of the arm

R.teach %initialising sliders in the GUI for arm, to manually change joint angles.Used for verifying inverse kinematics values or adjusting initial pickup position.

Td = transl([180 -80 -30])%3D position of the objective(point to be reached)  

q = R.ikine(Td, q0 ,'mask', [1 1 1 0 0 0])%using final kinematics,q gives final values of the joint angles at the objective

R.fkine(q)%forward kinematics values respective to 'q'(not of importance to us,but still good to know)

R.plot(q)%plotting the final arrangement for arm i.e the drop position