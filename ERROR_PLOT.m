%%Matlab Code for Plotting of Error%%

XY = [X(:) Y(:)];

Theta1P = evalfis(XY, anfis1);
Theta2P = evalfis(XY, anfis2);
Theta3P = evalfis(XY, anfis3);

aa = Theta1D(:) - Theta1P; %Theta1Diff
bb = Theta2D(:) - Theta2P; %Theta2Diff
cc = Theta2D(:) - Theta3P; %Theta3Diff

subplot(3,1,1);
plot(aa);
title('Error in THETA1','fontsize',10)

subplot(3,1,2);
plot(bb);
title('Error in THETA2','fontsize',10)

subplot(3,1,3);
plot(cc);
title('Error in THETA3','fontsize',10)

%%end%%