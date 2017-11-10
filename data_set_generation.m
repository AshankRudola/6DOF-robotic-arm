%data set Generation (dsg)
l1 = 0.084;
l2 = 0.0765;
l3 = 0.082;

Theta1 = -pi/2:0.2:pi/2;
Theta2 = -pi/2:0.2:pi/2;
Theta3 = -pi/2:0.2:pi/2;

data1 = []; data2 = []; data3 = [];

for i=1:1:length(Theta1)
    for j=1:1:length(Theta2)
        for k=1:1:length(Theta3)
            X = l1*cos(Theta1(i))+l2*cos(Theta1(i)+Theta2(j))+l3*cos(Theta1(i)+Theta2(j)+Theta3(k));
            Y = l1*sin(Theta1(i))+l2*sin(Theta1(i)+Theta2(j))+l3*sin(Theta1(i)+Theta2(j)+Theta3(k));
            scatter(X,Y)
            hold on
            data1 = [data1; X Y Theta1(i)];
            data2 = [data2; X Y Theta2(j)];
            data3 = [data3; X Y Theta3(k)];
        end
    end
end