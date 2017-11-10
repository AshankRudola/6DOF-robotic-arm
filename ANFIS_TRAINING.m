%ANFIS TRAINING MATLAB CODE%

fprintf('-->%s\n','ANFIS for R2 is training')
anfis1 = anfis(data1, 7, 150, [0,0,0,0]);

fprintf('-->%s\n','ANFIS for R3 is training')
anfis2 = anfis(data2, 6, 150, [0,0,0,0]);

fprintf('-->%s\n','ANFIS for R4 is training')
anfis3 = anfis(data3, 6, 150, [0,0,0,0]);

fprintf('-->%s\n','Training Completed')

%%end%%