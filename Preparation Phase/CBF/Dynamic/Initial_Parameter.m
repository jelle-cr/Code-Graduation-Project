function [Init_Par] = Initial_Parameter()  
    Init_Par.m = 0.01;          % mass
    Init_Par.d = 0.1;           % damping coefficient
    Init_Par.p_d = [-10 -10];    % desired position

    Init_Par.p_sc=1e5;
    Init_Par.c_convergence_rate=6;%2*0.1/0.01;%4;
    Init_Par.gamma=6;
    % Init_Par.k = 100;

    Init_Par.L1 = 1;            % Weights for CLF
    Init_Par.L2 = 1;

    Init_Par.k = [60; 10];    % Weights for CBF

    Init_Par.ob = [-2 -2.5;
                   -6 -4;
                   -7 -8];
    Init_Par.r = 1.5;
end

