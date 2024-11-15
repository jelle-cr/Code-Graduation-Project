function [Init_Par] = Initial_Parameter(t)  
    Init_Par.m = 0.01;          % mass
    Init_Par.d = 0.1;           % damping coefficient
    Init_Par.p_d = [10 10];     % desired position

    Init_Par.p_sc=1e5;
    Init_Par.c_convergence_rate=2;%2*0.1/0.01;%4;
    Init_Par.gamma=20;
    % Init_Par.k = 100;

    % Init_Par.ob = [-2 -2.5;
    %                -6 -4;
    %                -7 -8];
    Init_Par.r = 2;

    Init_Par.ob = {[1,1.5; 3.5,1.5; 3.5,4; 1,4],...
                   [4,6; 7,7; 6,2],...
                   [8,8; 10,9; 8.5,10]};
    % Init_Par.ob = {[0.5,0.5; 3,1; 1,2; -1,2]};
    % Init_Par.ob = {[0,0; 1,0; -1,-2]};
end

