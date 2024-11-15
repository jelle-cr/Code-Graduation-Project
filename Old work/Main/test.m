close all
clear all

% syms F_p [n 1]  % repulsive force w.r.t. pos gradient
% syms F_v [n 1]
% syms v [n 1]
% syms g      % tightening parameter
% 
% assume(F_p, 'real')
% assume(F_v, 'real')
% assume(v, 'real')
% assume(g, 'real')

% F_p = rand(n,1);
% F_v = rand(n,1);
% v = rand(n,1);
% g = rand(1,1);

% g = F_v'*F_p - F_p'*v
% % lhs = -(F_p'*v + g)/(F_v'*F_v) * F_v
% % % rhs = -F_p
% lhs = -(F_p'*v + g)*F_v
% rhs = -F_v'*F_v*F_p
% 
% lsh = F_v'*lhs
% rhs = F_v'*rhs
% 
% lhsnew = F_p'*v +g
% rhsnew = F_v'*F_p


n = 2;
syms p_i [n 1]
syms p_j [n 1]
syms p_ij
syms p_ji

assume(p_i, 'real')
assume(p_j, 'real')

p_ij = p_i - p_j;
p_ji = p_j - p_i;

(p_ij).'*(p_ij)

