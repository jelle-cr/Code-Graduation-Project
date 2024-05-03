clear all
close all

n = 101;
n2 = 11;

xi_ij = linspace(-1, 1, n);
v_ij = linspace(-1, 1, n2);
mmesh = meshgrid(xi_ij,v_ij);
% xi_ij = [-20;0];
% v_ij = [0;0];
% n=1
% xi_ij= -10;
% v_ij= 0;

m = 0.01;                % Mass
d = 0.1;                 % Damping coefficient
r_a = 0.05;              % Radius of agent

% CBF parameters for safety filter
l0 = 600;           
l1 = 50;
D = l1^2-4*l0   
roots = -l1 + sqrt(D)   % Check if roots are negative

for i = 1:n
    h(i) = xi_ij(i).'*xi_ij(i) - (2*r_a)^2;
    for j = 1:n2
        a_ij(i,j) = 2*v_ij(j).'*v_ij(j) + 2*(l1-d/m)*xi_ij(i).'*v_ij(j) + l0*h(i);
        % b_ij(i) = 2*1/m*xi_ij(i).';
    end
end

index_X = find(xi_ij == -100)
index_Y = find(v_ij == 0)
Z_desired = a_ij(index_X, index_Y)

surf(xi_ij, v_ij, a_ij');
xlabel('$\xi_{ij}$', 'Interpreter','latex', 'FontSize', 16);
ylabel('$v_{ij}$', 'Interpreter','latex', 'FontSize', 16);
zlabel('$a_{ij}$', 'Interpreter','latex', 'FontSize', 16);

