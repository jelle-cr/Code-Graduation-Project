close all
clear all

n = 100;
rho_0 = 1;
rho_12 = linspace(0, rho_0-0.5, n);
rho_13 = linspace(0, rho_0-0.5, n);
mmesh = meshgrid(rho_12, rho_13);

K_rep = 0.001;

alpha = zeros(n);
for i = 1:n
    for j = 1:n
        alpha(i,j) = 2/K_rep*(1/((1/rho_12(i)-1/rho_0)^2 + (1/rho_13(j)-1/rho_0)^2));
    end
end

surf(rho_12, rho_13, alpha');
ax = gca;
set(ax, 'FontSize', 12);
xlabel('$\rho_{12}$', 'Interpreter','latex', 'FontSize', 16);
ylabel('$\rho_{13}$', 'Interpreter','latex', 'FontSize', 16);
zlabel('$\alpha$', 'Interpreter','latex', 'FontSize', 16);
