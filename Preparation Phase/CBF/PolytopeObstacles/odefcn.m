function dpdt = odefcn(t,p)
    global u_save
    
    dpdt = zeros(2,1);
    % time
    t
    par = Initial_Parameter(t);
    load('polytopeLines.mat'); 

    V = 1/2*(p(1)-par.p_d(1))^2 + 1/2*(p(2)-par.p_d(2))^2
    % h = zeros(height(par.ob),1);
    % for i = 1:height(par.ob)
    %     h(i) = (p(1)-par.ob(i,1))^2+(p(2)-par.ob(i,2))^2 - par.r^2;
    % end
    % 
    h = zeros(width(polyLines),1);
    gradh = zeros(2,width(polyLines));
    for i = 1:width(polyLines)
        lineBarriers = zeros(height(polyLines{i}),1);
        for j = 1:height(polyLines{i})
            lineBarriers(j) = polyLines{i}(j,1)*p(1) + polyLines{i}(j,2)*p(2) + polyLines{i}(j,3);
        end
        [max_value, max_index] = max(lineBarriers);
        h(i) = max_value;
        gradh(:,i) = [polyLines{i}(max_index,1); polyLines{i}(max_index,2)];
    end
    h
    gradh;

    f = [0; 0];
    g = [1/par.m, 0; 0, 1/par.m];
    
    %% Formulate and Solve the QPs
    %  Use function quadprog
    Lf_clf= 0;
    Lg_clf= 1/par.m * [(p(1)-par.p_d(1)), (p(2)-par.p_d(2))];
    A_clf = [Lg_clf, -1];
    b_clf = -Lf_clf-par.c_convergence_rate*V;
    
    Lf_cbf = gradh'*f;
    Lg_cbf = gradh'*g;
    % for i = 1:height(par.ob)
    %     Lg_cbf(i,:) = 1/par.m * [2*(p(1)-par.ob(i,1)), 2*(p(2)-par.ob(i,2))];
    % end
    A_cbf = [-Lg_cbf, zeros(height(Lg_cbf),1)];
    b_cbf = Lf_cbf+par.gamma*h;
    
    A = [A_clf; A_cbf];
    b = [b_clf; b_cbf];
    
    % Input Constraints -1 <= u <= 1
    A = [A; eye(2), zeros(2, width(A)-2); -eye(2), zeros(2,width(A)-2)];
    b = [b; ones(4,1)];
    
    H = [eye(2), zeros(2,1); zeros(1,2), par.p_sc];
    F = [0; 0; 0];
    [u]=quadprog(H,F,A,b);
    u_save=[u_save,u];
    
    %% Dynamics of the system
    dpdt(1)=1/par.m*u(1);
    dpdt(2)=1/par.m*u(2);
end