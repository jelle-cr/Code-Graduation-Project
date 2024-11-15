function dpdt = odefcn(t,p)
    global u_save
    
    dpdt = zeros(4,1);
    % Note that [p(1) p(2) p(3) p(4)]==[x y x' y']
    % time
    t
    par = Initial_Parameter();

    V = 1/2*(p(1)-par.p_d(1))^2 + 1/2*(p(2)-par.p_d(2))^2 + 1/2*(p(3)+par.L1*(p(1)-par.p_d(1)))^2 + 1/2*(p(4)+par.L2*(p(2)-par.p_d(2)))^2
    gradV = [p(1) - par.p_d(1) + 2*par.L1*(p(3)+par.L1*(p(1)-par.p_d(1)));
              p(2) - par.p_d(2) + 2*par.L2*(p(4)+par.L2*(p(2)-par.p_d(2)));
              2*(p(3)+par.L1*(p(1)-par.p_d(1)));
              2*(p(4)+par.L2*(p(2)-par.p_d(2)))];
    h = zeros(height(par.ob),1);
    for i = 1:height(par.ob)
        h(i) = (p(1)-par.ob(i,1))^2+(p(2)-par.ob(i,2))^2 - par.r^2;
    end
    h
    
    f = [p(3); p(4); -par.d/par.m*p(3); -par.d/par.m*p(4)];
    g = [0 0; 0 0; 1/par.m 0; 0 1/par.m];

    %% Formulate and Solve the QPs
    %  Use function quadprog
    Lf_clf= dot(gradV, f);
    Lg_clf= [dot(gradV, g(:,1)), dot(gradV, g(:,2))];
    A_clf = [Lg_clf, -1];
    b_clf = -Lf_clf-par.c_convergence_rate*V;
    
    A_cbf = zeros(height(par.ob),3);
    b_cbf = zeros(height(par.ob),1);
    for i = 1:height(par.ob)
        Lfh = 2*(p(3)*(p(1)-par.ob(i,1)) + p(4)*(p(2)-par.ob(i,2)));
        Lf2h = 2*p(3)^2 + 2*p(4)^2 - 2*par.d/par.m*(p(3)*(p(1)-par.ob(i,1)) + p(4)*(p(2)-par.ob(i,2)));
        LgLfh = 2/par.m * [p(1)-par.ob(i,1), p(2)-par.ob(i,2), 0]; 
        H_collect = [h(i); Lfh];
        
        A_cbf(i,:) = -LgLfh;
        b_cbf(i,1) = par.k'*H_collect + Lf2h;
    end

    A = [A_clf; A_cbf];
    b = [b_clf; b_cbf];

    % Input Constraints -1 <= u <= 1
    A = [A; eye(2), zeros(2, width(A)-2); -eye(2), zeros(2,width(A)-2)];
    b = [b; 1*ones(4,1)];

    H = [eye(2), zeros(2,1); zeros(1,2), par.p_sc];
    % H = eye(2);
    F = [0; 0; 0];
    [u]=quadprog(H,F,A,b);
    u_save=[u_save,u];
    
    %% Dynamics of the system
    dpdt(1)=p(3);                           
    dpdt(2)=p(4);
    dpdt(3)=-par.d/par.m*p(3) + 1/par.m * u(1);
    dpdt(4)=-par.d/par.m*p(4) + 1/par.m * u(2);
end