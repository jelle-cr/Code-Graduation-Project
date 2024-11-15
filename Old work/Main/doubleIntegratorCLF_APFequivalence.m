close all
clear all
clc

n = 2;
k_p = 3;
k_v = 3; 
k_c = 1;
if k_p*k_v <= k_c^2
    warning('W matrix is not positive definite');
end

W = [k_p*eye(n/2), k_c*eye(n/2); 
     k_c*eye(n/2), k_v*eye(n/2)];

p_i = rand(2,1); v_i = rand(2,1);
p_d = rand(2,1); v_d = rand(2,1);
p_id = p_i - p_d;
v_id = v_i - v_d;

F_att1 = -(k_p+k_c)*p_id;
F_att2 = -(k_v+k_c)*v_id;

sigma = 1/k_c*(k_p+k_c)*norm(k_c*p_id + k_v*v_id)^2 - (k_p*p_id + k_c*v_id)'*v_i;
F_clf1 = -k_c*((k_p*p_id + k_c*v_id)'*v_i + sigma)/(norm(k_c*p_id + k_v*v_id)^2)*p_id;
F_clf2 = -k_v*((k_p*p_id + k_c*v_id)'*v_i + sigma)/(norm(k_c*p_id + k_v*v_id)^2)*v_id;

F_att1
F_clf1

F_att2
F_clf2
