% compare_k_fixed.m
% Compare K from old symbolic flow vs new cached function flow at one leg length.

clear; clc;

% === User settings ===
l_l_val = 0.20;
l_r_val = 0.20;
Ts = 0.001;
Q_cost = diag([200 200 1 10 1.0 1 1.0 1 2500 1]);
R_cost = diag([5 5 0.9 0.9]);

% === Leg-dependent params (same linear model as k_calc_chuan_real2.m) ===
l_wl_val = 0.4710 * l_l_val + 0.0083;
l_bl_val = 0.5290 * l_l_val - 0.0083;
I_ll_val = 0.0524 * l_l_val + 0.006;

l_wr_val = 0.4710 * l_r_val + 0.0083;
l_br_val = 0.5290 * l_r_val - 0.0083;
I_lr_val = 0.0524 * l_r_val + 0.006;

% === New flow: use A_fun / B_fun ===
if ~exist('num_matrix.mat', 'file')
    error('num_matrix.mat not found. Run Num_function.m first.');
end
load('num_matrix.mat', 'A_fun', 'B_fun');

A_new = A_fun(l_l_val, l_r_val, l_wl_val, l_wr_val, l_bl_val, l_br_val, I_ll_val, I_lr_val);
B_new = B_fun(l_l_val, l_r_val, l_wl_val, l_wr_val, l_bl_val, l_br_val, I_ll_val, I_lr_val);
K_new = lqrd(A_new, B_new, Q_cost, R_cost, Ts);

% === Old flow: rebuild symbolic A/B (as in k_calc_chuan_real2.m) ===
syms R_w R_l l_l l_r l_wl l_wr l_bl l_br l_c m_w m_l m_b real
syms I_w I_ll I_lr I_b I_z g real
syms ddtheta_wl ddtheta_wr ddtheta_ll ddtheta_lr ddtheta_b real
syms theta_ll theta_lr theta_b real
syms T_wl T_wr T_bl T_br real

equ1 = (I_w*l_l/R_w+m_w*R_w*l_l+m_l*R_w*l_bl)*ddtheta_wl ...
    +(m_l*l_wl*l_bl-I_ll)*ddtheta_ll ...
    +(m_l*l_wl+m_b*l_l/2)*g*theta_ll ...
    +T_bl-T_wl*(1+l_l/R_w)==0;

equ2 = (I_w*l_r/R_w+m_w*R_w*l_r+m_l*R_w*l_br)*ddtheta_wr ...
    +(m_l*l_wr*l_br-I_lr)*ddtheta_lr ...
    +(m_l*l_wr+m_b*l_r/2)*g*theta_lr ...
    +T_br-T_wr*(1+l_r/R_w)==0;

equ3 = -(m_w*R_w*R_w+I_w+m_l*R_w*R_w+m_b*R_w*R_w/2)*ddtheta_wl ...
    -(m_w*R_w*R_w+I_w+m_l*R_w*R_w+m_b*R_w*R_w/2)*ddtheta_wr ...
    -(m_l*R_w*l_wl+m_b*R_w*l_l/2)*ddtheta_ll ...
    -(m_l*R_w*l_wr+m_b*R_w*l_r/2)*ddtheta_lr+T_wl+T_wr==0;

equ4 = (m_w*R_w*l_c+I_w*l_c/R_w+m_l*R_w*l_c)*ddtheta_wl ...
    +(m_w*R_w*l_c+I_w*l_c/R_w+m_l*R_w*l_c)*ddtheta_wr ...
    +m_l*l_wl*l_c*ddtheta_ll+m_l*l_wr*l_c*ddtheta_lr ...
    -I_b*ddtheta_b+m_b*g*l_c*theta_b-(T_wl+T_wr)*l_c/R_w-(T_bl+T_br)==0;

equ5 = ((I_z*R_w)/(2*R_l)+I_w*R_l/R_w)*ddtheta_wl ...
    -((I_z*R_w)/(2*R_l)+I_w*R_l/R_w)*ddtheta_wr ...
    +(I_z*l_l)/(2*R_l)*ddtheta_ll ...
    -(I_z*l_r)/(2*R_l)*ddtheta_lr-T_wl*R_l/R_w+T_wr*R_l/R_w==0;

sol = solve(equ1, equ2, equ3, equ4, equ5, ...
    ddtheta_wl, ddtheta_wr, ddtheta_ll, ddtheta_lr, ddtheta_b);

ddtheta_wl_s = sol.ddtheta_wl;
ddtheta_wr_s = sol.ddtheta_wr;
ddtheta_ll_s = sol.ddtheta_ll;
ddtheta_lr_s = sol.ddtheta_lr;
ddtheta_b_s  = sol.ddtheta_b;

J_A = jacobian([ddtheta_wl_s, ddtheta_wr_s, ddtheta_ll_s, ddtheta_lr_s, ddtheta_b_s], ...
               [theta_ll, theta_lr, theta_b]);
J_B = jacobian([ddtheta_wl_s, ddtheta_wr_s, ddtheta_ll_s, ddtheta_lr_s, ddtheta_b_s], ...
               [T_wl, T_wr, T_bl, T_br]);

A_old_sym = sym(zeros(10, 10));
B_old_sym = sym(zeros(10, 4));

for p_idx = 1:3
    p = p_idx * 2 + 3;
    A_old_sym(2,p)  = R_w*(J_A(1,p_idx) + J_A(2,p_idx))/2;
    A_old_sym(4,p)  = (R_w*(-J_A(1,p_idx) + J_A(2,p_idx)))/(2*R_l) ...
                    - (l_l*J_A(3,p_idx))/(2*R_l) + (l_r*J_A(4,p_idx))/(2*R_l);
    A_old_sym(6,p)  = J_A(3,p_idx);
    A_old_sym(8,p)  = J_A(4,p_idx);
    A_old_sym(10,p) = J_A(5,p_idx);
end

for r = 1:2:9
    A_old_sym(r,r+1) = 1;
end

for h = 1:4
    B_old_sym(2,h)  = R_w*(J_B(1,h) + J_B(2,h))/2;
    B_old_sym(4,h)  = (R_w*(-J_B(1,h) + J_B(2,h)))/(2*R_l) ...
                    - (l_l*J_B(3,h))/(2*R_l) + (l_r*J_B(4,h))/(2*R_l);
    B_old_sym(6,h)  = J_B(3,h);
    B_old_sym(8,h)  = J_B(4,h);
    B_old_sym(10,h) = J_B(5,h);
end

% numeric constants (same as k_calc_chuan_real2.m)
R_w_val = 0.06;  R_l_val = 0.2;   l_c_val = -0.05636;
m_w_val = 0.612; m_l_val = 0.788; m_b_val = 10.085;
I_w_val = 0.000894; I_b_val = 0.22548; I_z_val = 0.2864;
g_val   = 9.8;

const_syms = [R_w, R_l, l_c, m_w, m_l, m_b, I_w, I_b, I_z, g];
const_vals = [R_w_val, R_l_val, l_c_val, m_w_val, m_l_val, m_b_val, I_w_val, I_b_val, I_z_val, g_val];
leg_syms   = [l_l, l_r, l_wl, l_wr, l_bl, l_br, I_ll, I_lr];
leg_vals   = [l_l_val, l_r_val, l_wl_val, l_wr_val, l_bl_val, l_br_val, I_ll_val, I_lr_val];

A_old = double(subs(A_old_sym, [const_syms, leg_syms], [const_vals, leg_vals]));
B_old = double(subs(B_old_sym, [const_syms, leg_syms], [const_vals, leg_vals]));
K_old = lqrd(A_old, B_old, Q_cost, R_cost, Ts);

% === Compare ===
fprintf('Leg length: l_l=%.3f, l_r=%.3f\n', l_l_val, l_r_val);
fprintf('A diff (max abs): %.6g\n', max(abs(A_new(:) - A_old(:))));
fprintf('B diff (max abs): %.6g\n', max(abs(B_new(:) - B_old(:))));
fprintf('K diff (max abs): %.6g\n', max(abs(K_new(:) - K_old(:))));
