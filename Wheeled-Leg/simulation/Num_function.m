% Num_function.m
% 功能：将A、B转换成可快速调用的数值函数

clear; clc;
load('state_and_control_matrix.mat','A','B');  % 你的符号A,B

% --------- 1) 固定参数（你原脚本里的那组常数）---------
R_w_v = 0.06;  R_l_v = 0.1961;   
l_c_v = -0.05; 
% l_c_v = 0.0;
m_w_v = 0.554; m_l_v = 1.067; m_b_v = 13.329;
I_w_v = 0.000698; I_b_v = 0.13756; I_z_v = 0.21072;
g_v   = 9.8;

% --------- 2) 先把这些常量代入 A,B ---------
const_syms = {'R_w','R_l','l_c','m_w','m_l','m_b','I_w','I_b','I_z','g'};
const_vals = { R_w_v, R_l_v, l_c_v, m_w_v, m_l_v, m_b_v, I_w_v, I_b_v, I_z_v, g_v };

A_const = subs(A, const_syms, const_vals);
B_const = subs(B, const_syms, const_vals);

A_const = simplify(A_const, 'Steps', 20);
B_const = simplify(B_const, 'Steps', 20);


vars = {'l_l','l_r','l_wl','l_wr','l_bl','l_br','I_ll','I_lr'};
A_fun = matlabFunction(A_const, 'Vars', vars);
B_fun = matlabFunction(B_const, 'Vars', vars);

save('num_matrix.mat','A_fun','B_fun');
disp('Saved: num_matrix.mat');

