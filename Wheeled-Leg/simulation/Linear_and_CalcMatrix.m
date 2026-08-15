% Linear_and_CalcMatrix.m
% 功能：对系统在平衡点处进行线性化并且求解得到A、B矩阵
clear; clc;

load('xdd.mat', 'xdd_expr')

% Ensure required symbolic variables exist after clear.
syms R_w R_l l_l l_r real
syms theta_ll theta_lr theta_b real
syms T_wl T_wr T_bl T_br real

ddtheta_wl = xdd_expr(1);
ddtheta_wr = xdd_expr(2);
ddtheta_ll = xdd_expr(3);
ddtheta_lr = xdd_expr(4);
ddtheta_b  = xdd_expr(5);

% 通过计算雅可比矩阵的方法得出控制矩阵A，B所需要的全部偏导数
J_A = jacobian([ddtheta_wl,ddtheta_wr,ddtheta_ll,ddtheta_lr,ddtheta_b], ...
               [theta_ll,theta_lr,theta_b]);
J_B = jacobian([ddtheta_wl,ddtheta_wr,ddtheta_ll,ddtheta_lr,ddtheta_b], ...
               [T_wl,T_wr,T_bl,T_br]);

% 定义矩阵A，B，将指定位置的数值根据上述偏导数计算出来并填入
A = sym(zeros(10, 10));
B = sym(zeros(10, 4));

% 填入A的非零、非对角线元素
% A(2,:), A(4,:), A(6,:), A(8,:), A(10,:) 是状态的导数行
for p_idx = 1:3 % 对应于 [theta_ll, theta_lr, theta_b]
    p = p_idx * 2 + 3; % 状态向量中的索引 5, 7, 9
                                  
    % A(2,p) = d(ds)/d(theta_x)
    A(2,p) = R_w*(J_A(1,p_idx) + J_A(2,p_idx))/2;

    % A(4,p) = d(dphi)/d(theta_x)
    A(4,p) = (R_w*(- J_A(1,p_idx) + J_A(2,p_idx)))/(2*R_l) - (l_l*J_A(3,p_idx))/(2*R_l) + (l_r*J_A(4,p_idx))/(2*R_l);

    % A(6,p), A(8,p), A(10,p)
    A(6, p)  = J_A(3, p_idx); % d(dtheta_ll)/d(theta_x)
    A(8, p)  = J_A(4, p_idx); % d(dtheta_lr)/d(theta_x)
    A(10, p) = J_A(5, p_idx); % d(dtheta_b)/d(theta_x)
end

% 填入A的对角线元素 (x_dot = v)
for r = 1:2:9
    A(r,r+1) = 1;
end

% 填入B的偶数行
for h = 1:4
    B(2,h) = R_w*(J_B(1,h) + J_B(2,h))/2;
    B(4,h) = (R_w*(- J_B(1,h) + J_B(2,h)))/(2*R_l) - (l_l*J_B(3,h))/(2*R_l) + (l_r*J_B(4,h))/(2*R_l);
    B(6,h)  = J_B(3,h);
    B(8,h)  = J_B(4,h);
    B(10,h) = J_B(5,h);
end

save('state_and_control_matrix.mat','A','B');
disp('Save: state_and_control_matrix.mat');
