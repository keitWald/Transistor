

tic
clear 
clc
%% 
% 定义机器人机体参数
syms R_w                % 驱动轮半径
syms R_l                % 驱动轮轮距/2
syms l_l l_r            % 左右腿长
syms l_wl l_wr          % 驱动轮质心到左右腿部质心距离
syms l_bl l_br          % 机体质心到左右腿部质心距离
syms l_c                % 机体质心到腿部关节中心点距离
syms m_w m_l m_b        % 驱动轮质量 腿部质量 机体质量
syms I_w                % 驱动轮转动惯量           (自然坐标系法向)
syms I_ll I_lr          % 驱动轮左右腿部转动惯量    (自然坐标系法向，实际上会变化)
syms I_b                % 机体转动惯量             (自然坐标系法向)
syms I_z                % 机器人z轴转动惯量        (简化为常量)

% 定义其他独立变量并补充其导数
syms theta_wl   theta_wr   % 左右驱动轮转角
syms dtheta_wl  dtheta_wr 
syms ddtheta_wl ddtheta_wr ddtheta_ll ddtheta_lr ddtheta_b

% 定义状态向量
syms s ds phi dphi theta_ll dtheta_ll theta_lr dtheta_lr theta_b dtheta_b

% 定义控制向量
syms T_wl T_wr T_bl T_br

% 输入物理参数：重力加速度
syms g

% 输入机器人的基本参数
R_w=0.06; R_l=0.2; l_c=-0.05636; m_w=0.612; m_l=0.788; m_b=10.085;
I_w=0.000894; I_b=0.22548; I_z=0.2864; g=9.8;



%% 这部分按照上交的建模
% ======================== 新建模代码 - 开始 ========================



% 通过原文方程组(3.11)-(3.15)，求出ddtheta_wl,ddtheta_wr,ddtheta_ll,ddtheta_lr,ddtheta_b表达式
eqn1 = (I_w*l_l/R_w+m_w*R_w*l_l+m_l*R_w*l_bl)*ddtheta_wl+(m_l*l_wl*l_bl-I_ll)*ddtheta_ll+(m_l*l_wl+m_b*l_l/2)*g*theta_ll+T_bl-T_wl*(1+l_l/R_w)==0;
eqn2 = (I_w*l_r/R_w+m_w*R_w*l_r+m_l*R_w*l_br)*ddtheta_wr+(m_l*l_wr*l_br-I_lr)*ddtheta_lr+(m_l*l_wr+m_b*l_r/2)*g*theta_lr+T_br-T_wr*(1+l_r/R_w)==0;
eqn3 = -(m_w*R_w*R_w+I_w+m_l*R_w*R_w+m_b*R_w*R_w/2)*ddtheta_wl-(m_w*R_w*R_w+I_w+m_l*R_w*R_w+m_b*R_w*R_w/2)*ddtheta_wr-(m_l*R_w*l_wl+m_b*R_w*l_l/2)*ddtheta_ll-(m_l*R_w*l_wr+m_b*R_w*l_r/2)*ddtheta_lr+T_wl+T_wr==0;
eqn4 = (m_w*R_w*l_c+I_w*l_c/R_w+m_l*R_w*l_c)*ddtheta_wl+(m_w*R_w*l_c+I_w*l_c/R_w+m_l*R_w*l_c)*ddtheta_wr+m_l*l_wl*l_c*ddtheta_ll+m_l*l_wr*l_c*ddtheta_lr-I_b*ddtheta_b+m_b*g*l_c*theta_b-(T_wl+T_wr)*l_c/R_w-(T_bl+T_br)==0;
eqn5 = ((I_z*R_w)/(2*R_l)+I_w*R_l/R_w)*ddtheta_wl-((I_z*R_w)/(2*R_l)+I_w*R_l/R_w)*ddtheta_wr+(I_z*l_l)/(2*R_l)*ddtheta_ll-(I_z*l_r)/(2*R_l)*ddtheta_lr-T_wl*R_l/R_w+T_wr*R_l/R_w==0;
[ddtheta_wl,ddtheta_wr,ddtheta_ll,ddtheta_lr,ddtheta_b] = solve(eqn1,eqn2,eqn3,eqn4,eqn5,ddtheta_wl,ddtheta_wr,ddtheta_ll,ddtheta_lr,ddtheta_b);

% 通过计算雅可比矩阵的方法得出控制矩阵A，B所需要的全部偏导数
J_A = jacobian([ddtheta_wl,ddtheta_wr,ddtheta_ll,ddtheta_lr,ddtheta_b],[theta_ll,theta_lr,theta_b]);
J_B = jacobian([ddtheta_wl,ddtheta_wr,ddtheta_ll,ddtheta_lr,ddtheta_b],[T_wl,T_wr,T_bl,T_br]);

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



%% %%%%%%%%%% 1. 计算“正常行驶”工况的系数矩阵 %%%%%%%%%%
disp('正在计算“正常行驶”工况的系数...');
% Q_cost_normal = diag([150 150 150 100 20 20 20 20 400 400]);
% R_cost_normal = diag([3 3 0.25 0.25]);
% Q_cost_normal=diag([150 150 150 100 20 20 20 20 200 200]);
% R_cost_normal=diag([50 50 0.5 0.5]);
% Q_cost_normal=diag([150 100 150 100 10 10 10 10 50 50]);
% R_cost_normal=diag([100 100 1 1]);
% Q_cost_normal=diag([50 50 50 50 20 10 20 10 50 50]);
% R_cost_normal=diag([10 10 0.5 0.5]);

  % 矩阵Q中，以下列分别对应：
%        s     ds     phi     dphi     theta_ll dtheta_ll theta_lr dtheta_lr theta_b dtheta_b
  % Q_cost_normal=diag([100 100 10 1 200 1 200 1 1750 20]);
  % Q_cost_normal=diag([60 60 8 1 2 1 2 1 625 35]);
  % Q_cost_normal=diag([200 200 1 10 200 20 200 20 2000 50]);
  Q_cost_normal=diag([200 200 1 10 1.0 1 1.0 1 2500 1]);

  % 矩阵中，以下列分别对应：
%        T_wl    T_wr     T_bl     T_br
   % R_cost_normal=diag([7.5 7.5 1.875 1.875]);
   R_cost_normal=diag([5 5 0.9 0.9]);
   % R_cost_normal=diag([3.75 3.75 1.25 1.25]);
K_cons_normal = calculate_and_fit_coeffs(A, B, Q_cost_normal, R_cost_normal);
disp('“正常行驶”工况的系数计算完成。');

%% %%%%%%%%%% 2. 计算“单腿离地”工况的系数矩阵 %%%%%%%%%%
disp('正在计算“单腿离地”工况的系数...');
Q_cost_off = diag([10 10 10 10 150 100 150 100 2000 50]);
R_cost_off = diag([3 3 0.25 0.25]); 
K_cons_off_ground = calculate_and_fit_coeffs(A, B, Q_cost_off, R_cost_off);
disp('“单腿离地”工况的系数计算完成。');


%% %%%%%%%%%% 3. 将两套系数矩阵写入 .cpp 源文件 %%%%%%%%%%
disp('正在生成嵌入式源文件 lqr_coeffs2.cpp ...');
filename = 'lqr_coeffs2.cpp'; % <--- 修改点：文件名后缀改为 .cpp
fileID = fopen(filename, 'w');

% --- 写入文件头和 #include ---
fprintf(fileID, '// LQR Gain Matrix Coefficients for Embedded Systems\n');
fprintf(fileID, '// This file is auto-generated by a MATLAB script.\n');
fprintf(fileID, '// Generated on: %s\n\n', datestr(now));
fprintf(fileID, '#include "lqr_coeffs.h"\n\n'); % <--- 修改点：包含对应的头文件

% --- 写入正常行驶的系数矩阵定义 ---
write_matrix_to_file(fileID, 'k', K_cons_normal, ...
    'Definition of coefficients for NORMAL driving state');

% --- 写入单腿离地的系数矩阵定义 ---
write_matrix_to_file(fileID, 'k_off', K_cons_off_ground, ...
    'Definition of coefficients for OFF-GROUND (single leg) state');

fclose(fileID);

disp(['成功！两套拟合系数矩阵已定义并保存到文件: ', filename]);
toc

%% %%%%%%%%%% 辅助函数区域 %%%%%%%%%%

function K_cons = calculate_and_fit_coeffs(A, B, Q_cost, R_cost)
    % (此函数内部无变化)
    leg_length = 0.16:0.01:0.41;
    len = size(leg_length, 2);
    sample_size = len^2;
    
    K = zeros(sample_size, 40);
    x1 = zeros(sample_size, 1);
    x2 = zeros(sample_size, 1);
    
    p = 1; % 行计数器
    for i = 1:len
        l_varl = leg_length(i);
        l_wl_ac = 0.4710 * l_varl + 0.0083;
        l_bl_ac = 0.5290 * l_varl - 0.0083;
        I_ll_ac = 0.0524 * l_varl + 0.006;
        for j = 1:len
            l_varr = leg_length(j);
            l_wr_ac = 0.4710 * l_varr + 0.0083;
            l_br_ac = 0.5290 * l_varr - 0.0083;
            I_lr_ac = 0.0524 * l_varr + 0.006;
            
            trans_A = subs(A, {'l_l', 'l_r', 'l_wl', 'l_wr', 'l_bl', 'l_br', 'I_ll', 'I_lr'}, ...
                              {l_varl, l_varr, l_wl_ac, l_wr_ac, l_bl_ac, l_br_ac, I_ll_ac, I_lr_ac});
            trans_B = subs(B, {'l_l', 'l_r', 'l_wl', 'l_wr', 'l_bl', 'l_br', 'I_ll', 'I_lr'}, ...
                              {l_varl, l_varr, l_wl_ac, l_wr_ac, l_bl_ac, l_br_ac, I_ll_ac, I_lr_ac});
                        % 可控性（可选）：确保满秩
            if rank(ctrb(trans_A ,trans_B )) < size( trans_A ,1), warning('增广系统不可控'); end
            
            KK = lqrd(double(trans_A), double(trans_B), Q_cost, R_cost, 0.001);
            
            K(p, :) = reshape(KK', 1, 40);
            x1(p, 1) = l_varl;
            x2(p, 1) = l_varr;
            p = p + 1;
        end
    end
    
    x1sq = x1.^2;
    x2sq = x2.^2;
    x1x2 = x1.*x2;
    
    K_cons = zeros(40, 6);
    X = [x1, x2, x1sq, x2sq, x1x2];
%决定每个kij的拟合方法
for k=1:40
    mdl = fitlm(X, K(:,k));
    disp(mdl.Coefficients.Estimate');
    K_cons(k,:)=mdl.Coefficients.Estimate'; 

end
end
%% 


function write_matrix_to_file(fileID, matrix_name, data_matrix, comment)
    % (此函数内部无变化)
    fprintf(fileID, '// %s\n', comment);
    fprintf(fileID, '// Fitting polynomial: K_ij = c0 + c1*l_l + c2*l_r + c3*l_l^2 + c4*l_r^2 + c5*l_l*l_r\n');
    fprintf(fileID, 'const float %s[40][6] = {\n', matrix_name);
    for i = 1:size(data_matrix, 1)
        fprintf(fileID, '    {');
        for j = 1:size(data_matrix, 2)
            if j < size(data_matrix, 2)
                fprintf(fileID, '%.4ff, ', data_matrix(i, j));
            else
                fprintf(fileID, '%.4ff', data_matrix(i, j));
            end
        end
        if i < size(data_matrix, 1)
            fprintf(fileID, '},\n');
        else
            fprintf(fileID, '}\n');
        end
    end
    fprintf(fileID, '};\n\n');
end