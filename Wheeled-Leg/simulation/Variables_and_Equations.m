% Variables_and_Equations.m
% 功能：定义机器人系统的状态变量，同时根据方程组求解得到ddtheta_wl,ddtheta_wr,ddtheta_ll,ddtheta_lr,ddtheta_b
clear; clc;
%% 定义整个机器人系统建模的符号变量

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


%% 求解方程得到各状态量的二阶导数

% 这部分参考上交的开源，通过联立equ1-equ5得到机器人各状态量的二阶导数

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

equs = [equ1;equ2;equ3;equ4;equ5];
xdd = [ddtheta_wl,ddtheta_wr,ddtheta_ll,ddtheta_lr,ddtheta_b];

% M_sym：每个方程里各个 ddtheta_* 的系数（5×5 矩阵）
% rhs_sym：剩下不含 ddtheta_* 的所有项（包含重力项、控制力矩项、角度项等）
% equationsToMatrix 的输出满足：M_sym * xdd = rhs_sym
[M_sym, rhs_sym] = equationsToMatrix(equs, xdd);
% 直接解 M_sym * xdd = rhs_sym

%% 化简并求解得到状态量的二阶导

M2 = simplify(M_sym, 'Steps', 50);
f2 = simplify(rhs_sym, 'Steps', 50);

xdd_expr = simplify(M2 \ f2);
save('xdd.mat','xdd_expr');

disp('Saved: xdd.mat');
