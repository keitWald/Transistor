%% 四个关节电机输出力矩与虚拟腿支持力的关系
% 机构：左右各一套偏置五连杆，每条腿由前、后两个关节电机驱动。
% 对照对象为 balance_robot_real.slx 的四个关节力矩输入，单位 N*m。
% 使用该模型 LVMC/RVMC 的 VMC_calc 尺寸、运动学分支和雅可比公式，
% 并计入 Gain=-1 及 Saturation4/5/6/7 的 +/-20 N*m 限幅。
% 当前 chassis 固件的尺寸为 0.1134/0.135/0.135/0.1134/0.210 m，
% 电机发送符号为左正右负、MIT 量程 +/-25 N*m，与此 SLX 不同。
% 本文件不混用固件参数，也不把关节输出轴力矩当作电机转子力矩。
%
% 广义虚功关系（单腿）：
%   [tau_front; tau_back] = J' * [F_leg; T_phi]
% 其中 F_leg [N] 为沿虚拟腿伸长方向的轴向力，T_phi [N*m] 为绕髋部的
% 虚拟摆动力矩。本题只研究支持力，因此默认 T_phi = 0。
%
% 模型中 LVMC/RVMC 的 T1/T2 输出均经过 Gain=-1 后送往关节：
%   [tau_LF; tau_LB; tau_RF; tau_RB]
% 限幅前需求 = [-k_front*F_left;
%    -k_back *F_left;
%    -k_front*F_right;
%    -k_back *F_right ]
%
% 运行后：
%   result             - 力矩需求、模型限幅后输出及可实现支持力
%   K_motor            - 限幅前支持力到四关节力矩的 4x2 线性映射
%   tau_motor_required - 四关节限幅前理论需求，4xN [N*m]
%   tau_motor          - 模型限幅后的四关节理论输出，4xN [N*m]
% 顺序为 [左phi1; 左phi4; 右phi1; 右phi4]，对应模型 L_Tp1/4、R_Tp1/4。
% 正常静态工况，不含 Subsystem 中额外的 Step 扰动力矩或动态控制器输出。

clear; clc;

%% 用户可修改参数
leg_length = (0.16:0.01:0.40).';  % 虚拟腿长 [m]
phi0 = pi/2;                     % 虚拟腿角度 [rad]；pi/2 表示竖直向下

F_support_total = 400;           % 两条腿合计支持力 [N]（左右各承担 100 N）
left_load_ratio = 0.5;           % 左腿承担比例，右腿为 1-left_load_ratio
T_phi_left = 0;                  % 左腿虚拟摆动力矩 [N*m]
T_phi_right = 0;                 % 右腿虚拟摆动力矩 [N*m]
motor_torque_limit = 40;         % SLX 四关节 Saturation4/5/6/7 [N*m]

% 是否把结果另存为 CSV；默认关闭，避免每次运行都写文件。
save_csv = true;
csv_name = 'joint_torque_support_force_results.csv';

%% 与 balance_robot_real.slx 中 LVMC/RVMC 的 VMC_calc 一致 [m]
l1 = 0.096;
l2 = 0.114;
l3 = 0.114;
l4 = 0.096;
l5 = 0.216;

%% 支持力分配
assert(left_load_ratio >= 0 && left_load_ratio <= 1, ...
    'left_load_ratio 必须在 0 到 1 之间。');
assert(all(isfinite([F_support_total,left_load_ratio,phi0,T_phi_left,T_phi_right])) ...
    && F_support_total >= 0 && all(isfinite(leg_length) & leg_length > 0) ...
    && isfinite(motor_torque_limit) && motor_torque_limit > 0, ...
    '支持力、姿态、腿长及力矩限幅参数无效。');
F_left = F_support_total * left_load_ratio;
F_right = F_support_total * (1-left_load_ratio);

n = numel(leg_length);
phi1 = nan(n,1); phi2 = nan(n,1); phi3 = nan(n,1); phi4 = nan(n,1);
k_front = nan(n,1); k_back = nan(n,1);
kT_front = nan(n,1); kT_back = nan(n,1);
tau_LF = nan(n,1); tau_LB = nan(n,1);
tau_RF = nan(n,1); tau_RB = nan(n,1);
K_motor = nan(4,2,n);

for i = 1:n
    L = leg_length(i);

    %% 逆运动学：由 (L, phi0) 求两个主动关节角
    xc_ik = (l4/l5)*L*cos(phi0);
    yc_ik = (l4/l5)*L*sin(phi0);

    A = (xc_ik+l1)^2 + yc_ik^2 - l2^2;
    B = -4*l1*yc_ik;
    C = (xc_ik-l1)^2 + yc_ik^2 - l2^2;
    D = (xc_ik+l4)^2 + yc_ik^2 - l3^2;
    E = -4*l4*yc_ik;
    G = (xc_ik-l4)^2 + yc_ik^2 - l3^2;
    disc1 = B^2-4*A*C;
    disc2 = E^2-4*D*G;

    if disc1 < -1e-12 || disc2 < -1e-12 || abs(A) < 1e-12 || abs(D) < 1e-12
        warning('腿长 %.3f m、phi0 %.3f rad 时逆运动学无解。', L, phi0);
        continue;
    end
    disc1 = max(disc1,0);
    disc2 = max(disc2,0);
    u = (-B-sqrt(disc1))/(2*A);
    v = (-E+sqrt(disc2))/(2*D);
    phi1(i) = wrapToPiLocal(2*atan2(u,1));
    phi4(i) = wrapTo2PiLocal(2*atan2(v,1));

    %% 正运动学中间角：分支与模型 VMC_calc 相同
    xb = l1*cos(phi1(i)); yb = l1*sin(phi1(i));
    xd = l4*cos(phi4(i)); yd = l4*sin(phi4(i));
    dx = xd-xb; dy = yd-yb;
    A0 = 2*l2*dx;
    B0 = 2*l2*dy;
    C0 = l2^2 + dx^2 + dy^2 - l3^2;
    disc0 = max(A0^2+B0^2-C0^2,0);
    phi2(i) = 2*atan2(B0-sqrt(disc0), A0+C0) + 2*pi;

    xc = (l5/l4)*(xb+l2*cos(phi2(i)));
    yc = (l5/l4)*(yb+l2*sin(phi2(i)));
    L_fk = hypot(xc,yc);
    phi0_fk = wrapTo2PiLocal(atan2(yc,xc));
    phi3(i) = atan2(yb+l2*sin(phi2(i))-yd, xb+l2*cos(phi2(i))-xd);

    if abs(L_fk-L) > 1e-8 || abs(wrapToPiLocal(phi0_fk-phi0)) > 1e-8
        error('腿长 %.3f m 处正逆运动学不一致。', L);
    end

    %% 单腿 VMC 雅可比：tau = [k_F, k_T] * [F_leg; T_phi]
    s23 = sin(phi2(i)-phi3(i));
    if abs(s23) < 1e-8
        warning('腿长 %.3f m 接近机构奇异位形，跳过该点。', L);
        continue;
    end
    k_front(i) = -(l1*l5/l4)*sin(phi0-phi3(i))*sin(phi1(i)-phi2(i))/s23;
    k_back(i)  = -l5*sin(phi0-phi2(i))*sin(phi3(i)-phi4(i))/s23;
    kT_front(i) = -(l1*l5/l4)*cos(phi0-phi3(i))*sin(phi1(i)-phi2(i))/(L*s23);
    kT_back(i)  = -l5*cos(phi0-phi2(i))*sin(phi3(i)-phi4(i))/(L*s23);

    % 同一文件内的虚功自检：雅可比必须等于腿长/腿角对主动角的导数。
    h = 1e-6;
    J_numeric = nan(2);
    q = [phi1(i);phi4(i)];
    for joint = 1:2
        qp = q; qm = q; qp(joint) = qp(joint)+h; qm(joint) = qm(joint)-h;
        [Lp,ap] = legFKLocal(qp,l1,l2,l3,l4,l5);
        [Lm,am] = legFKLocal(qm,l1,l2,l3,l4,l5);
        J_numeric(:,joint) = [(Lp-Lm)/(2*h);wrapToPiLocal(ap-am)/(2*h)];
    end
    J_torque = [k_front(i),kT_front(i);k_back(i),kT_back(i)];
    assert(norm(J_torque-J_numeric.', 'fro') < 1e-5, ...
        '腿长 %.3f m 处雅可比虚功校验失败。', L);

    % 模型中左右腿 VMC 输出均经过 Gain=-1 后送往关节。
    K_motor(:,:,i) = [-k_front(i), 0;
                       -k_back(i), 0;
                        0, -k_front(i);
                        0, -k_back(i) ];

    tau_LF(i) = -(k_front(i)*F_left + kT_front(i)*T_phi_left);
    tau_LB(i) = -(k_back(i) *F_left + kT_back(i) *T_phi_left);
    tau_RF(i) = -(k_front(i)*F_right + kT_front(i)*T_phi_right);
    tau_RB(i) = -(k_back(i) *F_right + kT_back(i)*T_phi_right);
end
tau_motor_required = [tau_LF.'; tau_LB.'; tau_RF.'; tau_RB.'];
tau_motor = clampTorqueLocal(tau_motor_required,motor_torque_limit);
torque_saturated = abs(tau_motor_required) > motor_torque_limit;
support_feasible = all(isfinite(tau_motor_required),1).' & ...
                   ~any(torque_saturated,1).';
[tau_LF,tau_LB,tau_RF,tau_RB] = deal(tau_motor(1,:).',tau_motor(2,:).', ...
                                   tau_motor(3,:).',tau_motor(4,:).');
F_left_achieved = nan(n,1); F_right_achieved = nan(n,1);
T_phi_left_achieved = nan(n,1); T_phi_right_achieved = nan(n,1);
for i = 1:n
    J = [k_front(i),kT_front(i);k_back(i),kT_back(i)];
    if any(~isfinite(J(:))) || rcond(J) < 1e-12
        continue;
    end
    % 撤销模型的 -1 增益，完整反解限幅后的轴向力及伴生摆动力矩。
    load_left = J \ (-tau_motor(1:2,i));
    load_right = J \ (-tau_motor(3:4,i));
    F_left_achieved(i) = load_left(1); T_phi_left_achieved(i) = load_left(2);
    F_right_achieved(i) = load_right(1); T_phi_right_achieved(i) = load_right(2);
    if support_feasible(i)
        assert(norm([load_left;load_right]-[F_left;T_phi_left;F_right;T_phi_right]) ...
            < 1e-7*max(1,norm([F_left;T_phi_left;F_right;T_phi_right])), ...
            '力矩到支持力的反解校验失败。');
    end
end
assert(isequal(clampTorqueLocal([25,-25,0],20),[20,-20,0]), ...
       '模型力矩限幅校验失败。');

%% 结果表
result = table(leg_length, phi1, phi4, k_front, k_back, ...
    tau_LF, tau_LB, tau_RF, tau_RB, ...
    tau_motor_required(1,:).',tau_motor_required(2,:).', ...
    tau_motor_required(3,:).',tau_motor_required(4,:).', ...
    support_feasible,F_left_achieved,F_right_achieved, ...
    T_phi_left_achieved,T_phi_right_achieved, ...
    'VariableNames', {'leg_length_m','phi1_rad','phi4_rad', ...
    'front_torque_per_N_m','back_torque_per_N_m', ...
    'tau_LF_Nm','tau_LB_Nm','tau_RF_Nm','tau_RB_Nm', ...
    'tau_LF_required_Nm','tau_LB_required_Nm', ...
    'tau_RF_required_Nm','tau_RB_required_Nm', ...
    'support_feasible','F_left_achieved_N','F_right_achieved_N', ...
    'T_phi_left_achieved_Nm','T_phi_right_achieved_Nm'});

fprintf('\n总支持力 %.3f N：左腿 %.3f N，右腿 %.3f N；phi0 = %.4f rad\n', ...
    F_support_total,F_left,F_right,phi0);
fprintf('系数单位 N*m/N（数值上等于 m）；模型中左右腿力矩均经过 -1 增益。\n\n');
fprintf('tau_*_Nm 为模型 +/-%.1f N*m 限幅后输出；*_required_Nm 为未限幅需求。\n', ...
    motor_torque_limit);
fprintf('支持力可实现点数 %d/%d；超限时不能把原设定支持力当作实际支持力。\n\n', ...
    nnz(support_feasible),n);
disp(result);

if save_csv
    writetable(result,csv_name);
    fprintf('结果已写入 %s\n',csv_name);
end

%% 关系说明
% 纯支持力、无摆动力矩且没有限幅时：
%   tau_LF = -k_front(L,phi0) * F_left
%   tau_LB = -k_back (L,phi0) * F_left
%   tau_RF = -k_front(L,phi0) * F_right
%   tau_RB = -k_back (L,phi0) * F_right
%
% 反算单腿轴向支持力（实测力矩含噪声时采用最小二乘）：
%   F_left_est  = [k_front k_back]*[-tau_LF;-tau_LB] / ...
%                 (k_front^2+k_back^2)
%   F_right_est = [k_front k_back]*[-tau_RF;-tau_RB] / ...
%                 (k_front^2+k_back^2)
%
% 注意：以上是理想静力映射，不含连杆自重、摩擦、减速器效率和惯性力。
% 若“支持力”指地面对车轮的竖直法向力且虚拟腿不竖直，应另外做投影；
% 静止且 T_phi=0 时，竖直分量为 F_vertical = F_leg*sin(phi0)。
% 限幅后可能产生非零 T_phi，应使用完整反解结果；上述最小二乘公式
% 仅用于已知纯轴向力的工况，不用于判断超限后的完整受力。

function tau = clampTorqueLocal(tau,limit)
tau = min(max(tau,-limit),limit);
end

function [L,a] = legFKLocal(q,l1,l2,l3,l4,l5)
xb = l1*cos(q(1)); yb = l1*sin(q(1));
xd = l4*cos(q(2)); yd = l4*sin(q(2));
dx = xd-xb; dy = yd-yb;
A = 2*l2*dx; B = 2*l2*dy; C = l2^2+dx^2+dy^2-l3^2;
disc = A^2+B^2-C^2;
assert(disc >= -1e-12,'雅可比自检的扰动姿态不可达。');
p2 = 2*atan2(B-sqrt(max(disc,0)),A+C);
x = (l5/l4)*(xb+l2*cos(p2)); y = (l5/l4)*(yb+l2*sin(p2));
L = hypot(x,y); a = atan2(y,x);
end

function a = wrapToPiLocal(a)
% 映射到 [-pi,pi)
a = mod(a+pi,2*pi)-pi;
end

function a = wrapTo2PiLocal(a)
% 映射到 [0,2*pi)
a = mod(a,2*pi);
end
