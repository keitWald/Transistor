function [phi1, phi4] = leg_ik(l0, phi0)
% 腿部偏置五连杆的运动学逆解
% 输入:
%   l0   - 虚拟腿长度
%   phi0 - 虚拟腿角度
%   l1, l2, l3, l4, l5 - 连杆长度参数
% 输出:
%   phi1 - 关节1角度
%   phi4 - 关节4角度

l1 = 113.4;
l2 = 135;
l3 = 135;
l4 = 113.4;
l5 = 210;

% 计算C点坐标
xc = l4 / l5 * l0 * cos(phi0);
yc = l4 / l5 * l0 * sin(phi0);

% 求解phi1
A_IK = (xc + l1)^2 + yc^2 - l2^2;
B_IK = -4 * l1 * yc;
C_IK = (xc - l1)^2 + yc^2 - l2^2;
u = (-B_IK - sqrt(B_IK^2 - 4 * A_IK * C_IK)) / (2 * A_IK);
phi1 = 2 * atan(u);

% 求解phi4
D_IK = (xc + l4)^2 + yc^2 - l3^2;
E_IK = -4 * l4 * yc;
F_IK = (xc - l4)^2 + yc^2 - l3^2;
v = (-E_IK + sqrt(E_IK^2 - 4 * D_IK * F_IK)) / (2 * D_IK);
phi4 = 2 * atan(v);
disp(phi1);
disp(phi4);
end
