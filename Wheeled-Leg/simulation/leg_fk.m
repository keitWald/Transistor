function [l0, phi0] = leg_fk(phi1, phi4)
% 腿部偏置五连杆的运动学正解
% 输入:
%   phi1 - 关节1角度
%   phi4 - 关节4角度
%   l1, l2, l3, l4, l5 - 连杆长度参数
% 输出:
%   l0   - 虚拟腿长度
%   phi0 - 虚拟腿角度

l1 = 113.4;
l2 = 135;
l3 = 135;
l4 = 113.4;
l5 = 210;

% 计算B点和D点坐标
xb = l1 * cos(phi1);
yb = l1 * sin(phi1);
xd = l4 * cos(phi4);
yd = l4 * sin(phi4);

% 计算BD距离
dx = xd - xb;
dy = yd - yb;
l_bd = sqrt(dx^2 + dy^2);

% 求解phi2
A0 = 2 * l2 * dx;
B0 = 2 * l2 * dy;
C0 = l2^2 + l_bd^2 - l3^2;
phi2 = 2 * atan2(B0 - sqrt(A0^2 + B0^2 - C0^2), A0 + C0);

% 计算C点坐标
xc = l5 / l4 * (xb + l2 * cos(phi2));
yc = l5 / l4 * (yb + l2 * sin(phi2));

% 计算虚拟腿参数
l0 = sqrt(xc^2 + yc^2);
phi0 = atan2(yc, xc);
disp(l0);
disp(phi0);
end