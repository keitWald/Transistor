%% 轮腿机器人正逆运动学验证脚本
% 此脚本通过正解->逆解->对比的方式验证运动学解算的正确性
% 作者: GitHub Copilot
% 日期: 2026年1月22日

clear; clc; close all;

%% 1. 设置连杆参数
% 请根据实际机器人参数修改这些值
l1 = 0.0955;  % 连杆1长度 (m)
l2 = 0.1142;  % 连杆2长度 (m)
l3 = 0.1142;  % 连杆3长度 (m)
l4 = 0.09556;  % 连杆4长度 (m)
l5 = 0.215;  % 连杆5长度 (m)

fprintf('===== 轮腿机器人正逆运动学验证 =====\n');
fprintf('连杆参数: l1=%.3f, l2=%.3f, l3=%.3f, l4=%.3f, l5=%.3f\n\n', l1, l2, l3, l4, l5);

%% 2. 单点验证测试
fprintf('【单点验证测试】\n');
% 测试输入：关节角度 phi1 和 phi4
phi1_test = deg2rad(0);   % 45度
phi4_test = deg2rad(180);  % -45度

fprintf('输入关节角度:\n');
fprintf('  phi1 = %.4f rad (%.2f deg)\n', phi1_test, rad2deg(phi1_test));
fprintf('  phi4 = %.4f rad (%.2f deg)\n\n', phi4_test, rad2deg(phi4_test));

% 步骤1: 正运动学求解 (phi1, phi4 -> l0, phi0)
[l0_fk, phi0_fk] = leg_fk(phi1_test, phi4_test, l1, l2, l3, l4, l5);
fprintf('正运动学结果:\n');
fprintf('  l0   = %.6f m\n', l0_fk);
fprintf('  phi0 = %.6f rad (%.2f deg)\n\n', phi0_fk, rad2deg(phi0_fk));

% 步骤2: 逆运动学求解 (l0, phi0 -> phi1, phi4)
[phi1_ik, phi4_ik] = leg_ik(l0_fk, phi0_fk, l1, l2, l3, l4, l5);
fprintf('逆运动学结果:\n');
fprintf('  phi1 = %.6f rad (%.2f deg)\n', phi1_ik, rad2deg(phi1_ik));
fprintf('  phi4 = %.6f rad (%.2f deg)\n\n', phi4_ik, rad2deg(phi4_ik));

% 步骤3: 计算误差
error_phi1 = abs(phi1_test - phi1_ik);
error_phi4 = abs(phi4_test - phi4_ik);
rmse_angle = sqrt((error_phi1^2 + error_phi4^2) / 2);

fprintf('误差分析:\n');
fprintf('  phi1 误差 = %.6e rad (%.6e deg)\n', error_phi1, rad2deg(error_phi1));
fprintf('  phi4 误差 = %.6e rad (%.6e deg)\n', error_phi4, rad2deg(error_phi4));
fprintf('  角度RMSE  = %.6e rad (%.6e deg)\n', rmse_angle, rad2deg(rmse_angle));

% 判断是否通过验证
tolerance = 1e-6;  % 容差阈值（弧度）
if rmse_angle < tolerance
    fprintf('  ✓ 单点验证通过！\n\n');
else
    fprintf('  ✗ 单点验证失败！误差超过容差\n\n');
end

%% 3. 批量验证测试
fprintf('【批量验证测试】\n');
% 生成测试数据集
n_tests = 50;
phi1_tests = linspace(deg2rad(20), deg2rad(70), n_tests);
phi4_tests = linspace(deg2rad(-70), deg2rad(-20), n_tests);

errors_phi1 = zeros(n_tests, 1);
errors_phi4 = zeros(n_tests, 1);
errors_rmse = zeros(n_tests, 1);

fprintf('生成 %d 组测试数据...\n', n_tests);
for i = 1:n_tests
    % 正运动学
    [l0_temp, phi0_temp] = leg_fk(phi1_tests(i), phi4_tests(i), l1, l2, l3, l4, l5);
    
    % 逆运动学
    [phi1_temp, phi4_temp] = leg_ik(l0_temp, phi0_temp, l1, l2, l3, l4, l5);
    
    % 计算误差
    errors_phi1(i) = abs(phi1_tests(i) - phi1_temp);
    errors_phi4(i) = abs(phi4_tests(i) - phi4_temp);
    errors_rmse(i) = sqrt((errors_phi1(i)^2 + errors_phi4(i)^2) / 2);
end

fprintf('批量测试完成！\n\n');

%% 4. 统计分析
fprintf('统计分析结果:\n');
fprintf('  phi1 误差 - 最大: %.6e rad, 平均: %.6e rad, 标准差: %.6e rad\n', ...
    max(errors_phi1), mean(errors_phi1), std(errors_phi1));
fprintf('  phi4 误差 - 最大: %.6e rad, 平均: %.6e rad, 标准差: %.6e rad\n', ...
    max(errors_phi4), mean(errors_phi4), std(errors_phi4));
fprintf('  总体RMSE  - 最大: %.6e rad, 平均: %.6e rad, 标准差: %.6e rad\n', ...
    max(errors_rmse), mean(errors_rmse), std(errors_rmse));

% 判断批量测试是否通过
if max(errors_rmse) < tolerance
    fprintf('  ✓ 批量验证通过！所有测试点误差均在容差范围内\n\n');
else
    fprintf('  ✗ 批量验证失败！部分测试点误差超过容差\n\n');
end

%% 5. 可视化结果
% figure('Name', '正逆运动学验证结果', 'Position', [100, 100, 1200, 800]);
% 
% % 子图1: phi1误差分布
% subplot(2, 3, 1);
% plot(rad2deg(phi1_tests), errors_phi1, 'b-', 'LineWidth', 1.5);
% hold on;
% plot([rad2deg(phi1_tests(1)), rad2deg(phi1_tests(end))], [tolerance, tolerance], 'r--', 'LineWidth', 1);
% grid on;
% xlabel('输入 phi1 (deg)');
% ylabel('误差 (rad)');
% title('phi1 误差分布');
% legend('误差', '容差阈值', 'Location', 'best');
% 
% % 子图2: phi4误差分布
% subplot(2, 3, 2);
% plot(rad2deg(phi4_tests), errors_phi4, 'g-', 'LineWidth', 1.5);
% hold on;
% plot([rad2deg(phi4_tests(1)), rad2deg(phi4_tests(end))], [tolerance, tolerance], 'r--', 'LineWidth', 1);
% grid on;
% xlabel('输入 phi4 (deg)');
% ylabel('误差 (rad)');
% title('phi4 误差分布');
% legend('误差', '容差阈值', 'Location', 'best');
% 
% % 子图3: RMSE分布
% subplot(2, 3, 3);
% plot(1:n_tests, errors_rmse, 'm-', 'LineWidth', 1.5);
% hold on;
% plot([1, n_tests], [tolerance, tolerance], 'r--', 'LineWidth', 1);
% grid on;
% xlabel('测试点编号');
% ylabel('RMSE (rad)');
% title('总体RMSE分布');
% legend('RMSE', '容差阈值', 'Location', 'best');
% 
% % 子图4: phi1误差对数尺度
% subplot(2, 3, 4);
% semilogy(rad2deg(phi1_tests), errors_phi1, 'b-', 'LineWidth', 1.5);
% hold on;
% semilogy([rad2deg(phi1_tests(1)), rad2deg(phi1_tests(end))], [tolerance, tolerance], 'r--', 'LineWidth', 1);
% grid on;
% xlabel('输入 phi1 (deg)');
% ylabel('误差 (rad, 对数)');
% title('phi1 误差分布 (对数尺度)');
% legend('误差', '容差阈值', 'Location', 'best');
% 
% % 子图5: phi4误差对数尺度
% subplot(2, 3, 5);
% semilogy(rad2deg(phi4_tests), errors_phi4, 'g-', 'LineWidth', 1.5);
% hold on;
% semilogy([rad2deg(phi4_tests(1)), rad2deg(phi4_tests(end))], [tolerance, tolerance], 'r--', 'LineWidth', 1);
% grid on;
% xlabel('输入 phi4 (deg)');
% ylabel('误差 (rad, 对数)');
% title('phi4 误差分布 (对数尺度)');
% legend('误差', '容差阈值', 'Location', 'best');
% 
% % 子图6: 误差直方图
% subplot(2, 3, 6);
% histogram(log10(errors_rmse), 20, 'FaceColor', [0.5, 0.5, 0.5]);
% hold on;
% xline(log10(tolerance), 'r--', 'LineWidth', 2);
% grid on;
% xlabel('log10(RMSE) (rad)');
% ylabel('频次');
% title('RMSE误差直方图');
% legend('误差分布', '容差阈值', 'Location', 'best');
% 
% %% 6. 工作空间可视化
% figure('Name', '正运动学工作空间', 'Position', [150, 150, 800, 600]);
% 
% % 生成工作空间网格
% n_grid = 20;
% phi1_range = linspace(deg2rad(20), deg2rad(70), n_grid);
% phi4_range = linspace(deg2rad(-70), deg2rad(-20), n_grid);
% 
% l0_workspace = zeros(n_grid, n_grid);
% phi0_workspace = zeros(n_grid, n_grid);
% 
% for i = 1:n_grid
%     for j = 1:n_grid
%         [l0_workspace(i,j), phi0_workspace(i,j)] = leg_fk(phi1_range(i), phi4_range(j), l1, l2, l3, l4, l5);
%     end
% end
% 
% % 转换为笛卡尔坐标
% x_workspace = l0_workspace .* cos(phi0_workspace);
% y_workspace = l0_workspace .* sin(phi0_workspace);
% 
% subplot(1, 2, 1);
% surf(rad2deg(phi1_range), rad2deg(phi4_range), l0_workspace');
% xlabel('phi1 (deg)');
% ylabel('phi4 (deg)');
% zlabel('l0 (m)');
% title('虚拟腿长度工作空间');
% colorbar;
% grid on;
% 
% subplot(1, 2, 2);
% plot(x_workspace(:), y_workspace(:), 'b.', 'MarkerSize', 5);
% hold on;
% plot(0, 0, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
% axis equal;
% grid on;
% xlabel('X (m)');
% ylabel('Y (m)');
% title('笛卡尔工作空间');
% legend('工作点', '原点', 'Location', 'best');
% 
% fprintf('===== 验证完成 =====\n');
% fprintf('图表已生成，请查看可视化结果。\n');
