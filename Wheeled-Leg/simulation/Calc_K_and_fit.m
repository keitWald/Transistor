%% compute_K_and_fit.m
% 功能：
% 1) 从 A_fun/B_fun 计算不同腿长组合下的离散 LQR 增益 K
% 2) 对 K 的每个元素做多项式拟合，输出 K_cons (40x6)

clear; clc;
t_all = tic;

%% 1) 读取 A_fun / B_fun
load('num_matrix.mat', 'A_fun', 'B_fun');  % 你已经生成好的函数句柄

%% 2) 读取腿部测量表（l0, l_wl, l_bl, I_ll）
load('leg_data.mat','l0','l_bl','l_wl','I_ll')

%% 3) 设置采样周期、Q/R
Ts = 0.002;
  % 矩阵Q中，以下列分别对应：
%        s     ds     phi     dphi     theta_ll dtheta_ll theta_lr dtheta_lr theta_b dtheta_b
  % 矩阵R中，以下列分别对应：
%        T_wl    T_wr     T_bl     T_br
% ---- 示例：正常行驶 ----
% Q_cost = diag([100 400 1 150 10 2 10 2 4200 67.5]);
% R_cost = diag([2.1 2.1 0.14 0.14]);
Q_cost = diag([100 600 1 150 10 2 10 2 4200 67.5]);
R_cost = diag([2.4 2.4 0.12 0.12]);

%% 4) 计算并拟合
[K_fit, fit_debug] = calculate_and_fit_coeffs_fun( ...
    A_fun, B_fun, Q_cost, R_cost, Ts, ...
    l0, l_wl, l_bl, I_ll);

%% 5) 保存结果
save('lqr_fitting_results.mat', 'K_fit', 'fit_debug', 'Q_cost', 'R_cost', 'Ts');

%% 6) Export C code for embedded (40x6)
% Format:
%   const float K_coeffs_normal[K_ROWS][K_COEFFS] = { ... };
c_out = 'K_coeffs_normal.h';
write_c_array_2d_float(c_out, 'K_coeffs_normal', K_fit);
fprintf('\nExported C header: %s\n', c_out);
disp('完成：已保存 lqr_fitting_results.mat');
fprintf('\n===== K_fit (each row: c0 c1 c2 c3 c4 c5) =====\n');
for i = 1:size(K_fit,1)
    fprintf('%02d: %.4f  %.4f  %.4f  %.4f  %.4f  %.4f\n', ...
        i, K_fit(i,1), K_fit(i,2), K_fit(i,3), K_fit(i,4), K_fit(i,5), K_fit(i,6));
end


fprintf('总耗时：%.2f 秒 (%.2f 分钟)\n', toc(t_all), toc(t_all)/60);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 本地函数区（脚本内函数，MATLAB R2016b+ 支持）
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [K_cons, fit_debug] = calculate_and_fit_coeffs_fun( ...
        A_fun, B_fun, Q_cost, R_cost, Ts, ...
        l0, l_wl, l_bl, I_ll)

    % 用测量表 l0 作为腿长扫描序列（最稳，不会和表格错位）
    leg_length = l0(:);
    len = numel(leg_length);
    sample_size = len^2;

    K = zeros(sample_size, 40);
    x1 = zeros(sample_size, 1);  % l_l
    x2 = zeros(sample_size, 1);  % l_r

    p = 1;
    for i = 1:len
        l_varl = leg_length(i);
        % Use measured table (leg_data.mat) directly via interpolation.
        [l_wl_ac, l_bl_ac, I_ll_ac] = get_leg_params(l_varl, l0, l_wl, l_bl, I_ll);

        for j = 1:len
            l_varr = leg_length(j);
            [l_wr_ac, l_br_ac, I_lr_ac] = get_leg_params(l_varr, l0, l_wl, l_bl, I_ll);
            
            % 直接调用函数句柄生成数值 A/B
            trans_A = A_fun(l_varl, l_varr, l_wl_ac, l_wr_ac, l_bl_ac, l_br_ac, I_ll_ac, I_lr_ac);
            trans_B = B_fun(l_varl, l_varr, l_wl_ac, l_wr_ac, l_bl_ac, l_br_ac, I_ll_ac, I_lr_ac);

            % 可控性检查（可选，挺耗时）
            if rank(ctrb(trans_A, trans_B)) < size(trans_A, 1)
                warning('增广系统不可控: l_l=%.3f, l_r=%.3f', l_varl, l_varr);
            end

            % 离散 LQR
            KK = lqrd(trans_A, trans_B, Q_cost, R_cost, Ts);

            K(p, :) = reshape(KK', 1, 40);
            x1(p) = l_varl;
            x2(p) = l_varr;
            p = p + 1;
        end

        % 进度提示
        fprintf('进度：%d/%d (左腿长=%.3f)\n', i, len, l_varl);
    end

    % 拟合特征
    X = [x1, x2, x1.^2, x2.^2, x1.*x2];

    % 每个 K 元素拟合出 6 个系数（含截距）
    K_cons = zeros(40, 6);

    % 可选：保存一些拟合诊断信息
    fit_debug = struct();
    fit_debug.R2 = zeros(40,1);
    fit_debug.RMSE = zeros(40,1);

    for k = 1:40
        mdl = fitlm(X, K(:, k));  % 默认包含截距
        K_cons(k, :) = mdl.Coefficients.Estimate';  % [c0 c1 c2 c3 c4 c5]

        fit_debug.R2(k) = mdl.Rsquared.Ordinary;
        fit_debug.RMSE(k) = mdl.RMSE;
    end
end


function [l_w, l_b, I_leg] = get_leg_params(l_query, l0, l_wl, l_bl, I_ll)
    % 用插值方式从表格得到对应腿长下的参数（更稳，避免浮点索引错位）
    % l_query: 单位 m

    l_w   = interp1(l0, l_wl, l_query, 'linear', 'extrap');
    l_b   = interp1(l0, l_bl, l_query, 'linear', 'extrap');
    I_leg = interp1(l0, I_ll, l_query, 'linear', 'extrap');
end

function write_c_array_2d_float(filename, var_name, mat)
    if ~ismatrix(mat) || size(mat, 2) ~= 6
        error('Expected a 2D matrix with 6 columns, got %dx%d', size(mat, 1), size(mat, 2));
    end
    if ~all(isfinite(mat(:)))
        error('Matrix contains NaN/Inf, refusing to export.');
    end

    fileID = fopen(filename, 'wt');
    if fileID < 0
        error('Failed to open file for writing: %s', filename);
    end
    cleaner = onCleanup(@() fclose(fileID));

    fprintf(fileID, '#pragma once\n\n');
    fprintf(fileID, '#ifndef K_ROWS\n#define K_ROWS %d\n#endif\n', size(mat, 1));
    fprintf(fileID, '#ifndef K_COEFFS\n#define K_COEFFS %d\n#endif\n\n', size(mat, 2));

    fprintf(fileID, 'const float %s[K_ROWS][K_COEFFS] = {\n', var_name);
    for i = 1:size(mat, 1)
        fprintf(fileID, '    {');
        for j = 1:size(mat, 2)
            if j < size(mat, 2)
                fprintf(fileID, '%.9gf, ', mat(i, j));
            else
                fprintf(fileID, '%.9gf', mat(i, j));
            end
        end
        if i < size(mat, 1)
            fprintf(fileID, '},\n');
        else
            fprintf(fileID, '}\n');
        end
    end
    fprintf(fileID, '};\n');
end

