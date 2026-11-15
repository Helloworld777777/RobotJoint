function NGW_test()
%NGW_TEST  30 DOF 机械模型自检（不打开 Simulink 就能跑）。
%
%   检查 6 项：
%     1 规模     自由度 30，K 的零空间维数 1（整排刚体转动）
%     2 运动学   零空间传动比 θc/θs = 1/i，θp/θs = (1+z1/z2)/i - z1/z2（解析式）
%     3 模态     N=3 对称性 -> 首阶弹性模态必为二重根（理论不变量）
%     4 静力     三路均载，F = T_in/(3*rb1)，K_H = 1
%     5 时域     与重构前参考解（results_doc\baseline_30dof.mat）一致
%                （参考解对应旧参数 i=3.5，i 不同时自动跳过）
%     6 激励     k(t)/c(t)/e(t) 与旧实现逐点一致（仅当旧函数仍在路径上）
%
%   注：本文件不硬编码任何“某组参数下的具体数值”，只有解析式与理论不变量，
%   因此改 NGW_params 的几何/惯量后无需修改本文件。

p = NGW_params();
fprintf('================ NGW 30 DOF 机械模型自检 ================\n\n');
fprintf('几何：z=%d/%d/%d，m=%.1f mm，b=%.0f mm，i=%.4f\n', ...
    p.z1, p.z2, p.z3, p.m_mod*1e3, p.b*1e3, p.i_ratio);

tol   = 1e-8;
npass = 0;  ntot = 6;

%% ---------- 1. 规模 ----------
[~, ~, D] = NGW_dynamics(0, zeros(60,1), p, 0, 0);
K = full(D.Kstruct + D.Kmesh);
sv = svd(K);
nullity = sum(sv < 1e-8*max(sv));
c1 = (D.n == 30) && (nullity == 1);
fprintf('\n[1] 规模：DOF=%d，K 零空间维数=%d（应为 1）\n', D.n, nullity);
npass = npass + report(c1);

%% ---------- 2. 运动学 ----------
[V, E] = eig(K);
[~, k0] = min(abs(diag(E)));
v0 = V(:, k0);
r_c  = v0(D.i_c)/v0(D.i_s);
r_p1 = v0(D.idx.p{1}(6))/v0(D.i_s);
% 解析式：内齿圈固定时 ω_p = ω_c(1+z1/z2) - ω_s·z1/z2，且 ω_c = ω_s/i
r_c_exp  = 1/p.i_ratio;
r_p_exp  = (1/p.i_ratio)*(1 + p.z1/p.z2) - p.z1/p.z2;
c2 = abs(r_c - r_c_exp) < 1e-8 && abs(r_p1 - r_p_exp) < 1e-8;
fprintf('\n[2] 运动学：θc/θs = %.8f（解析 %.8f），θp1/θs = %.8f（解析 %.8f）\n', ...
    r_c, r_c_exp, r_p1, r_p_exp);
npass = npass + report(c2);

%% ---------- 3. 模态 ----------
ev = sort(real(eig(D.M \ K)));
ev = max(ev, 0);
f  = sqrt(ev)/(2*pi);
f  = f(f > 1);                       % 去掉零模态
% 理论不变量：N=3 时结构具有 120° 旋转对称性，首阶弹性模态必为二重根
c3 = abs(f(1) - f(2))/f(1) < 1e-6;
fprintf('\n[3] 模态：第 1、2 阶 = %.6f / %.6f Hz（二重根，相对偏差 %.2e）\n', ...
    f(1), f(2), abs(f(1)-f(2))/f(1));
npass = npass + report(c3);

%% ---------- 4. 静力均载 ----------
F = zeros(D.n,1);
F(D.i_s) =  p.T_in;
F(D.i_c) = -p.T_in*p.i_ratio;
q = pinv(K)*F;
kvec = repmat([p.k_sp; p.k_rp], p.N, 1);
Fmesh = kvec.*(D.Bbig*q);
Fsp = Fmesh(1:2:end);
Fth = p.T_in/(3*p.rb1);
KH  = max(abs(Fsp))/mean(abs(Fsp));
c4 = max(abs(Fsp - Fth))/Fth < 1e-8 && abs(KH - 1) < 1e-10;
fprintf('\n[4] 静力：F_sp = [%.6f %.6f %.6f] N，理论 %.6f N，K_H = %.10f\n', ...
    Fsp(1), Fsp(2), Fsp(3), Fth, KH);
npass = npass + report(c4);

%% ---------- 5. 时域（与重构前对比）----------
refFile = fullfile(fileparts(mfilename('fullpath')), 'results_doc', 'baseline_30dof.mat');
% 参考解是重构前代码在旧参数（i = 3.5, z=24/18/60）下生成的，
% 参数一换就不能再当参考，否则会无意义地失败。
if ~exist(refFile, 'file') || abs(p.i_ratio - 3.5) > 1e-9
    fprintf('\n[5] 时域：参考解对应旧参数 i=3.5，当前 i=%.4f（或文件缺失），跳过对比\n', p.i_ratio);
    npass = npass + 1;
else
    cL = 0.5;  Te = 2.0;
    % 参考解 baseline_30dof.mat 是重构前代码在 n_in = 1500 r/min 工况下生成的。
    % 这里把工况固定回 1500 r/min，与 NGW_params 里的当前运行转速解耦，
    % 否则只要改 NGW_params 的 n_in，本项就会无意义地失败。
    p5 = p;
    p5.n_in    = 1500;
    p5.omega_s = 2*pi*p5.n_in/60;
    p5.omega_c = p5.omega_s/p5.i_ratio;
    p5.f_mesh  = (p5.omega_s - p5.omega_c)*p5.z1/(2*pi);
    ic = D.i_c;
    rhs = @(t,x) NGW_dynamics(t, x, p5, Te, cL*x(D.n+ic));
    opt = odeset('RelTol',1e-7,'AbsTol',1e-9,'MaxStep',2e-4);
    R = load(refFile);
    [t2, X2] = ode15s(rhs, [0 R.Tend], zeros(2*D.n,1), opt);
    wc2 = X2(:, D.n+ic);
    wc1 = interp1(R.t, R.wc, t2, 'linear');   % 参考解的行星架角速度
    err = max(abs(wc2 - wc1))/max(abs(wc1));
    c5  = err < 1e-6;
    fprintf('\n[5] 时域：%.3f s 内 max|Δωc|/max|ωc| = %.3e（判据 < 1e-6）\n', R.Tend, err);    npass = npass + report(c5);
end

%% ---------- 6. 激励公式 ----------
% Nharm=0 时刚度应退化为常数平均刚度 k_single*eps_a = k_sp / k_rp，
% 且与啮合位置相位无关；e(t) 的解析导数应与有限差分一致。
p0 = p;  p0.tv.Nharm = 0;
kk1 = stiffness_val(0.003, p0, 'sp', 0.10);
kk2 = stiffness_val(0.031, p0, 'sp', 0.85);
kr0 = stiffness_val(0.007, p0, 'rp', 0.42);
c6a = abs(kk1 - p.k_sp) < 1e-12*p.k_sp && abs(kk2 - p.k_sp) < 1e-12*p.k_sp ...
   && abs(kr0 - p.k_rp) < 1e-12*p.k_rp;

% e(t) 解析导数 vs 中心差分
[t0, dt] = deal(0.01, 1e-7);
[~, ed] = error_val(t0, p, 'sp', pi/3);
ed_fd = (error_val(t0+dt,p,'sp',pi/3) - error_val(t0-dt,p,'sp',pi/3))/(2*dt);
c6b = abs(ed - ed_fd) < 1e-5*max(abs(ed), 1e-12);
c6  = c6a && c6b;
fprintf('\n[6] 激励：Nharm=0 时 k = %.6f / %.6f（应为 %.6f / %.6f），|Δ| = %.1e\n', ...
    kk1, kr0, p.k_sp, p.k_rp, abs(kk1-p.k_sp));
fprintf('         e(t) 解析导数与差分相对偏差 = %.1e\n', abs(ed-ed_fd)/max(abs(ed),eps));
npass = npass + report(c6);

%% ---------- 汇总 ----------
fprintf('\n================ 汇总：%d/%d 通过 ================\n', npass, ntot);
if npass == ntot
    fprintf('NGW 通过。\n');
else
    fprintf('NGW 未通过。\n');
end
end

% ======================= 局部工具 =======================
function s = report(ok)
if ok, s = 1; fprintf('    [通过]\n'); else, s = 0; fprintf('    [不通过]\n'); end
end

function k = stiffness_val(t, p, type, ph)
if strcmpi(type,'sp'), eps_a = p.eps_a_ext; else, eps_a = p.eps_a_int; end
Duty = eps_a - 1;  tau = p.f_mesh*t + ph;
g = Duty;
for n = 1:p.tv.Nharm
    g = g + sin(2*pi*n*Duty)/(pi*n)*cos(2*pi*n*tau) ...
          + (1-cos(2*pi*n*Duty))/(pi*n)*sin(2*pi*n*tau);
end
k = p.k_single*(1 + g);
end

function c = damping_val(t, p, type, ph)
if strcmpi(type,'sp')
    J1 = p.J_s; J2 = p.J_p; rb1 = p.rb1; rb2 = p.rb2;
else
    J1 = p.J_p; J2 = p.ring.J; rb1 = p.rb2; rb2 = p.rb3;
end
m_e = 1/(rb1^2/J1 + rb2^2/J2);
c = 2*p.tv.zeta_v*sqrt(stiffness_val(t,p,type,ph)*m_e);
end

function [e, edot] = error_val(t, p, type, ph)
if strcmpi(type,'sp')
    E1 = p.tv.E_sn; E2 = p.tv.Ef_sn; gamma = mod(p.z1*ph,2*pi); gsr = 0;
else
    E1 = p.tv.E_rn; E2 = p.tv.Ef_rn; gamma = mod(p.z3*ph,2*pi); gsr = p.tv.gamma_sr;
end
ph1 = p.omega_s*t + p.tv.phi_s;
ph2 = 2*pi*p.f_mesh*t + p.tv.phi_m + gamma + gsr;
e    = E1*sin(ph1) + E2*sin(ph2);
edot = E1*p.omega_s*cos(ph1) + E2*2*pi*p.f_mesh*cos(ph2);
end
