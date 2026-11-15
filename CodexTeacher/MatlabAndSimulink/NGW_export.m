function info = NGW_export(outDir, opts)
%NGW_EXPORT  导出模型矩阵与参考时程，供 Wolfram Mathematica 独立验证。
%
%   本函数**不修改任何模型文件**，只读取 NGW_params / NGW_dynamics 的常量矩阵 D，
%   并把 Mathematica 侧需要的东西写成全精度 CSV + JSON。
%
%   用法：
%       NGW_export                              % 导出到 results_doc\ngw_export
%       NGW_export('D:\tmp\ngw')                % 指定目录
%       NGW_export('', JRotor=2.7e-3)           % 带电机转子惯量
%       NGW_export('', RunSimulink=false)       % 不跑 Simulink 基线
%
%   产出文件（目录内）：
%       M.csv            30x30   质量矩阵
%       K_struct.csv     30x30   结构刚度（支承 + 行星轮轴承）
%       C_struct.csv     30x30   结构阻尼 = beta*K_struct
%       K_mesh_avg.csv   30x30   平均啮合刚度矩阵 sum(k_i * B_i'B_i)
%       Bbig.csv          6x30   啮合投影矩阵，行序交错：2i-1 = 外啮合 i，2i = 内啮合 i
%       phases.csv        3x3    每个行星轮的 [phase_sp, phase_rp, phase_err]
%       meta.json               全部参数 + 布局 + 工况 + 来源信息
%       ref_t.csv / ref_q.csv / ref_v.csv / ref_acc.csv
%       ref_delta.csv / ref_Fmesh.csv
%                               ode15s 紧容差参考解（供 NDSolve 交叉比对）
%                               ref_acc.csv = 解析加速度 qdd（dx/dt 的下半块），
%                               供 Mathematica 直接代入运动方程算残差，避免数值微分
%       slx_t.csv / slx_state.csv / slx_mesh.csv
%                               NGW_mech_only.slx 的 Simulink 结果（可选）
%
%   精度：全部 '%.17g'（IEEE754 双精度可无损往返），索引 1-based。
%
%   注意：S-Function 输出端口 4 的顺序是 **分组** [sp1..spN; rp1..rpN]，
%         与本文件导出的 Fmesh（**交错**，与 Bbig 行对齐）不同，见 meta.json。

arguments
    outDir (1,:) char = ''
    opts.JRotor      (1,1) double  = 0
    opts.Te          (1,1) double  = 2.0
    opts.cL          (1,1) double  = 0.5
    opts.Tend        (1,1) double  = 0.3
    opts.dtOut       (1,1) double  = 1e-4
    opts.RunSimulink (1,1) logical = true
end

here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
if isempty(outDir)
    outDir = fullfile(here, 'results_doc', 'ngw_export');
end
if ~exist(outDir, 'dir'), mkdir(outDir); end

%% ---------- 1. 常量矩阵 ----------
p  = NGW_params(opts.JRotor);
x0 = zeros(60, 1);
[~, ~, D] = NGW_dynamics(0, x0, p, opts.Te, 0);

writeCSV(fullfile(outDir, 'M.csv'),          full(D.M));
writeCSV(fullfile(outDir, 'K_struct.csv'),   full(D.Kstruct));
writeCSV(fullfile(outDir, 'C_struct.csv'),   full(D.Cstruct));
writeCSV(fullfile(outDir, 'K_mesh_avg.csv'), full(D.Kmesh));
writeCSV(fullfile(outDir, 'Bbig.csv'),       full(D.Bbig));

ph = zeros(D.N, 3);
for i = 1:D.N
    ph(i,:) = [D.mesh(i).phase_sp, D.mesh(i).phase_rp, D.mesh(i).phase_err];
end
writeCSV(fullfile(outDir, 'phases.csv'), ph);

%% ---------- 2. 参考时程（紧容差 ode15s）----------
% TL = cL * omega_c，omega_c 是第 (n + i_c) 个状态
% 求解器内部仍用自适应小步长（RelTol 1e-10），但只在均匀网格 tspan 上输出，
% 否则 ode15s 会返回近 10^5 个点，CSV 会到上百 MB。
ic   = D.i_c;
rhs  = @(t, x) NGW_dynamics(t, x, p, opts.Te, opts.cL*x(D.n + ic));
oopt = odeset('RelTol', 1e-10, 'AbsTol', 1e-12, 'MaxStep', 1e-4);

tspan = 0:opts.dtOut:opts.Tend;
if tspan(end) < opts.Tend - 1e-12
    tspan(end+1) = opts.Tend;
end
[t, X] = ode15s(rhs, tspan, x0, oopt);

Nt    = numel(t);
delta = zeros(Nt, 2*D.N);
Frow  = zeros(Nt, 2*D.N);
Arow  = zeros(Nt, D.n);          % 解析加速度 qdd = dx/dt 的下半块
for k = 1:Nt
    xk = X(k,:).';
    [dxk, a] = NGW_dynamics(t(k), xk, p, opts.Te, opts.cL*xk(D.n + ic));
    delta(k,:)      = a.delta.';
    Frow(k, 1:2:end) = a.Fsp.';      % 按交错顺序放，与 Bbig 行对齐
    Frow(k, 2:2:end) = a.Frp.';
    Arow(k,:)        = dxk(D.n+1:2*D.n).';
end

writeCSV(fullfile(outDir, 'ref_t.csv'),      t);
writeCSV(fullfile(outDir, 'ref_q.csv'),      X(:, 1:D.n));
writeCSV(fullfile(outDir, 'ref_v.csv'),      X(:, D.n+1:2*D.n));
writeCSV(fullfile(outDir, 'ref_acc.csv'),    Arow);
writeCSV(fullfile(outDir, 'ref_delta.csv'),  delta);
writeCSV(fullfile(outDir, 'ref_Fmesh.csv'),  Frow);

%% ---------- 3. Simulink 基线（可选）----------
% Simulink 变步长会把每个内部步都记下来（几十 MB），这里插值到与 ode15s
% 参考解相同的均匀网格 tg，保证两边逐点可比、文件也小。
slxDone = false;
tg = t(:);
if opts.RunSimulink && exist(fullfile(here, 'NGW_mech_only.slx'), 'file')
    try
        prev = pwd; cd(here);
        out = sim('NGW_mech_only');
        cd(prev);
        [ts, st] = getlog(out.log_state);
        [tm, me] = getlog(out.log_mesh);
        st = interp1(ts, st, tg, 'linear', 'extrap');
        me = interp1(tm, me, tg, 'linear', 'extrap');
        writeCSV(fullfile(outDir, 'slx_t.csv'),     tg);
        writeCSV(fullfile(outDir, 'slx_state.csv'), st);
        writeCSV(fullfile(outDir, 'slx_mesh.csv'),  me);   % 端口 4：分组顺序
        slxDone = true;
    catch ME
        cd(here);
        warning('NGW_export:simulink', '跳过 Simulink 导出：%s', ME.message);
    end
end

%% ---------- 4. meta.json ----------
meta = struct();
meta.generated      = datestr(now, 'yyyy-mm-dd HH:MM:SS');
meta.matlab_version = version;
meta.purpose        = 'Independent verification package for Wolfram Mathematica';
meta.index_base     = 1;
meta.precision      = '%.17g';
meta.n_dof          = D.n;
meta.n_planets      = D.N;
meta.state_layout   = 'x = [q(1:30); v(31:60)]';
meta.q_layout       = 'q = [s(1:6) c(7:12) p1(13:18) p2(19:24) p3(25:30)]';
meta.dof_order      = {'x','y','z','thx','thy','thz'};
meta.units          = struct( ...
    'length','m', 'angle','rad', 'mass','kg', 'inertia','kg*m^2', ...
    'stiffness','N/m or N*m/rad', 'damping','N*s/m or N*m*s/rad', ...
    'force','N', 'torque','N*m');
meta.idx            = struct('s', D.idx.s, 'c', D.idx.c, ...
    'p1', D.idx.p{1}, 'p2', D.idx.p{2}, 'p3', D.idx.p{3});
meta.eom            = ['M*qdd = F_ext - K_struct*q - Bbig''*Fmesh - C_struct*qd ; ' ...
                       'Fmesh = k.*(Bbig*q - e) + c.*(Bbig*qd - ed)'];
meta.mesh_row_order = 'interleaved: row (2i-1) = sun-planet i, row (2i) = ring-planet i';
meta.port4_order    = 'WARNING: S-Function output port 4 is GROUPED [sp1..spN ; rp1..rpN]';
meta.reference_run  = struct( ...
    'Te', opts.Te, 'cL', opts.cL, 'TL_law', 'TL = cL*omega_c', ...
    't_end', opts.Tend, 'dt_out', opts.dtOut, 'x0', 'zeros(60,1)', ...
    'solver','ode15s', 'RelTol',1e-10, 'AbsTol',1e-12, 'MaxStep',1e-4);
meta.derived = struct( ...
    'i_ratio', p.i_ratio, 'rb1', p.rb1, 'rb2', p.rb2, 'rb3', p.rb3, ...
    'beta', D.beta, 'w1_rad_s', D.w1, 'f1_Hz', D.w1/(2*pi));
meta.params = p;
meta.simulink_exported = slxDone;

txt = jsonencode(meta, 'PrettyPrint', true);
fid = fopen(fullfile(outDir, 'meta.json'), 'w', 'n', 'UTF-8');
fwrite(fid, txt, 'char');
fclose(fid);

%% ---------- 5. 回显 ----------
info = struct('outDir', outDir, 'nFiles', 0, 'slx', slxDone);
d = dir(fullfile(outDir, '*'));
info.nFiles = sum(~[d.isdir]);

fprintf('\n导出于: %s\n', outDir);
fprintf('  矩阵: M, K_struct, C_struct, K_mesh_avg (%dx%d), Bbig (%dx%d), phases\n', ...
        D.n, D.n, 2*D.N, D.n);
fprintf('  参考解: %d 个时间点，t = 0 .. %.3g s，Te = %.3g N*m，TL = %.3g*omega_c\n', ...
        Nt, opts.Tend, opts.Te, opts.cL);
fprintf('  beta = %.6e s，w1 = %.4f rad/s，f1 = %.6f Hz\n', D.beta, D.w1, D.w1/(2*pi));
fprintf('  Simulink 基线: %s\n', string(slxDone));
fprintf('  共 %d 个文件。\n\n', info.nFiles);
end
%% ============================ 局部函数 ============================
function writeCSV(fname, A)
% 全精度 CSV，无表头（Mathematica 侧 Import[..., "Table", "FieldSeparators"->","]）
A = full(double(A));
if isvector(A) && size(A,1) == 1, A = A.'; end
[m, n] = size(A);
fmt = [repmat('%.17g,', 1, n-1) '%.17g\n'];
fid = fopen(fname, 'w');
if fid < 0
    error('NGW_export:io', '无法写入 %s', fname);
end
for i = 1:m
    fprintf(fid, fmt, A(i,:));
end
fclose(fid);
end

function [t, d] = getlog(s)
% 兼容 To Workspace 的三种 SaveFormat
if isa(s, 'timeseries')
    t = s.Time; d = s.Data;
elseif isstruct(s) && isfield(s, 'signals')
    t = s.time; d = s.signals.values;
elseif isstruct(s) && isfield(s, 'Data')
    t = s.Time; d = s.Data;
else
    d = s; t = (0:size(s,1)-1)';
end
end
