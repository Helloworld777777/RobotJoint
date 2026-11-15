function [dx, aux, D] = NGW_dynamics(t, x, p, Te, TL)
%NGW_DYNAMICS  3 行星 NGW 纯机械动力学（30 DOF，内齿圈固定）。
%
%   状态 x = [q(30); v(30)]，q 为广义位移、v 为广义速度；
%   每个构件 6 个自由度 [x y z thx thy thz]：
%       q( 1: 6) 太阳轮 s     q( 7:12) 行星架 c
%       q(13:18) 行星轮 1     q(19:24) 行星轮 2     q(25:30) 行星轮 3
%
%   方程（M、K、C 见下方各板块）：
%       M q̈ = F_ext - K_struct q - B'*F_mesh - C_struct q̇
%       F_mesh = k(t).*(B q - e(t)) + c(t).*(B q̇ - ė(t))     (2N 路)
%
%   输入：t 时间 [s]；Te 电磁转矩 [N*m]（作用于太阳轮 θs，正号驱动）；
%         TL 负载阻力矩 [N*m]（作用于行星架 θc，正号阻力）
%   输出：dx 状态导数；aux 辅助量（转角/转速/啮合力），供输出端口使用；
%         D 常量矩阵缓存（M/K/C/B），供离线测试与调试取用

D = model_cache(p);

n = D.n;
q = x(1:n);
v = x(n+1:2*n);

% --- 激励力：2N 路啮合力 ---
Fmesh = mesh_force(t, D, q, v);

% --- 外激励 ---
F = zeros(n,1);
F(D.i_s) =  Te;
F(D.i_c) = -TL;

% --- 状态方程 ---
acc = D.Mdec \ (F - D.Kstruct*q - D.Cstruct*v - D.Bbig.'*Fmesh);    % D.Bbig.'*Fmesh是时变啮合部分
dx  = [v; acc];

% --- 辅助量（供 Outputs 端口，不参与积分）---
aux = struct();
aux.theta_c = q(D.i_c);      aux.omega_c = v(D.i_c);
aux.theta_s = q(D.i_s);      aux.omega_s = v(D.i_s);
aux.Fsp = Fmesh(1:2:end);    aux.Frp = Fmesh(2:2:end);
aux.delta = D.Bbig*q;        % 各路啮合变形
end

%% ======================= 常量矩阵（只算一次）=======================
                % 首次构建、参数不变就复用、参数变了就重建
function D = model_cache(p)
persistent CACHE
if isempty(CACHE) || ~isequaln(CACHE.p, p)
    CACHE = build(p);
end
D = CACHE;
end

% -------------------------------------------------------------------
function D = build(p)
N = p.N;
n = 6 + 6 + 6*N;                 % 30

% ---- 1. 自由度索引 ----
idx.s = 1:6;
idx.c = 7:12;
idx.p = cell(1,N);
for i = 1:N
    idx.p{i} = 12 + 6*(i-1) + (1:6);
end

% ---- 2. 质量矩阵 M ----
M = zeros(n);
% 太阳轮绕 z 轴惯量 = 齿轮本体 + 电机转子（同一根轴，无减速）
Js_total = p.J_s + p.J_rotor;
M(idx.s, idx.s) = diag([p.m_s p.m_s p.m_s Js_total/2 Js_total/2 Js_total]);
% 行星架绕 z 轴惯量 = 本体 + 输出端负载惯量（薄盘近似：横向取一半）
Jc_total = p.J_c1 + p.J_load;
M(idx.c, idx.c) = diag([p.m_c1 p.m_c1 p.m_c1 ...
                        p.J_c1_xx + p.J_load/2, p.J_c1_xx + p.J_load/2, Jc_total]);
Mp = diag([p.m_p p.m_p p.m_p p.J_p/2 p.J_p/2 p.J_p]);
for i = 1:N
    M(idx.p{i}, idx.p{i}) = Mp;
end

% ---- 3. 结构刚度 K_struct（支承 + 行星轮轴承，不含啮合）----
Ks = zeros(n);
Ks(idx.s, idx.s) = diag([p.k_sun_bearing p.k_sun_bearing p.k_sun_bearing ...
                         p.k_tilt_bearing p.k_tilt_bearing 0]);
Ks(idx.c, idx.c) = diag([p.k_carrier_bearing p.k_carrier_bearing p.k_carrier_bearing ...
                         p.k_tilt_bearing p.k_tilt_bearing 0]);
Kb_p = diag([p.k_planet_bearing p.k_planet_bearing p.k_planet_bearing ...
             p.k_tilt_bearing p.k_tilt_bearing 0]);
for i = 1:N
    ph = 2*pi*(i-1)/N;
    rr = [cos(ph); sin(ph); 0];
    if strcmpi(p.bearing_layout, 'pair')
        % 两端双轴承：z = ∓b/2
        for sg = [-1 1]
            TA = t_off_local(sg*(p.b/2)*[0;0;1]);      % 行星轮本体 → 轴承处
            TB = t_off_local(p.a*rr + sg*(p.b/2)*[0;0;1]); % 行星架 → 轴承处
            id = [idx.p{i}, idx.c];
            Ks(id, id) = Ks(id, id) + bearing_K(Kb_p, TA, TB);
        end
    else
        % 中心单轴承
        Ks([idx.p{i}, idx.c], [idx.p{i}, idx.c]) = ...
            Ks([idx.p{i}, idx.c], [idx.p{i}, idx.c]) + ...
            bearing_K(Kb_p, eye(6), t_off_local(p.a*rr));
    end
end

% ---- 4. 啮合几何：B 矩阵（2N×n）与静态啮合刚度 K_mesh ----
Bbig  = sparse(2*N, n);
Kmesh = sparse(n, n);
mesh  = struct('phase_sp', {}, 'phase_rp', {}, 'phase_err', {});
for i = 1:N
    ph = 2*pi*(i-1)/N;
    rr = [cos(ph); sin(ph); 0];
    tt = [-sin(ph); cos(ph); 0];
    n_sp = cos(p.alpha)*tt + sin(p.alpha)*rr;      % 啮合线方向

    % 外啮合：太阳轮 — 行星轮 i
    [Bsp, Ksp] = mesh_elem(n_sp, p.r1*rr, -p.r2*rr, p.k_sp);
    Bbig(2*i-1, [idx.s, idx.p{i}]) = Bsp;
    Kmesh([idx.s, idx.p{i}], [idx.s, idx.p{i}]) = ...
        Kmesh([idx.s, idx.p{i}], [idx.s, idx.p{i}]) + Ksp;

    % 内啮合：行星轮 i — 固定齿圈（刚度直接加在行星轮节点）
    Brp = n_sp.' * [eye(3), -skew3_local(p.r2*rr)];
    Bbig(2*i, idx.p{i}) = Brp;
    Kmesh(idx.p{i}, idx.p{i}) = Kmesh(idx.p{i}, idx.p{i}) + p.k_rp*(Brp.'*Brp);

    % 三路相位：外啮合 (i-1)/N，内啮合 +0.5 个啮合周期，误差按位置角
    mesh(i).phase_sp  = (i-1)/N;
    mesh(i).phase_rp  = (i-1)/N + 0.5;
    mesh(i).phase_err = ph;
end

% ---- 5. 结构阻尼 C_struct = beta*K_struct ----
% beta 由最低阶弹性模态反算：c = 2*zeta*w 型比例阻尼。
% 不使用 alpha*M：质量比例项会给"整条传动链的自由转动"引入虚假阻尼。
Kstruct = Ks;
Cstruct = sparse(n, n);
beta = 0;  w1 = NaN;
ev = sort(real(eig(M \ full(Ks + Kmesh))));
ev = max(ev, 0);
w  = sqrt(ev(ev > 1e-8*max(ev)));
if ~isempty(w)
    w1 = w(1);
    beta = 2*p.damp.zeta_b/w1;
    Cstruct = beta*Kstruct;
end

% ---- 打包 ----
D = struct();
D.p = p;  D.n = n;  D.N = N;  D.idx = idx;
D.M = M;  D.Kstruct = Kstruct;  D.Cstruct = sparse(Cstruct);
D.Kmesh = Kmesh;  D.Bbig = Bbig;  D.mesh = mesh;
D.i_s = idx.s(6);  D.i_c = idx.c(6);
D.Mdec = decomposition(M);     % 分解一次，每步只做回代
D.beta = beta;  D.w1 = w1;
end

%% ======================= 激励：2N 路啮合力 =======================
function Fmesh = mesh_force(t, D, q, v)
N = D.N;  p = D.p;
kvec  = zeros(2*N,1);  cvec  = zeros(2*N,1);
evec  = zeros(2*N,1);  edvec = zeros(2*N,1);    %  时变啮合刚度;时变啮合阻尼;齿轮误差位移;误差的时间导数
for i = 1:N
    kvec(2*i-1) = stiffness_tv(t, p, 'sp', D.mesh(i).phase_sp);
    cvec(2*i-1) = damping_tv(t, p, 'sp', D.mesh(i).phase_sp);
    [evec(2*i-1), edvec(2*i-1)] = error_tv(t, p, 'sp', D.mesh(i).phase_err);

    kvec(2*i) = stiffness_tv(t, p, 'rp', D.mesh(i).phase_rp);
    cvec(2*i) = damping_tv(t, p, 'rp', D.mesh(i).phase_rp);
    [evec(2*i), edvec(2*i)] = error_tv(t, p, 'rp', D.mesh(i).phase_err);
end
delta  = D.Bbig*q;
ddelta = D.Bbig*v;
Fmesh = kvec.*(delta - evec) + cvec.*(ddelta - edvec);  % 沿啮合线方向的 6个 标量力（sp1、rp1、sp2、rp2、sp3、rp3）,考虑了齿轮误差位移
end

%% ======================= 时变啮合刚度 k(t) =======================
function k = stiffness_tv(t, p, meshType, phase)
% 单/双齿交替的方波近似（Fourier 截断）；'velex' 与 'square' 直齿下等价。
if strcmpi(meshType, 'sp')
    eps_a = p.eps_a_ext;
else
    eps_a = p.eps_a_int;
end
Duty = eps_a - 1;                       % 双齿接触占空比
tau  = p.f_mesh*t + phase;              % 归一化啮合周期

model = 'square';
if isfield(p.tv, 'model') && ~isempty(p.tv.model)
    model = lower(char(p.tv.model));
end

switch model
    case 'const'
        k = p.k_single*eps_a;
    otherwise
        g = Duty;
        for n = 1:p.tv.Nharm
            an = sin(2*pi*n*Duty)/(pi*n);
            bn = (1 - cos(2*pi*n*Duty))/(pi*n);
            g = g + an*cos(2*pi*n*tau) + bn*sin(2*pi*n*tau);
        end
        k = p.k_single*(1 + g);
end
end

%% ======================= 时变啮合阻尼 c(t) =======================
function c = damping_tv(t, p, meshType, phase)
% c(t) = 2*zeta_v*sqrt(k(t)*m_e)，m_e 为两轮折算到啮合线的等效质量
if strcmpi(meshType, 'sp')
    J1 = p.J_s + p.J_rotor;  J2 = p.J_p;  rb1 = p.rb1;  rb2 = p.rb2;
else
    J1 = p.J_p;  J2 = p.ring.J;  rb1 = p.rb2;  rb2 = p.rb3;
end
m_e = 1/(rb1^2/J1 + rb2^2/J2);
c = 2*p.tv.zeta_v*sqrt(stiffness_tv(t, p, meshType, phase)*m_e);
end

%% ======================= 传递误差 e(t) =======================
function [e, edot] = error_tv(t, p, meshType, planetPhase)
% e(t) = E1*sin(w_s t + phi_s) + E2*sin(w_m t + phi_m + gamma + gamma_sr)
if strcmpi(meshType, 'sp')
    E1 = p.tv.E_sn;  E2 = p.tv.Ef_sn;
    gamma = mod(p.z1*planetPhase, 2*pi);
    gamma_sr = 0;
else
    E1 = p.tv.E_rn;  E2 = p.tv.Ef_rn;
    gamma = mod(p.z3*planetPhase, 2*pi);
    gamma_sr = p.tv.gamma_sr;
end
w1 = p.omega_s;
w2 = 2*pi*p.f_mesh;
ph1 = w1*t + p.tv.phi_s;
ph2 = w2*t + p.tv.phi_m + gamma + gamma_sr;
e    = E1*sin(ph1) + E2*sin(ph2);
edot = E1*w1*cos(ph1) + E2*w2*cos(ph2);
end

%% ======================= 小工具 =======================
function S = skew3_local(v)
S = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
end

function T = t_off_local(r)
% 刚体偏置变换：X_P = T*[u; theta]，u_P = u + theta×r
T = [eye(3), -skew3_local(r); zeros(3), eye(3)];
end

function Kc = bearing_K(Kb, TA, TB)
% 两连接点之间的相对位移弹簧（TB = [] 表示接地）
if nargin < 3 || isempty(TB)
    Kc = TA.'*Kb*TA;
else
    Kc = [ TA.'*Kb*TA, -TA.'*Kb*TB
          -TB.'*Kb*TA,  TB.'*Kb*TB ];
end
end

function [B, K] = mesh_elem(n, rA, rB, k)
% 12 自由度啮合单元：B = n'*[E -[rA]x -E [rB]x]，K = k*B'B
n = n(:); rA = rA(:); rB = rB(:);
B = n.' * [eye(3), -skew3_local(rA), -eye(3), skew3_local(rB)];
K = k*(B.'*B);
end
