function p = NGW_params(J_rotor, J_load)
%NGW_PARAMS  3 行星 NGW 减速器参数（30 DOF，内齿圈固定）。
%
%   本文件是**唯一参数源**：改参数只改这里，其余文件不出现硬编码数值。
%   单位：长度 m、角度 rad、质量 kg、转动惯量 kg*m^2、刚度 N/m、N*m/rad。
%
%   参数来源：谢飞《足式机器人关节动力学建模与控制研究》
%     表2.1 行星减速器参数：z_a/z_g/z_b = 12/60/132, i = 12,
%           J_a/J_g/J_H = 2.5987/7.9715/38.5660 kg·mm^2, M_g = 0.04316 kg, r_H = 21.83 mm
%     表4.6 关节物理参数（电机）：Rs = 1.05 Ω, Ld=Lq = 1.46 mH, ψf = 0.01337 Wb,
%           Pn = 14, Jm = 105.21 kg·mm^2, Jeq = 3.53416 kg·mm^2, JL = 4.53 kg·cm^2
%     论文未给的参数（模数由 r_H 反推；齿宽/轴承刚度/阻尼/传递误差）均标注 [假设]
%
%   使用方法：
%       p = NGW_params();                    % 纯机械（转子/负载惯量 0）
%       p = NGW_params(J_rotor);             % 接电机
%       p = NGW_params(J_rotor, J_load);     % 接电机 + 输出端负载惯量
%       在 S-Function 的 Parameters 里填 NGW_params(J_rotor, J_load)

if nargin < 1 || isempty(J_rotor), J_rotor = 0; end   % 纯机械基线：不接电机
if nargin < 2 || isempty(J_load),  J_load  = 0; end   % 不接负载

p = struct();
p.J_rotor = J_rotor;            % 电机转子惯量，并入太阳轮绕 z 轴 [kg*m^2]
p.J_load  = J_load;             % 负载惯量，并入行星架（输出轴）[kg*m^2]

%% ==================== 1. 几何（论文 表2.1）====================
% 数据源：谢飞《足式机器人关节动力学建模与控制研究》表2.1
p.z1 = 12;                      % 太阳轮齿数 z_a
p.z2 = 60;                      % 行星轮齿数 z_g
p.z3 = 132;                     % 内齿圈齿数 z_b
p.N  = 3;                       % 行星轮个数 N

p.alpha = deg2rad(20);          % 压力角 [rad]（论文未给，取标准值；z_a<17 需变位）
p.a     = 21.83e-3;             % 中心距 = 行星架半径 r_H [m]（表2.1）
p.b     = 4e-3;                 % [假设] 齿宽 [m]，论文未给，按 b/m≈6.6 取

% 标准齿轮：a = m*(z1+z2)/2  =>  由 r_H 反推模数
p.m_mod = 2*p.a/(p.z1 + p.z2);  % ≈ 0.6064 mm

p.r1 = p.m_mod*p.z1/2;          % 太阳轮分度圆半径
p.r2 = p.m_mod*p.z2/2;          % 行星轮分度圆半径
p.r3 = p.m_mod*p.z3/2;          % 内齿圈分度圆半径
p.rb1 = p.r1*cos(p.alpha);      % 基圆半径
p.rb2 = p.r2*cos(p.alpha);
p.rb3 = p.r3*cos(p.alpha);
p.ra1 = p.r1 + p.m_mod;         % 齿顶圆半径
p.ra2 = p.r2 + p.m_mod;
p.ra3 = p.r3 - p.m_mod;

p.i_ratio = 1 + p.z3/p.z1;      % = 12（传动比唯一来源，表2.1）

% 几何合法性：改齿数后不满足条件会立刻报错，而不是算出无意义的数
assert(p.z3 == p.z1 + 2*p.z2, 'NGW 几何不满足同心条件：z3 应等于 z1+2*z2');
assert(mod(p.z1 + p.z3, p.N) == 0, 'NGW 几何不满足装配条件：(z1+z3)/N 必须为整数');
assert(p.z2 >= 17, '行星轮齿数小于不根切最少齿数 17');
assert(abs((p.r1 + p.r2) - p.a) < 1e-9, '由 r_H 反推的模数与齿数不自洽');

%% ==================== 2. 材料 ====================
p.rho = 7850;                   % 密度 [kg/m^3]
p.E   = 2.1e11;                 % 弹性模量 [Pa]
p.nu  = 0.3;                    % 泊松比
p.G   = p.E/(2*(1+p.nu));       % 剪切模量 [Pa]

%% ==================== 3. 啮合刚度 ====================
% c' = 14 N/(mm*um) = 1.4e10 N/m^2，ISO 6336 常用区间 14~20
c_prime = 1.4e10;
base = pi*p.m_mod*cos(p.alpha);
p.eps_a_ext = ( sqrt(p.ra1^2-p.rb1^2) + sqrt(p.ra2^2-p.rb2^2) ...
                - p.a*sin(p.alpha) )/base;               % 外啮合端面重合度
p.eps_a_int = ( sqrt(p.ra2^2-p.rb2^2) - sqrt(p.ra3^2-p.rb3^2) ...
                + p.a*sin(p.alpha) )/base;               % 内啮合端面重合度
p.k_single  = c_prime * p.b;                             % 单对齿刚度
p.k_sp = p.k_single * p.eps_a_ext;                       % 外啮合平均刚度
p.k_rp = p.k_single * p.eps_a_int;                       % 内啮合平均刚度

%% ==================== 4. 惯量与质量 ====================
% —— 论文 表2.1 直接给出的值（优先采用）——
%   J_a = 2.5987, J_g = 7.9715, J_H = 38.5660  [kg·mm^2]；M_g = 0.04316 [kg]
p.J_s  = 2.5987e-6;             % 太阳轮转动惯量 J_a
p.J_p  = 7.9715e-6;             % 行星轮转动惯量 J_g
p.J_c1 = 38.5660e-6;            % 行星架转动惯量 J_H
p.m_p  = 0.04316;               % 行星轮质量 M_g

% —— 论文未给的质量/尺寸，按简化几何估算 [假设] ——
Jfun = @(ro, ri) 0.5*p.rho*pi*p.b*(ro^4 - ri^4);   % 薄空心圆盘极转动惯量
mfun = @(ro, ri) p.rho*pi*p.b*(ro^2 - ri^2);       % 圆盘质量

p.d_sun_bore = 0.0030;          % [假设] 太阳轮中心孔
p.d_pl_bore  = 0.0020;          % [假设] 行星轮中心孔
p.d_pin      = 0.0030;          % [假设] 销轴直径
p.d_plate    = 0.048;           % [假设] 行星架端板外径
p.d_outshaft = 0.008;           % [假设] 输出轴直径

p.m_s = mfun(p.r1, p.d_sun_bore/2);         % 太阳轮质量（仅参与径向动力学）

% 销轴
p.r_pin = p.d_pin/2;
p.L_pin = 0.008;                 % [假设] 销轴长度
p.m_pin = p.rho*pi*p.r_pin^2*p.L_pin;
p.J_pin_xx = p.m_pin*(3*p.r_pin^2 + p.L_pin^2)/12;

% 行星架（端板 + 3 根销轴）
p.t_plate = 0.004;               % [假设] 端板厚度
p.R_plate = p.d_plate/2;
p.R_hub   = p.d_outshaft/2;
p.J_plate = 0.5*p.rho*pi*p.t_plate*(p.R_plate^4 - p.R_hub^4);
p.m_plate = p.rho*pi*p.t_plate*(p.R_plate^2 - p.R_hub^2);
p.m_c1 = p.m_plate + p.N*p.m_pin;

% 行星架横向惯量：J_H 是绕 z 轴值，薄盘近似取一半
p.J_c1_xx = p.J_c1/2;

%% ==================== 5. 支承刚度（论文未给，[假设]）====================
p.k_sun_bearing     = 1e8;      % 太阳轮轴承（≈与啮合刚度同量级）
p.k_carrier_bearing = 2e8;      % 行星架轴承
p.k_planet_bearing  = 5e7;      % 行星轮轴承（销轴，径向）
% 抗摆刚度：直齿轮啮合不约束绕 x、y 轴转动，不约束会多出零特征值
p.k_tilt_bearing    = 1e4;      % [N*m/rad]
p.bearing_layout    = 'center'; % 'center'（单轴承）| 'pair'（两端双轴承）

%% ==================== 6. 内齿圈（固定，仅用于啮合阻尼等效质量）====================
p.ring.R_out = 0.048;                        % [假设] 毛坯外半径
p.ring.R_in  = p.r3 + 1.25*p.m_mod;          % 齿根圆半径
p.ring.R_c   = (p.ring.R_in + p.ring.R_out)/2;
p.ring.A     = (p.ring.R_out - p.ring.R_in)*p.b;
p.ring.m     = p.rho*p.ring.A*2*pi*p.ring.R_c;
p.ring.J     = p.ring.m*p.ring.R_c^2;        % 绕 z 轴转动惯量

%% ==================== 7. 阻尼 ====================
% 结构阻尼按 Rayleigh 处理：C_struct = beta*K_struct，beta = 2*zeta_b/w1
% （w1 为装配后最低阶弹性模态；不用 alpha*M，否则整体转动模态会被错误阻尼）
p.damp.zeta_b = 0.02;           % 结构阻尼比
p.tv.zeta_v   = 0.15;           % 啮合阻尼比（式 2.47）

%% ==================== 8. 工况与激励 ====================
% 运行转速：必须与 NGW_init.m 里的 w_ref 一致。
% f_mesh 由它换算，时变啮合刚度/传递误差的基频跟着它走。
p.n_in    = 500;                        % 运行转速 [r/min]（= NGW_init 的 w_ref）
p.omega_s = 2*pi*p.n_in/60;
p.omega_c = p.omega_s/p.i_ratio;
p.f_mesh  = (p.omega_s - p.omega_c)*p.z1/(2*pi);   % 啮频 [Hz]
p.T_in    = 20;                         % 额定输入转矩（静态分析用）[N*m]

% --- 时变啮合刚度 k(t)（式 2.43–2.46）---
p.tv.Nharm = 5;                 % Fourier 阶数，0 = 常数
p.tv.model = 'velex';           % 'velex' | 'square' | 'const'

% --- 传递误差 e(t)（式 2.50–2.52）---
p.tv.E_rot  = 0.20e-6;          % 随轴频变化的等效法向误差幅值 [m]
p.tv.E_mesh = 0.10e-6;          % 随啮频变化的等效法向误差幅值 [m]
p.tv.E_sn  = p.tv.E_rot;   p.tv.Ef_sn = p.tv.E_mesh;   % 外啮合
p.tv.E_rn  = p.tv.E_rot;   p.tv.Ef_rn = p.tv.E_mesh;   % 内啮合
p.tv.phi_s = 0;                 % 太阳轮转频初相位
p.tv.phi_m = 0;                 % 啮频初相位
if mod(p.z2,2) == 0
    p.tv.gamma_sr = pi;         % 式(2.52)：行星轮齿数为偶数
else
    p.tv.gamma_sr = 0;
end
end
