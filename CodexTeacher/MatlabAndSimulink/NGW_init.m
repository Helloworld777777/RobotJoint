function NGW_init()
%NGW_INIT  电机 + FOC 控制参数初始化，NGW_model.slx 的 InitFcn 调用本函数。
%
%   模型里哪些块依赖这里定义的变量：
%     wm_ref   (Constant)      -> w_ref      太阳轮转速指令 [r/min]
%     Gain     (控制系统)       -> P_n        极对数
%     Gain3    (速度环)         -> NtoW       r/min -> rad/s
%     PID 1/2  (电流环)         -> Kpc, Kic
%     PID 7    (速度环)         -> Kps, Kis
%     DC src   (Inverter)      -> Amplitude  直流母线电压 [V]
%     Load_cL  (顶层)           -> cL        TL = cL*omega_c
%     S-Function 参数           -> J_rotor   转子惯量，NGW_params(J_rotor) 的入参
%
%   电机与减速器同轴：转子惯量直接并入太阳轮绕 z 轴惯量。
%   改任何一个参数只改本文件（机械参数在 NGW_params.m）。
%
%   参数来源：论文 表4.6（谢飞，足式机器人关节动力学建模与控制研究）
%     Rs = 1.05 Ω,  Ld = Lq = 1.46 mH,  ψf = 0.01337 Wb,  Pn = 14,
%     Jm = 105.21 kg·mm^2 = 1.0521e-4 kg·m^2,  JL = 4.53 kg·cm^2 = 4.53e-4 kg·m^2,
%     Udc = 72 V,  开关频率 10 kHz

%% ---------- 1. 机械端：转子惯量 + 负载惯量 ----------
J_rotor = 1.0521e-4;    % [kg*m^2] 论文表4.6  Jm = 105.21 kg·mm^2
J_load  = 4.53e-4;      % [kg*m^2] 论文表4.6  JL = 4.53 kg·cm^2

%% ---------- 2. 驱动与母线 ----------
Amplitude = 72;         % [V]   直流母线电压（论文表4.6）
w_ref     = 500;        % [r/min] 太阳轮（电机轴）转速指令（论文 4.3 节阶跃工况）
P_n       = 14;         % [-]   极对数（论文表4.6）
NtoW      = 2*pi/60;    % [rad/s per r/min]
cL        = 0;          % [N*m*s/rad] 阻尼负载系数；论文工况1 为“空载启动”，取 0

% 反电动势校核：psi=0.01337 Wb，500 r/min 时
%   w_e = Pn*2*pi*500/60 = 733.0 rad/s，e_peak = psi*w_e = 9.80 V < 41.57 V（可达到）
%   极限转速 ≈ 2120 r/min（线性调制），500 r/min 远在线性区内

%% ---------- 3. 电流环（IMC 整定，带宽 500 Hz）----------
% Kp = wc*L,  Ki = wc*R；Ld = Lq = 1.46 mH，R = 1.05 Ohm（论文表4.6）
wc_i = 2*pi*500;
Kpc  = wc_i*1.46e-3;    % [V/A]   = 4.587
Kic  = wc_i*1.05;       % [V/(A*s)] = 3299

%% ---------- 4. 速度环（带宽 20 Hz，IMC 整定 + 负载极点对消）----------
% 被控对象： w_s(s)/i_q(s) = Kt / (J_eq*s + c_eq)
%   Kt   = 1.5*P_n*Flux                            [N*m/A]
%   J_eq = J_s + J_rotor + 行星轮系折算惯量          [kg*m^2]
%   c_eq = cL/i^2  （阻尼负载折算到太阳轮）          [N*m/(rad/s)]
% PI：  Ti = J_eq/c_eq（零点对消负载极点），
%       Kp = wc*J_eq/Kt，  Ki = Kp/Ti  ->  开环 = wc/s（PM ~90°）
% 注：这里 Kp/Ki 必须除以 Kt，且 Ki ≠ Kp*0.01，否则会产生很大的稳态误差。
Flux  = 0.01337;                    % [Wb] 永磁磁链（论文表4.6）
Kt    = 1.5*P_n*Flux;               % [N*m/A] 转矩常数 = 0.28077
p_m   = NGW_params(J_rotor, J_load);
J_ref = (p_m.J_c1 + 3*(p_m.J_p + p_m.m_p*p_m.a^2))/p_m.i_ratio^2;   % 轮系折算到太阳轮
J_eq  = p_m.J_s + J_rotor + J_ref;  % [kg*m^2]
c_eq  = cL/p_m.i_ratio^2;           % [N*m/(rad/s)]
wc_s  = 2*pi*20;
Kps   = wc_s*J_eq/Kt;               % [A/(rad/s)]
% 积分时间常数：有阻尼负载时用极点对消 Ti = J_eq/c_eq；
% 空载（c_eq = 0）时取 Ti = 4/wc_s（积分零点在 wc/4）
if c_eq > 0
    Ti_s = J_eq/c_eq;
else
    Ti_s = 4/wc_s;
end
Kis   = Kps/Ti_s;                   % [A/(rad*s)]

%% ---------- 5. 负载（线性阻尼负载，TL = cL*omega_c）----------
% （cL 已在上面定义，速度环用它折算 c_eq）

%% ---------- 6. 工况2：负载阶跃（论文 4.3 节：0.2 s 时 6 N·m 阶跃）----------
TL_t0  = 0.2;           % [s]   阶跃时刻
TL_amp = 6;             % [N*m] 阶跃幅值（作用于行星架/关节输出）

%% ---------- 7. 纯机械基线模型 NGW_mech_only.slx 仍用这两个 ----------
Te_cmd = 2.0;           % [N*m] 常值驱动转矩

%% ---------- 7. 写入 base 工作区 ----------
i_ratio = p_m.i_ratio;  % 传动比，供模型里 TL/i 折算增益使用
names = {'J_rotor','J_load','Amplitude','w_ref','P_n','NtoW','Kpc','Kic','Kps','Kis','cL','Te_cmd', ...
         'Flux','Kt','J_eq','c_eq','wc_s','i_ratio','TL_t0','TL_amp'};
for k = 1:numel(names)
    assignin('base', names{k}, eval(names{k}));
end

end
