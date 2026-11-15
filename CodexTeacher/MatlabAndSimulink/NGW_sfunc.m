function NGW_sfunc(block)
%NGW_SFUNC  3 行星 NGW 机械系统（Level-2 MATLAB S-Function，30 DOF）
%
%   ============================ 接口 ============================
%   输入 1  Te  [N*m]  电磁转矩，作用于太阳轮 θs（正号 = 驱动）
%   输入 2  TL  [N*m]  负载阻力矩，作用于行星架 θc（正号 = 阻力）
%
%   输出 1  [θc; ωc]              (2×1)  行星架输出
%   输出 2  [θs; ωs]              (2×1)  太阳轮输出（后续接电机反馈）
%   输出 3  [q; v]                (60×1) 全部状态：30 位移 + 30 速度
%   输出 4  [Fsp1..3; Frp1..3]    (6×1)  外/内啮合力 [N]
%
%   参数    1 个：在 S-Function 的 Parameters 里填  NGW_params(J_rotor, J_load)
%           （J_rotor = 电机转子惯量，并入太阳轮；J_load = 输出负载惯量，并入行星架）
%           不接电机/负载时可直接填 NGW_params()，两者默认均为 0。
%   状态    60 个连续状态，初始全零
%   方程    见 NGW_dynamics.m（M、K、C 与啮合激励都在那里）
%
%   典型接法：Te ← 电机（或常数）；TL ← 负载（或 cL*ωc 的 Gain）；
%             ωs（输出 2 的第 2 个元素）→ 电机转速反馈。
%   ==============================================================

setup(block);

end

%% ======================= 端口与状态注册 =======================
function setup(block)
block.NumDialogPrms = 1;                     % 参数：p = NGW_params()
% block.DialogPrmsTunable = {'Nontunable'};

block.NumInputPorts  = 2;
block.NumOutputPorts = 4;

% 输入：Te、TL
for k = 1:2
    block.InputPort(k).Dimensions        = 1;
    block.InputPort(k).DatatypeID        = 0;    % double
    block.InputPort(k).Complexity        = 'Real';
    block.InputPort(k).DirectFeedthrough = false;
    block.InputPort(k).SamplingMode      = 'Sample';
end

% 输出：行星架、太阳轮、全状态、啮合力
outs = [2 2 60 6];
for k = 1:4
    block.OutputPort(k).Dimensions   = outs(k);
    block.OutputPort(k).DatatypeID   = 0;
    block.OutputPort(k).Complexity   = 'Real';
    block.OutputPort(k).SamplingMode = 'Sample';
end

block.NumContStates = 60;                    % 30 位移 + 30 速度
block.SampleTimes   = [0 0];                 % 连续系统
block.SimStateCompliance = 'DefaultSimState';

block.RegBlockMethod('Start',                @Start);
block.RegBlockMethod('InitializeConditions', @InitCond);
block.RegBlockMethod('Outputs',              @Outputs);
block.RegBlockMethod('Derivatives',          @Derivatives);
end

%% ======================= 初始化 =======================
function Start(block) %#ok<INUSD>
p = block.DialogPrm(1).Data;
NGW_dynamics(0, zeros(60,1), p, 0, 0);       % 预热：常量矩阵只算这一次
end

function InitCond(block)
block.ContStates.Data = zeros(60,1);         % 静止启动
end

%% ======================= 输出 =======================
function Outputs(block)
p  = block.DialogPrm(1).Data;
x  = block.ContStates.Data;
Te = block.InputPort(1).Data;
TL = block.InputPort(2).Data;

[~, aux] = NGW_dynamics(block.CurrentTime, x, p, Te, TL);

block.OutputPort(1).Data = [aux.theta_c; aux.omega_c];
block.OutputPort(2).Data = [aux.theta_s; aux.omega_s];
block.OutputPort(3).Data = x;
block.OutputPort(4).Data = [aux.Fsp; aux.Frp];
end

%% ======================= 状态导数 =======================
function Derivatives(block)
p  = block.DialogPrm(1).Data;
x  = block.ContStates.Data;
Te = block.InputPort(1).Data;
TL = block.InputPort(2).Data;

block.Derivatives.Data = NGW_dynamics(block.CurrentTime, x, p, Te, TL);
end
