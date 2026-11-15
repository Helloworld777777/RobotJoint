function NGW_build_model()
%NGW_BUILD_MODEL  生成**纯机械基线模型** NGW_mech_only.slx。
%
%   ⚠ 本函数不生成、也不覆盖 NGW_model.slx（那个是含电机 + FOC 的完整模型，
%     由 Simulink 手工搭建，请勿用本脚本重建）。
%
%   生成内容：
%     Te (Constant) ──┐
%                     ├─> NGW_mech (Level-2 S-Function, NGW_sfunc)
%     Load_cL (Gain) ─┘        │
%                              ├─ out1 [theta_c; omega_c] -> Take_wc(Demux2) -> Load_cL -> TL
%                              ├─ out2 [theta_s; omega_s] -> log_sun
%                              ├─ out3 [q; v] (60)        -> log_state
%                              └─ out4 [F_sp; F_rp] (6)   -> log_mesh
%
%   用法：  NGW_build_model;  然后 open_system('NGW_mech_only')

mdl = 'NGW_mech_only';
here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
old = pwd; cd(here);
cleanup = onCleanup(@() cd(old)); %#ok<NASGU>

if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist([mdl '.slx'], 'file'), delete([mdl '.slx']); end

NGW_init();      % 必须先定义 J_rotor 等变量，否则 S-Function 的 Parameters 求值失败，
                 % setup() 不会被调用，端口也就不会 populate

new_system(mdl);

% ---------------- 模块 ----------------
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', [mdl '/NGW_mech'], ...
    'Position', [260 100 360 180]);
set_param([mdl '/NGW_mech'], 'FunctionName', 'NGW_sfunc', 'Parameters', 'NGW_params()');

add_block('simulink/Sources/Constant', [mdl '/Te'], ...
    'Position', [100 80 160 120], 'Value', 'Te_cmd');

% S-Function 的端口数量要等第一次 update 之后才由 setup() 确定。
% 先只接输入 1，update 一次让端口就位，再补其它接线。
ph = @(b, k) get_param(get_param([mdl '/' b], 'PortHandles').Outport(k), 'Handle');
dh = @(b, k) get_param(get_param([mdl '/' b], 'PortHandles').Inport(k), 'Handle');

add_line(mdl, ph('Te', 1), dh('NGW_mech', 1), 'autorouting', 'on');
try
    set_param(mdl, 'SimulationCommand', 'update');   % 触发 setup()，端口就位
catch
    % 此时输入 2 尚未连接，update 会报错，但端口已被 populate，忽略即可
end
phN = get_param([mdl '/NGW_mech'], 'PortHandles');
if numel(phN.Inport) < 2 || numel(phN.Outport) < 4
    error('NGW_build_model:portNotReady', ...
          'S-Function 端口未就位（in=%d out=%d），请确认 NGW_sfunc.m 在路径上且 J_rotor 已定义', ...
          numel(phN.Inport), numel(phN.Outport));
end

add_block('simulink/Math Operations/Gain', [mdl '/Load_cL'], ...
    'Position', [430 250 470 290], 'Gain', 'cL');

add_block('simulink/Signal Routing/Demux', [mdl '/Take_wc'], ...
    'Position', [430 120 440 160], 'Outputs', '2');

add_block('simulink/Sinks/To Workspace', [mdl '/log_out'], ...
    'Position', [520 110 600 150], 'VariableName', 'log_out', 'SaveFormat', 'timeseries');
add_block('simulink/Sinks/To Workspace', [mdl '/log_sun'], ...
    'Position', [520 180 600 220], 'VariableName', 'log_sun', 'SaveFormat', 'timeseries');
add_block('simulink/Sinks/To Workspace', [mdl '/log_state'], ...
    'Position', [520 250 600 290], 'VariableName', 'log_state', 'SaveFormat', 'timeseries');
add_block('simulink/Sinks/To Workspace', [mdl '/log_mesh'], ...
    'Position', [520 320 600 360], 'VariableName', 'log_mesh', 'SaveFormat', 'timeseries');

% ---------------- 连线 ----------------
add_line(mdl, ph('NGW_mech', 1),  dh('Take_wc', 1),  'autorouting', 'on');  % [thc;wc]
add_line(mdl, ph('Take_wc', 2),   dh('Load_cL', 1),  'autorouting', 'on');  % wc
add_line(mdl, ph('Load_cL', 1),   dh('NGW_mech', 2), 'autorouting', 'on');  % TL  -> 行星架
add_line(mdl, ph('NGW_mech', 1),  dh('log_out', 1),  'autorouting', 'on');
add_line(mdl, ph('NGW_mech', 2),  dh('log_sun', 1),  'autorouting', 'on');
add_line(mdl, ph('NGW_mech', 3),  dh('log_state', 1),'autorouting', 'on');
add_line(mdl, ph('NGW_mech', 4),  dh('log_mesh', 1), 'autorouting', 'on');

% ---------------- 求解器与初始化 ----------------
set_param(mdl, 'Solver', 'ode15s', 'SolverType', 'Variable-step', ...
    'StartTime', '0', 'StopTime', '0.3', ...
    'MaxStep', '2e-4', 'MinStep', 'auto', 'RelTol', '1e-7', 'AbsTol', '1e-9', ...
    'SaveFormat', 'Dataset', 'SaveOutput', 'on', 'ReturnWorkspaceOutputs', 'on');
set_param(mdl, 'InitFcn', 'NGW_init');

save_system(mdl);
fprintf('已生成 %s.slx（纯机械基线）\n', mdl);
fprintf('运行： out = sim(''%s'');  plot(out.log_out.Time, out.log_out.Data(:,2))\n', mdl);
end


