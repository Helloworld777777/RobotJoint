%% 绘制图 4.18 和图 4.19
% 电流环采用论文的一阶近似：
% Gc(s) = 1 / (1 + 3*Tpwm*s)

clear;
clc;
close all;

%% 1. 基本参数
Tpwm = 1/10000;       % 逆变器开关周期 [s]
miu=0.0079;
Jm = 105.21e-6;  
JL = 4530e-6;                                                                 % 负载转动惯量[Kg·m^2]
i=12;     
Jeq=Jm+3.53416e-6+(1/(i^2)+miu/i)*JL;        % 等效转动惯量 [kg*m^2]
Pn=14;
Flux = 0.01337;                                                             % 电机磁通量[Wb]
Kt = 3/2 * Pn * Flux;   
TLstep = 3.0;         % 负载转矩阶跃幅值 [N*m]

% 注意：
% Jeq、Kt 和 TLstep 应替换成论文或实际模型中的数据。
% 当前数值只是为了让代码能够直接运行。

%% 2. 图 4.18：速度环开环伯德图
deltaSets = {
    [0.5, 1, 2], ...
    [2, 4, 6, 8, 10, 20, 30]
};

freqRanges = {
    [1e1, 1e6], ...
    [1e-1, 1e5]
};

figure('Color', 'w', 'Name', 'Figure 4.18');

for panel = 1:2
    f = logspace(log10(freqRanges{panel}(1)), ...
                 log10(freqRanges{panel}(2)), 4000).';

    w = 2*pi*f;
    s = 1j*w;
    deltas = deltaSets{panel};

    axMag = subplot(2, 2, 2*panel-1);
    hold(axMag, 'on');

    axPhase = subplot(2, 2, 2*panel);
    hold(axPhase, 'on');

    for k = 1:numel(deltas)
        delta = deltas(k);

        % 积分系数
        Ki = 1/(3*Tpwm*delta^2);

        % 代入 Kp 后，K = Kt*Kp/Jeq
        K = 1/(3*Tpwm*delta);

        % 速度环开环传递函数
        % Gopen(s) = K*(s+Ki) / [s^2*(1+3*Tpwm*s)]
        Gopen = K*(s+Ki) ./ ...
            (s.^2 .* (1 + 3*Tpwm*s));

        magDB = 20*log10(abs(Gopen));

        % Keep the branch near -180 deg.  A plain angle() call maps the
        % delta<1 branch (-180 to -220 deg) to +140 to +180 deg, so the
        % delta=0.5 phase curve would disappear outside the figure.
        phaseDeg = angle(Gopen)*180/pi;
        phaseDeg(phaseDeg > 0) = phaseDeg(phaseDeg > 0) - 360;

        label = sprintf('\\delta = %g', delta);

        semilogx(axMag, f, magDB, ...
            'LineWidth', 1.4, ...
            'DisplayName', label);

        semilogx(axPhase, f, phaseDeg, ...
            'LineWidth', 1.4, ...
            'DisplayName', label);
    end

    if panel == 1
        panelText = '(a) \delta = [0.5, 1, 2]';
    else
        panelText = '(b) \delta = [2, 4, 6, 8, 10, 20, 30]';
    end

    % 幅频图
    title(axMag, [panelText, ' 幅频特性']);
    xlabel(axMag, '频率 (Hz)');
    ylabel(axMag, '幅值 (dB)');

    yline(axMag, 0, '--', ...
        'Color', [0.45, 0.45, 0.45], ...
        'HandleVisibility', 'off');

    set(axMag, 'XScale', 'log');
    xlim(axMag, freqRanges{panel});
    grid(axMag, 'on');
    grid(axMag, 'minor');
    legend(axMag, 'Location', 'northeast');

    % 相频图
    title(axPhase, [panelText, ' 相频特性']);
    xlabel(axPhase, '频率 (Hz)');
    ylabel(axPhase, '相位 (deg)');

    yline(axPhase, -180, '--', ...
        'Color', [0.45, 0.45, 0.45], ...
        'HandleVisibility', 'off');

    set(axPhase, 'XScale', 'log');
    xlim(axPhase, freqRanges{panel});
    grid(axPhase, 'on');
    grid(axPhase, 'minor');
    legend(axPhase, 'Location', 'northeast');

    if panel == 1
        ylim(axMag, [-100, 100]);
        ylim(axPhase, [-220, -140]);
    else
        ylim(axMag, [-100, 150]);
        ylim(axPhase, [-180, -80]);
    end
end

%% 3. 图 4.19：闭环阶跃响应和负载扰动响应
deltaTime = [2, 4, 6, 8, 10, 20, 30];

t = linspace(0, 0.2, 6000);

figure('Color', 'w', 'Name', 'Figure 4.19');

axStep = subplot(1, 2, 1);
hold(axStep, 'on');

axLoad = subplot(1, 2, 2);
hold(axLoad, 'on');

for k = 1:numel(deltaTime)
    delta = deltaTime(k);

    Ki = 1/(3*Tpwm*delta^2);

    Kp = Jeq/(3*Tpwm*delta*Kt);

    % 速度环 PI：
    % Cpi(s) = Kp*(1 + Ki/s) = Kp*(s+Ki)/s
    Cpi = tf(Kp*[1, Ki], [1, 0]);

    % 电流环一阶近似
    Gcurrent = tf(1, [3*Tpwm, 1]);

    % 速度误差到 iq 的控制器
    C = Cpi*Gcurrent;

    % 转矩到转速的机械环节：1/(Jeq*s)
    P = tf(1, [Jeq, 0]);

    % 速度给定到实际转速的闭环传递函数
    Gref = minreal(feedback(P*Kt*C, 1));

    % 负载转矩到实际转速的传递函数
    % 机械方程：Jeq*dw/dt = Te - TL
    Gload = minreal(-feedback(P, Kt*C));

    % 单位速度阶跃响应
    [yStep, ~] = step(Gref, t);

    % 单位负载转矩阶跃响应
    yLoad = lsim(Gload, TLstep*ones(size(t)), t);

    label = sprintf('\\delta = %g', delta);

    plot(axStep, t, yStep, ...
        'LineWidth', 1.4, ...
        'DisplayName', label);

    plot(axLoad, t, yLoad, ...
        'LineWidth', 1.4, ...
        'DisplayName', label);
end

% 阶跃响应图
title(axStep, '(a) 速度环闭环系统阶跃响应');
xlabel(axStep, '时间 (s)');
ylabel(axStep, '转速');

yline(axStep, 1, '--', ...
    'Color', [0.45, 0.45, 0.45], ...
    'HandleVisibility', 'off');

xlim(axStep, [0, 0.15]);
grid(axStep, 'on');
legend(axStep, 'Location', 'northeast');

% 负载扰动响应图
title(axLoad, '(b) 速度环闭环系统抗负载扰动响应');
xlabel(axLoad, '时间 (s)');
ylabel(axLoad, '转速变化');

yline(axLoad, 0, '--', ...
    'Color', [0.45, 0.45, 0.45], ...
    'HandleVisibility', 'off');

xlim(axLoad, [0, 0.2]);
grid(axLoad, 'on');
legend(axLoad, 'Location', 'southeast');

%% 4. 可选：导出高清图片
% exportgraphics(figure(1), 'fig_4_18.png', 'Resolution', 600);
% exportgraphics(figure(2), 'fig_4_19.png', 'Resolution', 600);