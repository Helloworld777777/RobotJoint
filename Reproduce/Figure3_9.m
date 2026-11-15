clc
clear all
close all
Rs = 1.05;                                                                  % 定子电阻[Ω]
Lq = 1.46e-3;                                                               % 交轴电感[H]
Jm = 105.21e-6;                                                             % 电机转动惯量[Kg·m^2]
Jeq=3.53416e-6;                                                              % 电机侧减速器等效转动惯量[Kg·m^2]
JL=800e-6;                                                                  % 负载转动惯量[Kg·m^2]
i=12;                                                                       % 传动比
Pn = 14;
Flux = 0.01337;
Kt = 3/2 * Pn * Flux;                                                       % 转矩常数
J=Jm+Jeq+JL/(i^2);
Kpc=3;
Kic=2000;
Kps=0.2;
Kis=20;

% 转动惯量变化前后的幅频特性曲线
s = tf('s');
G_PI      = Kps+ Kis/s;
G_current = (Kpc+Kic/s)*(1/(Lq*s+Rs)) /  (1+(Kpc+Kic/s)*(1/(Lq*s+Rs)));
G_mech_1    = Kt / (0.5*J*s);
G_total_1   = G_PI * G_current * G_mech_1;
G_mech_2    = Kt / (J*s);
G_total_2   = G_PI * G_current * G_mech_2;

%% ========= 关键修改：手动定义统一频率 =========
f = logspace(0,5,300);    % 1Hz ~ 1e5 Hz，和论文图横轴保持一致
w = 2*pi*f;               % 转为角rad/s，传给bode

figure('Position', [200, 200, 800, 600]);
set(gcf, 'Color', 'w');

[mag1, phase1, ~] = bode(G_total_1, w);
mag_db_1 = 20*log10(squeeze(mag1));
phase_deg_1 = squeeze(phase1);

[mag2, phase2, ~] = bode(G_total_2, w);
mag_db_2 = 20*log10(squeeze(mag2));
phase_deg_2 = squeeze(phase2);

semilogx(f, mag_db_1, 'r', 'LineWidth', 1.5);
hold on;
semilogx(f, mag_db_2, 'g', 'LineWidth', 1.5);

grid on;
xlabel('频率 (Hz)');
ylabel('幅值 (dB)');
title('图3.9 幅频特性曲线');
legend('转动惯量0.5J','转动惯量J','Location','best');
% xlim([1,1e5]);     % 锁定横轴范围和论文图保持一致
% ylim([-80,100]);   % 锁定纵轴，和论文图上下边界对齐
ax = gca;
ax.FontSize = 12;
