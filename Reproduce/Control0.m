clc
clear all
close all


i=12;                                                                       % 传动比




%%%%%%%%
U_dc=72;
R_s = 1.05;                                                                  % 定子电阻[Ω]
L_d = 1.46e-3;                                                               % 直轴电感[H]
L_q = L_d;       

P_n = 14;  
phi_f = 0.01337; 
K_t = 3/2 * P_n * phi_f;  

WtoN = 30 / pi;                                                             % rad/s to r/min
NtoW = pi / 30;                                                             % r/min to rad/s

Kpc=3;
Kic=2000;
Kps=0.15;
Kis=10;

%机械部分
%等效惯量
miu=0.0079;
T_c0=0.0104;
f_v=1.7086e-04;


J_m = 105.21e-6;     
J_eq= 3.53416e-6;                                                             % 电机侧减速器等效转动惯量[Kg·m^2]
J_L = 4530e-6;                                                                 % 负载转动惯量[Kg·m^2]
J=J_m+J_eq+J_L/(i^2);                                                          % 整个系统等效转动惯量[Kg·m^2]
J_total=J_m+J_eq+(1/(i^2)+miu/i)*J_L;                                               % 加入摩擦模型的等效转动惯量[Kg·m^2]

%SVPWM
TPWM = 1/10000;                                                             % 逆变器环节时间常数[s]


% PI控制器参数设计
delta = 6;
Kpc_2=L_q/(3*TPWM);
Kic_2=R_s/L_q;
Kis_2=1/(3*TPWM*delta^2);
Kps_2=J_total/(3*TPWM*delta*K_t);