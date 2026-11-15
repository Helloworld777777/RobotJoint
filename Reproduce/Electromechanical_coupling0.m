clc
clear all
close all

%电机部分
P_n = 14;  
phi_f = 0.01337; 
K_t = 3/2 * P_n * phi_f;  

R_s = 1.05;                                                                  % 定子电阻[Ω]
L_d = 1.46e-3;                                                               % 直轴电感[H]
L_q = L_d;       

%机械部分
%等效惯量
J_m = 105.21e-6;                                                             % 电机转动惯量[Kg·m^2]
J_eq= 3.53416e-6;                                                             % 电机侧减速器等效转动惯量[Kg·m^2]
J_L = 4530e-6;  
i=12;   
J=J_m+J_eq+J_L/(i^2);   %谢飞师兄论文公式3.10等式左侧等效惯量之和


%控制参数(暂定)
K_p = 3;
K_i = 2000;

WtoN = 30 / pi;                                                             % rad/s to r/min
NtoW = pi / 30;                                                             % r/min to rad/s

