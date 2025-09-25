% plant_params.m
% 深度控制一维纵向(heave)简化模型参数 (Depth 1-DOF Heave Model Parameters)
% 所有单位统一为 SI: m, s, N, kg
% 中文+英文关键词 (Chinese + English keywords)

clear; clc;

%% 几何与物理参数 (Geometry & Physical Parameters)
% 质量 (dry mass) m [kg]
m = 2.08;                     % TODO: 若有称重数据替换 (replace with measured mass)
% 附加质量 (added mass) m_a [kg]
m_a = 0.267;                 % 来自初步估计 (from initial estimate)
m_eff = m + m_a;             % 有效质量 effective mass

% 浮力 (Buoyancy) B 与 重力 (Weight) W 以“等效 kg”给出 -> 转为牛顿
B_kg = 2.1;  % displaced water mass equivalent [kg]
W_kg = m;  % vehicle mass in air (approx) [kg]
g = 9.80665; % 重力加速度
B = B_kg * g;    % 浮力 [N]
W = W_kg * g;    % 重力 [N]
F_buoy = (B - W); % 净上浮力 (net upward force) [N], 若>0表示上浮趋势 upward tendency

% 线性 / 二次阻力 (Linear / Quadratic drag)
d1 = 2.71;    % [N·s/m]
d2 = 1.42;    % [N·s^2/m^2]

%% 推进器模型 (Thruster Model)
% PWM -> 推力 (thrust) 近似线性区间: 以中立点 (neutral) 为 1500
% 定义缩放: u_pwm ∈ [pwm_min, pwm_max]; 令 u = (pwm - pwm_neutral)
% 假设: 最大正向推力 ~ T_max, 负向(上行)推力 ~ -T_min (可不同)
pwm_neutral = 1500;
pwm_min = 1000; pwm_max = 2000;          % 硬件限制
T_forward_max = 0.2*g;  % [N] TODO: 替换为实测 thrust test in water
T_reverse_max = 0.4*g;   % [N]

% 线性化系数 (thrust gain) k_t 以对称近似: 每 PWM count 对应 N
k_t_forward = T_forward_max / (pwm_max - pwm_neutral);
k_t_reverse = T_reverse_max / (pwm_neutral - pwm_min);
% 简化使用平均值 (average)
k_t = 0.5*(k_t_forward + k_t_reverse);

%% 控制周期 (Control period)
dt = 0.01;                % 10 ms，与固件保持一致 (match firmware loop)
sim_time = 40.0;          % 仿真总时长 [s]
t_vec = 0:dt:sim_time;    % 时间向量

%% 速度内环线性化 (Velocity loop linearization)
% 原连续模型: (m_eff) dv/dt + d1 v + d2 |v| v + (B-W) = T
% 在小速度附近忽略二次阻力 & 浮力通过配平推力 T0 = (B-W)
% 线性化内环传递函数: G_v(s) = 1 / (m_eff s + d1)
tau_v = m_eff / d1;  % 一阶时间常数 (time constant)
K_v = 1 / d1;        % 静态增益 (static gain)

%% 速度 PI 设计 (Velocity PI design via IMC)
% PI: C_v(s) = Kp_v + Ki_v / s
% IMC公式 (无延迟 first-order plant): Kp = tau_v / (K_v * lambda), Ki = Kp / tau_v
lambda_v = 0.1;  % 期望闭环时间常数 (desired closed-loop time constant) < tau_v
Kp_v_thrust = tau_v / (K_v * lambda_v)*0.45;
Ki_v_thrust = Kp_v_thrust / tau_v*0.7;
% 转为 PWM 域 (divide by k_t)
Kp_v_pwm = Kp_v_thrust / k_t;
Ki_v_pwm = Ki_v_thrust / k_t;

%% 深度外环 (Depth outer loop) 近似
% 内环收敛后，视 v ≈ v_ref, 深度为一重积分 (integrator)
% 选深度闭环时间常数 lambda_z >> lambda_v (带宽分离 bandwidth separation) 以此保证v ≈
% v_ref，控制系统的深度响应速度，越小响应越快，lambda_z=2.0,表明期望系统在2s内完成整个行程（如需要从3m到4m）的63.2%
% 采用 PI 对象: ez = z_target - z
% 经验：Kp_z 约 = (1 / tau_z) * (scale_vref_max / depth_error_band)
% 直接采用 IMC 对积分器 (G_z(s)=1/s) 的近似：设计闭环 1/(lambda_z s +1)
lambda_z = 2.0; 
Kp_z = 1 / lambda_z;               % 等效使得 v_ref ≈ Kp_z * ez (简单比例)
Ki_z = 0;%%0.3 * Kp_z / lambda_z;      % 保守积分 (conservative integral)

% 限幅与斜率 (Limits & Slew)
v_ref_max_app = 0.1;    % 与固件 CTRL_V_REF_MAX_APP 对齐
v_ref_max_hold = 0.05;   % 与固件 CTRL_V_REF_MAX_HOLD
v_ref_slew = 0.01;       % per step
pwm_slew_per_tick = 5;  

%% 目标 (Target)
z_target = 2.0;  % m 深度 (positive downward)

%% 打包参数到结构 (Pack into struct)
plant = struct('m',m,'m_a',m_a,'m_eff',m_eff,'d1',d1,'d2',d2,'F_buoy',F_buoy, ...
    'k_t',k_t,'pwm_neutral',pwm_neutral,'pwm_min',pwm_min,'pwm_max',pwm_max, ...
    'T_fwd_max',T_forward_max,'T_rev_max',T_reverse_max);

ctrl = struct();
ctrl.inner = struct('Kp',Kp_v_pwm,'Ki',Ki_v_pwm,'integ',0,'v_ref',0,'v_ref_prev',0);
ctrl.outer = struct('Kp',Kp_z,'Ki',Ki_z,'integ',0);
ctrl.limits = struct('v_ref_max_app',v_ref_max_app,'v_ref_max_hold',v_ref_max_hold, ...
    'v_ref_slew',v_ref_slew,'pwm_slew',pwm_slew_per_tick, ...
    'dir_thresh_pwm',40,'neutral_hold_steps',5); % 方向切换阈值与保持周期 (direction reversal threshold & neutral dwell)
ctrl.modes = struct('mode','APPROACH'); % or HOLD

sim = struct('dt',dt,'t',t_vec,'z_target',z_target);

%% 显示关键初值 (Display key initial values)
disp('--- Plant & Controller Key Parameters ---');
fprintf('m_eff = %.3f kg\n', m_eff);
fprintf('tau_v = %.3f s, K_v = %.3f 1/N\n', tau_v, K_v);
fprintf('Velocity PI (PWM domain): Kp_v=%.3f Ki_v=%.3f\n', Kp_v_pwm, Ki_v_pwm);
fprintf('Depth PI: Kp_z=%.3f Ki_z=%.3f\n', Kp_z, Ki_z);
fprintf('Net buoyancy F_buoy = %.3f N (positive=upward)\n', F_buoy);

% 保存到工作区 (variables remain in base workspace for subsequent scripts)
