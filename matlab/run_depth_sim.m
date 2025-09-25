% run_depth_sim.m
% 主仿真脚本 (Main simulation harness) for depth hold tuning
% 运行前先执行 plant_params.m 初始化参数

if ~exist('plant','var')
    error('请先运行 plant_params.m (Run plant_params.m first)');
end

dt = sim.dt; t = sim.t; Nt = numel(t);

% 状态 (states)
z = 0;          % 初始深度 m (surface)
v = 0;          % 初始速度 m/s

% 数据记录 (logging)
log = struct();
log.t = t; log.z = zeros(1,Nt); log.v = zeros(1,Nt); log.v_ref = zeros(1,Nt);
log.pwm = zeros(1,Nt); log.T = zeros(1,Nt); log.ez = zeros(1,Nt); log.ev = zeros(1,Nt);

% 估计器 (EMA) 与测量噪声 (noise)
alpha = 0.08; z_filt = z; z_prev_filt = z; v_est = 0;
noise_std = 0.05; % 5 cm 标准差

% 控制结构 (copy controller struct)
ctrl_local = ctrl; mode = 'APPROACH';

for k = 1:Nt
    tk = t(k);

    % 任务阶段简单逻辑 (simple mission-like mode switch)
    if strcmp(mode,'APPROACH')
        if abs(z - sim.z_target) < 0.30
            mode = 'HOLD';
        end
    end

    % 估计器更新 (Estimator update)
    z_meas = z + noise_std * randn();
    z_filt = z_filt + alpha * (z_meas - z_filt);
    % 原始差分速度
    v_raw = (z_filt - z_prev_filt)/dt;
    z_prev_filt = z_filt;
    % 一阶低通滤波 (LPF) 抑制噪声放大
    if k == 1
        v_est = v_raw; % 初始化
    else
        beta_v = 0.2; % 0.1~0.3 可调权衡响应与降噪
        v_est = v_est + beta_v * (v_raw - v_est);
    end

    % 控制器 (Controller)
    [pwm_cmd, v_ref, ctrl_local] = depth_controller(z_filt, v_est, struct('plant',plant,'sim',sim), ctrl_local, mode);

    % 推力模型 (Thrust model)
    pwm_delta = pwm_cmd - plant.pwm_neutral;
    if pwm_delta >= 0
        T = min(pwm_delta * plant.k_t, plant.T_fwd_max);
    else
        T = max(pwm_delta * plant.k_t, -plant.T_rev_max);
    end

    % 连续动力学离散积分 (Integrate dynamics)
    drag_lin = plant.d1 * v;
    drag_quad = plant.d2 * abs(v) * v;
    dv = (T - drag_lin - drag_quad - plant.F_buoy) / plant.m_eff;
    v = v + dv * dt;
    z = z + v * dt;  % 深度向下为正 (positive downward)

    % 记录 (Log)
    log.z(k)=z; log.v(k)=v; log.pwm(k)=pwm_cmd; log.T(k)=T; log.v_ref(k)=v_ref;
    log.ez(k)=sim.z_target - z; log.ev(k)=v_ref - v;
end

%% 绘图 (Plots)
figure('Name','Depth Control Simulation');
subplot(3,1,1); plot(t, log.z,'b', t, sim.z_target*ones(size(t)),'r--'); grid on;
ylabel('Depth z [m]'); legend('z','target'); title('Depth Response');

subplot(3,1,2); plot(t, log.v,'b', t, log.v_ref,'m--'); grid on;
ylabel('Velocity [m/s]'); legend('v','v_{ref}');

subplot(3,1,3);
plot(t, log.pwm,'b');
grid on;
ylabel('PWM');
xlabel('Time [s]');
title('PWM Command');

%% 性能指标 (Performance metrics)
settle_idx = find(abs(log.z - sim.z_target) < 0.3,1,'first');
if ~isempty(settle_idx)
    fprintf('初次进入 ±0.3m 时间: %.2f s\n', t(settle_idx));
else
    fprintf('未在仿真窗口内进入 ±0.3m 带宽\n');
end

overshoot = max(log.z) - sim.z_target;
fprintf('最大超调 (overshoot) = %.3f m\n', overshoot);
