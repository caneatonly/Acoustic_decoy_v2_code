% depth_controller.m
% 串级 PI 控制实现 (Cascaded PI control implementation)
% 输入: 当前深度 z, 速度 v, 目标深度 z_target, 模式 mode ('APPROACH'/'HOLD')
% 输出: pwm_cmd, v_ref

function [pwm_cmd, v_ref, ctrl] = depth_controller(z, v, params, ctrl, mode)

% 选择速度参考最大值 (select v_ref max)
if strcmp(mode,'APPROACH')
    v_ref_max = ctrl.limits.v_ref_max_app;
else
    v_ref_max = ctrl.limits.v_ref_max_hold;
end

dt = params.sim.dt;
pwm_neutral = params.plant.pwm_neutral;
% 预先计算 PWM 跨度供条件积分 / 限幅使用
pwm_span = (params.plant.pwm_max - params.plant.pwm_min);

% -------- 外环 (Depth outer loop) -------- %
ez = params.sim.z_target - z;  % 深度误差 (depth error): positive -> need go deeper
ctrl.outer.integ = ctrl.outer.integ + ctrl.outer.Ki * ez * dt; % 积分 (integrator)
% 防风up 积分限幅 (anti-windup clamp) -> 限在速度参考范围
ctrl.outer.integ = max(min(ctrl.outer.integ, v_ref_max), -v_ref_max);

v_ref_cmd = ctrl.outer.Kp * ez + ctrl.outer.integ;
% 限幅 (clamp)
v_ref_cmd = max(min(v_ref_cmd, v_ref_max), -v_ref_max);

% 斜率限制 (slew rate)
dv = v_ref_cmd - ctrl.inner.v_ref_prev;
max_slew = ctrl.limits.v_ref_slew;
if dv > max_slew, dv = max_slew; elseif dv < -max_slew, dv = -max_slew; end
ctrl.inner.v_ref = ctrl.inner.v_ref_prev + dv;
ctrl.inner.v_ref_prev = ctrl.inner.v_ref;
v_ref = ctrl.inner.v_ref;

% -------- 内环 (Velocity inner loop) -------- %
ev = v_ref - v;
% 条件积分：若即将饱和并且误差继续推动同方向则暂停积分
projected = pwm_neutral + ctrl.inner.Kp * ev + ctrl.inner.integ; % 预测当前输出
at_high = (projected > (params.plant.pwm_max - 0.05*pwm_span)) && (ev > 0);
at_low  = (projected < (params.plant.pwm_min + 0.05*pwm_span)) && (ev < 0);
if ~(at_high || at_low)
    ctrl.inner.integ = ctrl.inner.integ + ctrl.inner.Ki * ev * dt;
end
% 可选积分限幅（目前关闭，可按需启用）
% ctrl.inner.integ = max(min(ctrl.inner.integ, 0.5*pwm_span), -0.5*pwm_span);

pwm_cmd_f = pwm_neutral + ctrl.inner.Kp * ev + ctrl.inner.integ;

% PWM 限幅 (clamp)
raw_pwm = round(max(min(pwm_cmd_f, params.plant.pwm_max), params.plant.pwm_min));

% 初始化 prev_pwm 字段（首次调用）
if ~isfield(ctrl.inner,'prev_pwm') || isempty(ctrl.inner.prev_pwm)
    ctrl.inner.prev_pwm = raw_pwm;
end

% PWM 斜率限制 (slew rate limiting)
prev_before = ctrl.inner.prev_pwm; % 保存换向检测用的上一周期值
max_step = ctrl.limits.pwm_slew; % 每个循环允许的最大变化
delta = raw_pwm - prev_before;
if delta > max_step
    delta = max_step;
elseif delta < -max_step
    delta = -max_step;
end
pwm_cmd = prev_before + delta; % 暂不写回 prev_pwm，待方向保护逻辑后再写

% ---------- 方向切换保护 (direction reversal protection) ---------- %
% 若试图换向且绝对偏离中立尚不足阈值 -> 输出中立并启动/维持一个中立保持计数
if isfield(ctrl.limits,'dir_thresh_pwm')
    dir_thresh = ctrl.limits.dir_thresh_pwm; % 例如 40 counts
    neutral_center = pwm_neutral;
    if ~isfield(ctrl.inner,'last_dir') || isempty(ctrl.inner.last_dir)
        prev_dev_tmp = prev_before - neutral_center;
        if prev_dev_tmp > 0
            ctrl.inner.last_dir = 1;
        elseif prev_dev_tmp < 0
            ctrl.inner.last_dir = -1;
        else
            ctrl.inner.last_dir = 0;
        end
    end
    dev = pwm_cmd - neutral_center;
    if dev > 0
        desired_dir = 1;
    elseif dev < 0
        desired_dir = -1;
    else
        desired_dir = 0;
    end
    % 若想换向且尚未达到阈值幅度 -> 强制保持中立，不更新 last_dir
    if desired_dir ~= 0 && desired_dir ~= ctrl.inner.last_dir && abs(dev) < dir_thresh
        pwm_cmd = neutral_center;
        % 积分逐步衰减避免释放跳变
        ctrl.inner.integ = 0.98 * ctrl.inner.integ;
    else
        % 一旦达到阈值并真正输出该方向则锁定新方向
        if desired_dir ~= 0 && abs(dev) >= dir_thresh
            ctrl.inner.last_dir = desired_dir;
        end
    end
end

% 现在写回 prev_pwm
ctrl.inner.prev_pwm = pwm_cmd;

end
