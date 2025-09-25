# MATLAB 定深控制仿真框架 (Depth Hold Simulation Framework)

> 目的 (Purpose): 在入水实验前离线验证与初步整定 (initial tuning) 串级 PI 控制器，减少现场调参迭代。

## 1. 模型假设 (Model Assumptions)
1. 仅保留竖直 heave 方向 (1-DOF vertical).
2. 推力 (thrust) 与 PWM 在中立点邻域线性 (locally linear around neutral).
3. 小速度附近线性化时忽略二次阻力 (quadratic drag) 与净浮力已由配平推力抵消 (trim thrust cancels net buoyancy).
4. 估计器使用 EMA (Exponentially Weighted Moving Average) 与简单差分速度 (difference velocity estimate).

动力学方程 (Dynamics):
\[(m + m_a) \dot v + d_1 v + d_2 |v| v + (B-W) = T(u)\]
\[\dot z = v\]

配平 (Trim): 取 \(T_0 = (B - W)\) 使得 \(v=0\) 为平衡点，控制器实际调节的是 \(T' = T - T_0\)。

## 2. 串级结构 (Cascaded Structure)
- 外环 (Outer loop): 深度 PI (Depth PI) 生成速度参考 v_ref。
- 内环 (Inner loop): 速度 PI (Velocity PI) 生成 PWM / 推力命令 (thrust command)。
- 斜率限制 (Slew limits) 与积分限幅 (integrator clamping) 避免过冲和风up (windup)。

## 3. 快速使用 (Quick Start)
在 MATLAB 中依次运行:
1. `plant_params.m` 生成参数与初值 (parameters and initial gains)
2. `run_depth_sim.m` 运行仿真 (run simulation)
3. 根据图形/指标调节 `plant_params.m` 中 `lambda_v`, `tau_z`, `Ki` 等再重复

## 4. 文件说明 (Files)
| 文件 | 作用 (Purpose) |
|------|----------------|
| `plant_params.m` | 定义物理/控制参数与初始 PI 增益 (physical + control params) |
| `depth_controller.m` | 串级 PI 控制器实现 (cascaded PI implementation) |
| `run_depth_sim.m` | 主仿真脚本 (main simulation harness) |
| `README.md` | 说明文档 (this doc) |

## 5. 固件变量映射 (Firmware <-> MATLAB Mapping)
| 固件 (Firmware) | MATLAB 变量 | 含义 (Meaning) |
|------------------|-------------|----------------|
| `CTRL_V_REF_MAX_APP` | `ctrl.limits.v_ref_max_app` | 接近模式速度参考上限 (approach v_ref max) |
| `CTRL_V_REF_MAX_HOLD`| `ctrl.limits.v_ref_max_hold`| 保持模式速度参考上限 (hold v_ref max) |
| `CTRL_V_REF_SLEW` | `ctrl.limits.v_ref_slew` | 速度参考斜率限制 (slew rate) |
| `PID_Z_KP_APP/HOLD` | `ctrl.outer.Kp` (per mode) | 深度环比例 (depth Kp) |
| `PID_Z_KI_APP/HOLD` | `ctrl.outer.Ki` (per mode) | 深度环积分 (depth Ki) |
| `PID_V_KP_APP/HOLD` | `ctrl.inner.Kp` | 速度环比例 (velocity Kp) |
| `PID_V_KI_APP/HOLD` | `ctrl.inner.Ki` | 速度环积分 (velocity Ki) |
| `CTRL_PWM_NEUTRAL` | `plant.pwm_neutral` | 中立 PWM (neutral) |
| `CTRL_PWM_MIN/MAX` | `plant.pwm_min/max` | PWM 限幅 (limits) |
| `CTRL_PWM_SLEW_PER_TICK` | `ctrl.limits.pwm_slew` | PWM 斜率限制 (slew) |
| EMA alpha (`ESTIMATOR_EMA_ALPHA_Z`) | `alpha` | 深度滤波系数 (depth filter) |

## 6. IMC 初始增益 (IMC Initial Gains)
内环线性化: \(G_v(s)=1/(m_{eff}s + d_1)\)，设 \(\lambda_v < m_{eff}/d_1\) 得:
\[K_{p,v} = \frac{m_{eff}/d_1}{(1/d_1)\lambda_v} = \frac{m_{eff}}{\lambda_v}\]
更精确使用通用公式: \(K_{p,v} = \tau_v / (K_v \lambda_v)\), \(K_{i,v}=K_{p,v}/\tau_v\)。
外环将闭合速度环视为一阶系统，选择 \(\lambda_z \gg \lambda_v\)。

## 7. 调参建议 (Tuning Tips)
1. 先关掉深度积分 (Ki_z = 0) 观察无偏差漂移，确认配平推力或浮力误差 (trim bias)。
2. 逐步增大 Kp_z 到能接受的响应与较小超调 (acceptable overshoot)。
3. 加入小 Ki_z 消除稳态偏差 (steady-state error)；防止过大导致缓慢振荡 (slow oscillation)。
4. 内环过快会放大噪声 (noise amplification)；保持 `lambda_v` 约为 0.3~0.5 * 原始 `tau_v`。
5. 若出现 PWM 饱和 (saturation) -> 降低外环 Kp 或增加斜率限制。

## 8. 导出到固件 (Export to Firmware)
调参完成后，将最终 Kp/Ki 写回 `control_config.h` 对应宏；保留原值注释历史。

## 9. 后续扩展 (Next Steps)
- Monte Carlo: 扰动 m_a, d2, F_buoy ±30% -> 统计稳态误差与超调。
- 数据拟合: 使用实验 CSV 拟合 d1/d2/F_buoy (最小二乘)。
- 抗积分饱和策略 (Anti-windup): 条件积分 + back-calculation。
- ROS2 + Gazebo: 用统一参数接口加载。

## 10. 许可证 (License)
内部研究使用 (internal use). 如需开源需补充许可说明。
