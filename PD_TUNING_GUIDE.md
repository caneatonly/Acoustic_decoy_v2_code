# PD 调参测试模式使用指南

## 🎯 功能说明
本分支 `feature/pdtest-manual-target` 提供**岸上 PD 调参测试模式**，允许在不下水的情况下调试气囊充气控制算法。

### 核心特性
✅ **跳过任务状态机**：不会因 10s 入水超时进入 FAILSAFE  
✅ **电机自动失能**：避免螺旋桨转动  
✅ **阀控独立运行**：只测试充气算法  
✅ **手动目标压力**：可设定任意目标进行测试  

---

## 📋 岸上调参完整流程

### 1. 硬件准备
```
- 连接 ST-Link 或 nanoDAP 调试器
- 连接蓝牙串口（UART1，波特率 115200）
- 确保气囊压力传感器（ADC）正常连接
- 确保 MS5837 深度传感器正常连接
- 高压气瓶接入充气系统
```

### 2. 烧录固件
```bash
# 在 VS Code 中运行任务
Task: CubeProg: Flash project (ST-Link)
# 或
Task: OpenOCD: Flash project (nanoDAP-wireless)
```

### 3. 启动测试模式
通过蓝牙串口终端（如 PuTTY, MobaXterm, Arduino Serial Monitor）连接：

```bash
help              # 查看所有命令
status            # 确认传感器数据正常
pdtest on         # 🔥 启用测试模式（核心命令）
```

**确认信息**：
```
pdtest: ON
```

### 4. 设置目标压力
```bash
# 方法1: 根据当前大气压设定（推荐）
status            # 查看当前 P_water（大气压约101kPa）
set ptarget=110.0 # 设定目标为 110kPa（高于大气压9kPa）

# 方法2: 模拟水下压力
set ptarget=150.0 # 模拟5米水深的压力目标
```

### 5. 查看初始参数
```bash
params
```

**输出示例**：
```
Params: b.kp=0.0150 b.kd=0.3000 b.eps=2.000 kPa b.margin=1.000 kPa
```

### 6. 观察充气响应
```bash
status            # 每隔几秒查看一次
```

**关键指标**：
- `BARO: XX.XXkPa` - 气囊当前压力
- `MS5837: P=XX.XXkPa` - 环境压力（岸上≈101kPa）
- 观察压力是否稳定收敛到目标值

### 7. 迭代调整参数

#### 响应过慢（欠阻尼）
```bash
set b.kp=0.020    # 增大 Kp
status            # 观察响应
```

#### 震荡/超调（过激）
```bash
set b.kd=0.40     # 增大 Kd（增强阻尼）
# 或
set b.kp=0.012    # 减小 Kp
```

#### 稳态误差过大
```bash
set b.eps=1.5     # 减小死区
set b.margin=1.5  # 调整压力裕量
```

### 8. 记录最优参数
```bash
params            # 查看最终参数
```

**示例输出**：
```
Params: b.kp=0.0180 b.kd=0.3500 b.eps=2.000 kPa b.margin=1.000 kPa
```

### 9. 退出测试模式
```bash
pdtest off        # 关闭测试模式，恢复正常任务逻辑
```

---

## 🔧 常用调参场景

### 场景 1：模拟 5 米水深充气
```bash
pdtest on
set ptarget=150.0    # 5m水深 ≈ 101 + 50 = 151kPa
status
```

### 场景 2：快速响应测试
```bash
set b.kp=0.025       # 更激进的比例增益
set b.kd=0.25        # 降低阻尼快速响应
```

### 场景 3：平滑稳定测试
```bash
set b.kp=0.012       # 温和的比例增益
set b.kd=0.45        # 强阻尼避免震荡
```

---

## ⚠️ 注意事项

### 安全提醒
1. ⚠️ **气瓶压力**：确认高压气瓶压力充足且安全阀正常
2. ⚠️ **超压保护**：系统会在 `P_bag > P_water + 30kPa` 时自动关阀
3. ⚠️ **手动关阀**：异常时可用 `valve_close` 命令紧急关闭

### 限制说明
- 岸上测试时 `P_water ≈ 101kPa`（大气压），实际水下会动态变化
- 测试模式下电机始终失能（PWM=1500 中立）
- 气囊状态机不参与，只测试纯 PD 控制算法

### 典型参数范围（参考）
```
Kp: 0.010 ~ 0.030     （过大会震荡）
Kd: 0.20 ~ 0.50       （过小响应慢，过大阻尼大）
eps: 1.0 ~ 3.0 kPa    （死区，对应 ±10~30cm 水深）
margin: 0.5 ~ 2.0 kPa （压力裕量，常规 1.0kPa）
```

---

## 📊 调参评价指标

### 优秀的 PD 参数应满足
✅ **快速响应**：30-60秒内接近目标压力  
✅ **无超调/小超调**：超调量 < 10%  
✅ **稳定收敛**：稳态误差 < eps（死区内）  
✅ **无震荡**：压力曲线平滑单调或轻微阻尼振荡  

---

## 🚀 调参完成后

### 更新代码中的默认值
将优化后的参数写入 `valve_ctrl.c`：

```c
// PD Controller Parameters 
static float  Kp_valve = 0.018f;  // ← 更新你的最优值
static float  Kd_valve = 0.35f;   // ← 更新你的最优值
static float  eps = 2.0f;         // ← 根据需要调整
static float  dp_margin = 1.0f;   // ← 根据需要调整
```

### 进行水池联合测试
```bash
pdtest off        # 关闭测试模式
ctrl on           # 确认控制循环启用
reset             # 复位，进入完整任务流程
```

---

## 🐛 故障排查

### 问题：status 显示传感器无效
```bash
# 检查 I2C 连接（MS5837）
# 检查 ADC 连接（气囊压力）
power_on          # 确认 12V 电源已打开
```

### 问题：充气无响应
```bash
get ptarget       # 确认目标压力已设定
params            # 确认 Kp/Kd 不为零
valve_open        # 手动测试阀门是否工作
```

### 问题：压力超调严重
```bash
set b.kd=0.50     # 增大阻尼
set b.kp=0.010    # 降低增益
```

---

## 📝 版本信息
- **分支**：feature/pdtest-manual-target
- **固件版本**：查看 `ver` 命令输出
- **更新日期**：2025-10-03
- **作者**：Zyshine3 <zyshine3@sjtu.edu.cn>

---

**提示**：完成岸上调参后，建议在浅水池进行验证测试，最终在实际工况下微调。
