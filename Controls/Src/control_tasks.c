#include "control_tasks.h"

#include "bsp_usart.h"
#include "console.h"
#include "im948_CMD.h"
#include "usart.h"
#include "MS5837_lib.h"
#include "i2c.h"

// Control framework
#include "depth_estimator.h"
#include "mission_manager.h"
#include "mission_exec.h"
#include "telemetry.h"
#include "sensors_data_get.h"
#include "control_config.h"
#include "depth_cascaded_ctrl.h"
#include "actuators.h"
#include "valve_ctrl.h"
#include "balloon_state.h"
#include <math.h>
#include <stddef.h>

#define IMU_RX_QUEUE_LENGTH     256U
#define IMU_TX_QUEUE_LENGTH     6U

QueueHandle_t g_imuRxQueue = NULL;
QueueHandle_t g_imuTxQueue = NULL;
SemaphoreHandle_t g_ms5837DataMutex = NULL;

// Control loop shared data structures and mutexes
static depth_estimator_t g_depth_est;
static depth_ctrl_t g_depth_ctrl = {0};
static balloon_status_t g_balloon = {0};

SemaphoreHandle_t g_depthEstMutex = NULL;
SemaphoreHandle_t g_depthCtrlMutex = NULL;
SemaphoreHandle_t g_balloonMutex = NULL;
SemaphoreHandle_t g_balloonStartSem = NULL;

#define TLM_HISTORY_LENGTH 5U

typedef struct {
  TickType_t tick;
  float depth_err;
  float vel_err;
  int16_t pwm;
  float vref;
} telemetry_history_sample_t;

static telemetry_history_sample_t g_tlm_history[TLM_HISTORY_LENGTH] = {0};
static size_t g_tlm_history_count = 0U;
static size_t g_tlm_history_index = 0U;

static void IMU_ProcessBlock(const ImuRxBlock_t *block)
{
  if (block == NULL || block->data == NULL || block->length == 0U)
  {
    return;
  }

  uint8_t *data = block->data;
  uint16_t length = block->length;

  for (uint16_t i = 0U; i < length; ++i)
  {
    Cmd_GetPkt(data[i]);
  }
}

void Task_Led(void *argument)
{
  (void)argument;
  for (;;)
  {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void Task_ImuProcess(void *argument)
{
  (void)argument;
  ImuRxBlock_t block = {0};

  for (;;)
  {
    if (xQueueReceive(g_imuRxQueue, &block, portMAX_DELAY) == pdPASS)
    {
      IMU_ProcessBlock(&block);
      while (xQueueReceive(g_imuRxQueue, &block, 0) == pdPASS)
      {
        IMU_ProcessBlock(&block);
      }
    }
  }
}

void Task_ImuSend(void *argument)
{
  (void)argument;
  ImuTxFrame_t frame;

  for (;;)
  {
    if (xQueueReceive(g_imuTxQueue, &frame, portMAX_DELAY) == pdPASS)
    {
      (void)HAL_UART_Transmit(&huart2, frame.payload, frame.length, HAL_MAX_DELAY);
    }
  }
}

void Task_UartDebug(void *argument)
{
  (void)argument;
  for (;;)
  {
    UART1_DataHandler();
    // 防止饿死其他任务
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

/**
 * @brief MS5837压力传感器数据采集任务
 * @param argument 
 * 
 * 功能说明:
 * - 周期性调用MS5837状态机处理函数
 * - MS5837完整采样周期: ~50ms (D1转换19ms + D2转换19ms + 通信开销)
 * - 状态机内部使用vTaskDelay实现ADC转换等待
 * - 最终数据更新率: 约20Hz (每50ms一组新数据)
 * - 数据更新到全局变量 g_ms5837_data (互斥量保护)
 */
void Task_MS5837Process(void *argument)
{
  (void)argument;
  
  for (;;)
  {
    // 调用MS5837状态机处理函数
    // 状态机会根据当前状态自动处理延时（WAIT状态会调用vTaskDelay）
    MS5837_Process(&hi2c1, &MS5837_info_t);
    // 防止任务饿死其他低优先级任务
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

/**
 * @brief 深度估计器任务
 * @param argument 任务参数（未使用）
 * 
 * 功能说明:
 * - 专门负责深度滤波和速度计算
 * - 从 MS5837 原始数据中提取深度信息，进行 EMA 滤波
 * - 计算垂直速度（基于深度变化率）
 * - 周期: 20ms (50Hz) - 与 MS5837 数据更新率匹配
 */
void Task_DepthEstimator(void *argument)
{
  (void)argument;
  
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(20); // 20ms, 50Hz
  
  for (;;)
  {
    TickType_t now_tick = xTaskGetTickCount();
    
    // 读取 MS5837 最新深度数据
    const MS5837_Data_t *ms = MS5837_data_get();
    if (ms && ms->data_valid)
    {
      // 使用互斥量保护深度估计器更新
      if (xSemaphoreTake(g_depthEstMutex, pdMS_TO_TICKS(5)) == pdPASS)
      {
        DepthEst_Update(&g_depth_est, ms->depth, now_tick);
        xSemaphoreGive(g_depthEstMutex);
      }
    }
    
    // 周期性延时
    
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

/**
 * @brief 气囊状态机任务
 * @param argument 任务参数（未使用）

 * - 周期: 50ms (20Hz) - 气囊状态变化较慢，无需高频更新
 * 主要职责:
 * 1. 读取气阀控制参数（占空比、压力变化率）
 * 2. 调用气囊状态机更新函数
 * 3. 更新后的气囊状态供任务管理器使用
 * 
 * 状态转换逻辑:
 * - INFLATING: 充气中，当占空比=0 且 dP/dt 稳定 → STABILIZING
 * - STABILIZING: 稳定中，连续 N 个周期满足条件 → STABLE
 * - STABLE: 稳定完成（终态）
 */
void Task_BalloonStateMachine(void *argument)
{
  (void)argument;
  if (g_balloonStartSem != NULL)
  {
    (void)xSemaphoreTake(g_balloonStartSem, portMAX_DELAY);
  }
  
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(50); // 50ms, 20Hz
  
  for (;;)
  {
    TickType_t now_tick = xTaskGetTickCount();
    
    // 1. 原子读取气阀控制参数（确保 duty 和 dPdt 来自同一周期）
    ValveTelemetryData_t valve_data;
    Valve_GetTelemetryData(&valve_data);
    
    // 2. 使用互斥量保护气囊状态机更新
    if (xSemaphoreTake(g_balloonMutex, pdMS_TO_TICKS(5)) == pdPASS)
    {
      Balloon_Update(&g_balloon, valve_data.duty, valve_data.dPdt, now_tick);
      xSemaphoreGive(g_balloonMutex);
    }
    
    // 周期性延时
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

/**
 * @brief 遥测数据发布任务
 * @param argument 任务参数（未使用）
 * 
 * 功能说明:
 * - 独立的遥测任务，周期性发布系统状态数据
 * - 周期: 1000ms (1Hz) - 与原始 CTRL_STATUS_PUBLISH_MS 一致
 * - 使用 console_printf 进行线程安全的串口输出
 * 
 * 主要职责:
 * 1. 从遥测数据结构中读取最新状态（互斥量保护）
 * 2. 通过串口发布遥测信息（深度、速度、任务状态、控制输出等）
 */
void Task_Telemetry(void *argument)
{
  (void)argument;
  
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(CTRL_STATUS_PUBLISH_MS);
  
  for (;;)
  {    
    TickType_t now_tick = xTaskGetTickCount();

    const IMU_Data_t* imu = IMU_GetData();
    const MS5837_Data_t* ms5837 = MS5837_GetData();
    const BaroADC_Data_t* baro = BaroADC_GetData();

    float depth_est = 0.0f;
    float velocity_est = 0.0f;
    if (xSemaphoreTake(g_depthEstMutex, pdMS_TO_TICKS(5)) == pdPASS)
    {
      depth_est = DepthEst_GetDepth(&g_depth_est);
      velocity_est = DepthEst_GetVelocity(&g_depth_est);
      xSemaphoreGive(g_depthEstMutex);
    }

    mission_status_t mission_snapshot = {0};
    mission_status_t *mission_locked = Mission_LockStatus(pdMS_TO_TICKS(5));
    if (mission_locked != NULL)
    {
      mission_snapshot = *mission_locked;
      Mission_UnlockStatus();
    }

    float ctrl_vref = 0.0f;
    int16_t ctrl_pwm = CTRL_PWM_NEUTRAL;
    if (xSemaphoreTake(g_depthCtrlMutex, pdMS_TO_TICKS(5)) == pdPASS)
    {
      ctrl_vref = DepthCtrl_GetVref(&g_depth_ctrl);
      ctrl_pwm = DepthCtrl_GetPwm(&g_depth_ctrl);
      xSemaphoreGive(g_depthCtrlMutex);
    }

    balloon_state_t balloon_state = BALLOON_INFLATING;
    if (xSemaphoreTake(g_balloonMutex, pdMS_TO_TICKS(5)) == pdPASS)
    {
      balloon_state = g_balloon.state;
      xSemaphoreGive(g_balloonMutex);
    }

    ValveTelemetryData_t valve_data = {0};
    Valve_GetTelemetryData(&valve_data);

    float depth_target = mission_snapshot.target_depth_m;
    float depth_err = depth_est - depth_target;
    float vel_err = velocity_est - ctrl_vref;

    telemetry_history_sample_t new_sample = {
        .tick = now_tick,
        .depth_err = depth_err,
        .vel_err = vel_err,
        .pwm = ctrl_pwm,
        .vref = ctrl_vref
    };

    g_tlm_history[g_tlm_history_index] = new_sample;
    g_tlm_history_index = (g_tlm_history_index + 1U) % TLM_HISTORY_LENGTH;
    if (g_tlm_history_count < TLM_HISTORY_LENGTH)
    {
      g_tlm_history_count++;
    }

    const TickType_t window_ticks = pdMS_TO_TICKS(5000U);
    size_t sample_count = 0U;
    float depth_err_sum = 0.0f;
    float depth_err_abs_sum = 0.0f;
    float depth_err_sq_sum = 0.0f;
    float depth_err_max_abs = 0.0f;
    float vel_err_sum = 0.0f;
    float vel_err_abs_sum = 0.0f;
    float vel_err_sq_sum = 0.0f;
    float vel_err_max_abs = 0.0f;
    float pwm_sum = 0.0f;
    float vref_sum = 0.0f;
    int16_t pwm_min = CTRL_PWM_MAX;
    int16_t pwm_max = CTRL_PWM_MIN;
    size_t pwm_sat_count = 0U;
    TickType_t oldest_tick = now_tick;

    for (size_t i = 0U; i < g_tlm_history_count; ++i)
    {
      telemetry_history_sample_t *hist = &g_tlm_history[i];
      TickType_t age_ticks = (now_tick >= hist->tick) ? (now_tick - hist->tick) : 0U;
      if (age_ticks > window_ticks)
      {
        continue;
      }

      sample_count++;
      if (hist->tick < oldest_tick)
      {
        oldest_tick = hist->tick;
      }

      float depth_err_abs = fabsf(hist->depth_err);
      float vel_err_abs = fabsf(hist->vel_err);

      depth_err_sum += hist->depth_err;
      depth_err_abs_sum += depth_err_abs;
      depth_err_sq_sum += hist->depth_err * hist->depth_err;
      if (depth_err_abs > depth_err_max_abs)
      {
        depth_err_max_abs = depth_err_abs;
      }

      vel_err_sum += hist->vel_err;
      vel_err_abs_sum += vel_err_abs;
      vel_err_sq_sum += hist->vel_err * hist->vel_err;
      if (vel_err_abs > vel_err_max_abs)
      {
        vel_err_max_abs = vel_err_abs;
      }

      pwm_sum += (float)hist->pwm;
      vref_sum += hist->vref;
      if (hist->pwm < pwm_min)
      {
        pwm_min = hist->pwm;
      }
      if (hist->pwm > pwm_max)
      {
        pwm_max = hist->pwm;
      }
      if ((hist->pwm <= CTRL_PWM_MIN) || (hist->pwm >= CTRL_PWM_MAX))
      {
        pwm_sat_count++;
      }
    }

    float window_seconds = fminf(5.0f, sample_count * ((float)CTRL_STATUS_PUBLISH_MS / 1000.0f));
    if (sample_count == 0U)
    {
      window_seconds = 0.0f;
    }

    float depth_err_avg = (sample_count > 0U) ? (depth_err_sum / (float)sample_count) : 0.0f;
    float depth_err_avg_abs = (sample_count > 0U) ? (depth_err_abs_sum / (float)sample_count) : 0.0f;
    float depth_err_rms = (sample_count > 0U) ? sqrtf(depth_err_sq_sum / (float)sample_count) : 0.0f;

    float vel_err_avg = (sample_count > 0U) ? (vel_err_sum / (float)sample_count) : 0.0f;
    float vel_err_avg_abs = (sample_count > 0U) ? (vel_err_abs_sum / (float)sample_count) : 0.0f;
    float vel_err_rms = (sample_count > 0U) ? sqrtf(vel_err_sq_sum / (float)sample_count) : 0.0f;

    float pwm_avg = (sample_count > 0U) ? (pwm_sum / (float)sample_count) : (float)CTRL_PWM_NEUTRAL;
    float vref_avg = (sample_count > 0U) ? (vref_sum / (float)sample_count) : 0.0f;
    float pwm_sat_pct = (sample_count > 0U) ? ((float)pwm_sat_count * 100.0f / (float)sample_count) : 0.0f;

    // console_printf("System Status: IMU=%s MS5837=%s BARO=%s | samples=%u window=%.1fs\r\n",
    //                imu->data_valid ? "OK" : "--",
    //                ms5837->data_valid ? "OK" : "--",
    //                baro->data_valid ? "OK" : "--",
    //                (unsigned int)sample_count,
    //                window_seconds);

    console_printf("Depth=%.2f,Depth_target=%.2f,err=%.3f,depth_err_avg=%.3f,abs=%.3f,rms=%.3f,max_abs=%.3f\r\n",
                   depth_est, depth_target, depth_err,
                   depth_err_avg, depth_err_avg_abs, depth_err_rms, depth_err_max_abs);

    console_printf("Velocity=%.3f,V_ref=%.3f,err=%.3f | avg=%.3f,abs=%.3fm,rms=%.3fm,max_abs=%.3fm\r\n",
                   velocity_est, ctrl_vref, vel_err,
                   vel_err_avg, vel_err_avg_abs, vel_err_rms, vel_err_max_abs);

    console_printf("PWM: now %d,avg=%.0f,min=%d,max=%d,sat=%.1f%% | vref_avg=%.3f\r\n",
                   ctrl_pwm, pwm_avg, pwm_min, pwm_max, pwm_sat_pct, vref_avg);

    console_printf("Mission: state %d ctrl_mode %d motor %s valve %s balloon %d\r\n",
                   (int)mission_snapshot.state,
                   (int)mission_snapshot.ctrl_mode,
                   mission_snapshot.motor_active ? "ON" : "OFF",
                   mission_snapshot.valve_enable ? "ON" : "OFF",
                   (int)balloon_state);

    // console_printf("Valve: duty %.2f p_bag %.2f p_water %.2f dPdt %.2f\r\n",
    //                valve_data.duty,
    //                valve_data.p_bag,
    //                valve_data.p_water,
    //                valve_data.dPdt);

    // console_printf("IMU: Angle[%.2f,%.2f,%.2f] Accel[%.2f,%.2f,%.2f] | MS5837: T=%.2fC D=%.2fm P=%.2fkPa | BARO: %.2fkPa (%.3fV, raw=%u)\r\n",
    //                imu->angleX, imu->angleY, imu->angleZ,
    //                imu->accelX, imu->accelY, imu->accelZ,
    //                ms5837->temperature, ms5837->depth, ms5837->pressure_water,
    //                baro->pressure_bag, baro->voltage_v, baro->raw);

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

/**
 * @brief 任务状态机管理任务 (Mission Manager)
 * @param argument 任务参数（未使用）

 * 主要职责:
 * 1. 读取深度估计器的最新数据
 * 2. 读取气囊状态
 * 3. 更新任务状态机

 */
void Task_MissionManager(void *argument)
{
  (void)argument;
  
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(CTRL_PERIOD_MS);
  
  for (;;)
  {
    // 检查控制循环使能标志（ctrl on/off 命令控制）
    extern volatile uint8_t g_control_loop_enabled;
    if (!g_control_loop_enabled) {
      // 控制循环禁用，跳过状态机更新，允许手动控制
      vTaskDelayUntil(&xLastWakeTime, xPeriod);
      continue;
    }
    
    TickType_t now_tick = xTaskGetTickCount();
    
    // 1. 获取当前深度和速度估计
    float depth_est = 0.0f;
    float velocity_est = 0.0f;
    if (xSemaphoreTake(g_depthEstMutex, pdMS_TO_TICKS(5)) == pdPASS)
    {
      depth_est = DepthEst_GetDepth(&g_depth_est);
      velocity_est = DepthEst_GetVelocity(&g_depth_est);
      xSemaphoreGive(g_depthEstMutex);
    }
    
    // 2. 获取气囊状态
    balloon_state_t balloon_state = BALLOON_INFLATING;
    if (xSemaphoreTake(g_balloonMutex, pdMS_TO_TICKS(5)) == pdPASS)
    {
      balloon_state = g_balloon.state;
      xSemaphoreGive(g_balloonMutex);
    }
    
    // 3. Mission Manager 统一管理阶段切换
    Mission_Update(now_tick, depth_est, velocity_est, balloon_state);
  
    // 周期性延时
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

/**
 * @brief 任务执行机任务 (Mission Executor)
 * @param argument

 * 主要职责:
 * 1. 调用 Mission_Execute 执行当前任务状态对应的动作
 * 2. 更新深度控制器
 * 3. 输出电机PWM控制
 * 4. 更新气阀控制器
 */
void Task_MissionExecutor(void *argument)
{
  (void)argument;
  
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(CTRL_PERIOD_MS);
  
  for (;;)
  {
    // 检查控制循环使能标志（ctrl on/off 命令控制）
    extern volatile uint8_t g_control_loop_enabled;
    if (!g_control_loop_enabled) {
      // 控制循环禁用，跳过控制输出，允许手动控制
      vTaskDelayUntil(&xLastWakeTime, xPeriod);
      continue;
    }
    
    TickType_t now_tick = xTaskGetTickCount();
    
    // 1. 获取当前深度和速度估计（互斥量保护）
    float depth_est = 0.0f;
    float velocity_est = 0.0f;
    if (xSemaphoreTake(g_depthEstMutex, pdMS_TO_TICKS(5)) == pdPASS)
    {
      depth_est = DepthEst_GetDepth(&g_depth_est);
      velocity_est = DepthEst_GetVelocity(&g_depth_est);
      xSemaphoreGive(g_depthEstMutex);
    }
    
    // 2. 获取当前任务状态并执行动作
    mission_status_t *mstat = Mission_LockStatus(pdMS_TO_TICKS(5));
    if (mstat != NULL)
    {
      if (xSemaphoreTake(g_depthCtrlMutex, pdMS_TO_TICKS(5)) == pdPASS)
      {
        Mission_Execute(now_tick, depth_est, velocity_est, &g_depth_ctrl, mstat);
        xSemaphoreGive(g_depthCtrlMutex);
      }
      Mission_UnlockStatus();
    }
    
    // 周期性延时
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

void ControlTasks_Init(void)
{
  // Initialize console mutex before creating tasks
  console_init();

  // Initialize control loop data structures
  // 1. 深度估计器初始化
  DepthEst_Init(&g_depth_est, ESTIMATOR_EMA_ALPHA_Z, ESTIMATOR_VEL_BETA);
  
  // 2. 任务管理器初始化
  Mission_Init(CTRL_DEPTH_TARGET_M);
  
  // 3. 深度控制器初始化
  depth_ctrl_config_t cfg = {0};
  cfg.depth_app.kp = PID_Z_KP_APP; 
  cfg.depth_app.ki = PID_Z_KI_APP; 
  cfg.depth_app.kd = PID_Z_KD_APP;
  cfg.vel_app.kp   = PID_V_KP_APP; 
  cfg.vel_app.ki   = PID_V_KI_APP; 
  cfg.vel_app.kd   = PID_V_KD_APP;
  cfg.depth_hold.kp= PID_Z_KP_HOLD; 
  cfg.depth_hold.ki= PID_Z_KI_HOLD; 
  cfg.depth_hold.kd= PID_Z_KD_HOLD;
  cfg.vel_hold.kp  = PID_V_KP_HOLD; 
  cfg.vel_hold.ki  = PID_V_KI_HOLD; 
  cfg.vel_hold.kd  = PID_V_KD_HOLD;
  cfg.v_ref_max_app = CTRL_V_REF_MAX_APP; 
  cfg.v_ref_max_hold = CTRL_V_REF_MAX_HOLD; 
  cfg.v_ref_slew = CTRL_V_REF_SLEW;
  cfg.pwm_neutral = CTRL_PWM_NEUTRAL; 
  cfg.pwm_min = CTRL_PWM_MIN; 
  cfg.pwm_max = CTRL_PWM_MAX; 
  cfg.pwm_slew_per_tick = CTRL_PWM_SLEW_PER_TICK;
  cfg.dir_thresh_pwm = CTRL_PWM_DIR_THRESH;
  DepthCtrl_Init(&g_depth_ctrl, &cfg, CTRL_DEPTH_TARGET_M);
  
  // 4. 气囊状态机初始化
  Balloon_Init(&g_balloon);
  
  
  Telemetry_Init();

  // Create mutexes for shared data structures
  if (g_depthEstMutex == NULL)
  {
    g_depthEstMutex = xSemaphoreCreateMutex();
    configASSERT(g_depthEstMutex != NULL);
  }
  
  if (g_depthCtrlMutex == NULL)
  {
    g_depthCtrlMutex = xSemaphoreCreateMutex();
    configASSERT(g_depthCtrlMutex != NULL);
  }
  
  if (g_balloonMutex == NULL)
  {
    g_balloonMutex = xSemaphoreCreateMutex();
    configASSERT(g_balloonMutex != NULL);
  }

  if (g_balloonStartSem == NULL)
  {
    g_balloonStartSem = xSemaphoreCreateBinary();
    configASSERT(g_balloonStartSem != NULL);
  }

  if (g_imuRxQueue == NULL)
  {
    g_imuRxQueue = xQueueCreate(IMU_RX_QUEUE_LENGTH, sizeof(ImuRxBlock_t));
    configASSERT(g_imuRxQueue != NULL);
  }

  /* 启动IMU串口DMA接收（双缓冲 + IDLE回调） */
  if (IMU_UART_StartDmaReception() != HAL_OK)
  {
    configASSERT(0);
  }

  if (g_imuTxQueue == NULL)
  {
    g_imuTxQueue = xQueueCreate(IMU_TX_QUEUE_LENGTH, sizeof(ImuTxFrame_t));
    configASSERT(g_imuTxQueue != NULL);
  }

  // 创建MS5837数据保护互斥量
  if (g_ms5837DataMutex == NULL)
  {
    g_ms5837DataMutex = xSemaphoreCreateMutex();
    configASSERT(g_ms5837DataMutex != NULL);
  }

  /* Valve control relies on MS5837 mutex for sensor snapshots. */
  Valve_ControlAlgorithm_Init();

  BaseType_t status;

  // 创建LED心跳任务（最低优先级）
  status = xTaskCreate(Task_Led, "led", 256, NULL, tskIDLE_PRIORITY + 1, NULL);
  configASSERT(status == pdPASS);

  // 创建IMU处理任务（中等优先级）
  status = xTaskCreate(Task_ImuProcess, "imu_proc", 512, NULL, tskIDLE_PRIORITY + 4, NULL);
  configASSERT(status == pdPASS);

  // 创建IMU发送任务（中等优先级）
  status = xTaskCreate(Task_ImuSend, "imu_tx", 384, NULL, tskIDLE_PRIORITY + 3, NULL);
  configASSERT(status == pdPASS);

  // 创建调试串口任务（较高优先级）
  status = xTaskCreate(Task_UartDebug, "uart_dbg", 384, NULL, tskIDLE_PRIORITY + 4, NULL);
  configASSERT(status == pdPASS);

  // 创建MS5837传感器任务（中等优先级）
  status = xTaskCreate(Task_MS5837Process, "ms5837", 512, NULL, tskIDLE_PRIORITY + 2, NULL);
  configASSERT(status == pdPASS);

  // 创建深度估计器任务（中等偏高优先级 - 为状态管理提供数据）
  status = xTaskCreate(Task_DepthEstimator, "depth_est", 384, NULL, tskIDLE_PRIORITY + 4, NULL);
  configASSERT(status == pdPASS);

  // 创建气囊状态机任务（中等优先级 - 监测气囊状态）
  status = xTaskCreate(Task_BalloonStateMachine, "balloon_sm", 384, NULL, tskIDLE_PRIORITY + 3, NULL);
  configASSERT(status == pdPASS);

  // 创建遥测发布任务（低优先级 - 避免串口输出阻塞控制逻辑）
  status = xTaskCreate(Task_Telemetry, "telemetry", 512, NULL, tskIDLE_PRIORITY + 1, NULL);
  configASSERT(status == pdPASS);

  // 创建任务管理器任务（高优先级 - 负责状态更新和数据采集）
  // 栈大小优化: 768->512 words (任务逻辑简单，无深度递归)
  status = xTaskCreate(Task_MissionManager, "mission_mgr", 512, NULL, tskIDLE_PRIORITY + 4, NULL);
  configASSERT(status == pdPASS);

  // 创建任务执行机任务（高优先级 - 负责控制输出）
  // 栈大小优化: 768->512 words (Mission_Execute不涉及复杂计算)
  status = xTaskCreate(Task_MissionExecutor, "mission_exec", 512, NULL, tskIDLE_PRIORITY + 4, NULL);
  configASSERT(status == pdPASS);
}
