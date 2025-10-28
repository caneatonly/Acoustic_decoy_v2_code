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
  uint8_t byte = 0U;

  for (;;)
  {
    if (xQueueReceive(g_imuRxQueue, &byte, portMAX_DELAY) == pdPASS)
    {
      Cmd_GetPkt(byte);

      while (xQueueReceive(g_imuRxQueue, &byte, 0) == pdPASS)
      {
        Cmd_GetPkt(byte);
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
    uint32_t now_ms = HAL_GetTick();
    
    // 读取 MS5837 最新深度数据
    const MS5837_Data_t *ms = MS5837_data_get();
    if (ms && ms->data_valid)
    {
      // 使用互斥量保护深度估计器更新
      if (xSemaphoreTake(g_depthEstMutex, pdMS_TO_TICKS(5)) == pdPASS)
      {
        DepthEst_Update(&g_depth_est, ms->depth, now_ms);
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
    uint32_t now_ms = HAL_GetTick();
    
    // 1. 原子读取气阀控制参数（确保 duty 和 dPdt 来自同一周期）
    ValveTelemetryData_t valve_data;
    Valve_GetTelemetryData(&valve_data);
    
    // 2. 使用互斥量保护气囊状态机更新
    if (xSemaphoreTake(g_balloonMutex, pdMS_TO_TICKS(5)) == pdPASS)
    {
      Balloon_Update(&g_balloon, valve_data.duty, valve_data.dPdt, now_ms);
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

    balloon_state_t balloon_state = BALLOON_INFLATING;
    if (xSemaphoreTake(g_balloonMutex, pdMS_TO_TICKS(5)) == pdPASS)
    {
      balloon_state = g_balloon.state;
      xSemaphoreGive(g_balloonMutex);
    }
    Telemetry_SetBalloon((int)balloon_state);   
    float depth_est = 0.0f;
    float velocity_est = 0.0f;
    if (xSemaphoreTake(g_depthEstMutex, pdMS_TO_TICKS(5)) == pdPASS)
    {
      depth_est = DepthEst_GetDepth(&g_depth_est);
      velocity_est = DepthEst_GetVelocity(&g_depth_est);
      xSemaphoreGive(g_depthEstMutex);
    }
    uint32_t now_ms = HAL_GetTick();
    mission_status_t mission_snapshot = {0};
    mission_status_t *mission_locked = Mission_LockStatus(pdMS_TO_TICKS(5));
    if (mission_locked != NULL)
    {
      mission_snapshot = *mission_locked;
      Mission_UnlockStatus();
    }
    Telemetry_SetMissionState((int)mission_snapshot.state);
    Telemetry_SetDepth(depth_est, velocity_est, mission_snapshot.target_depth_m);

    // 原子读取所有气阀遥测数据（一次锁内完成，确保数据来自同一周期）
    ValveTelemetryData_t valve_data;
    Valve_GetTelemetryData(&valve_data);

    Telemetry_SetPressures(valve_data.p_bag, valve_data.p_water, 
                           valve_data.dPdt, valve_data.duty);
    
    // 读取控制数据（互斥量保护）
    float vref_pub = 0.0f;
    int16_t pwm_pub = CTRL_PWM_NEUTRAL;
    if (xSemaphoreTake(g_depthCtrlMutex, pdMS_TO_TICKS(5)) == pdPASS)
    {
      vref_pub = (mission_snapshot.state == MISSION_APPROACH || 
                  mission_snapshot.state == MISSION_PREP_HOLD || 
                  mission_snapshot.state == MISSION_DEPTH_HOLD)
                    ? DepthCtrl_GetVref(&g_depth_ctrl) : 0.0f;
      pwm_pub = mission_snapshot.motor_active ? DepthCtrl_GetPwm(&g_depth_ctrl) : CTRL_PWM_NEUTRAL;
      xSemaphoreGive(g_depthCtrlMutex);
    }
    Telemetry_SetControl(vref_pub, pwm_pub);
    // 发布遥测数据（内部使用互斥量保护 + console_printf）
    Telemetry_Publish(now_ms);
    
    // 周期性延时（1秒）
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
    uint32_t now_ms = HAL_GetTick();
    
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
    Mission_Update(now_ms, depth_est, velocity_est, balloon_state);
  
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
    uint32_t now_ms = HAL_GetTick();
    
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
        Mission_Execute(now_ms, depth_est, velocity_est, &g_depth_ctrl, mstat);
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
    g_imuRxQueue = xQueueCreate(IMU_RX_QUEUE_LENGTH, sizeof(uint8_t));
    configASSERT(g_imuRxQueue != NULL);
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
  status = xTaskCreate(Task_ImuProcess, "imu_proc", 512, NULL, tskIDLE_PRIORITY + 3, NULL);
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
