/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "power_manager.h"
#include "task_manager.h"
#include "opt4048.h"
#include "i2c_manager.h"
#include "tmp117.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// System State
typedef struct {
    float currentLux;
    float currentTemp;
    bool luxConditionMet;
    bool tempCritical;
    uint32_t modeStartTime;
    PowerMode_t currentMode;
} SystemState_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// System Configuration
#define LUX_WAKE_THRESHOLD          100.0f    // Lux level to wake to normal mode
#define LUX_SLEEP_THRESHOLD         50.0f     // Lux level to allow sleep
#define TEMP_CRITICAL_THRESHOLD     85.0f     // Temperature threshold (°C)
#define NORMAL_MODE_TIMEOUT_MS      30000     // Time in normal mode before sleep
#define LOW_POWER_TIMEOUT_MS        5000      // Time in low power before sleep
#define SENSOR_READ_INTERVAL_MS     1000      // Sensor reading interval
#define DEBUG_BUFFER_SIZE           256

// Task Priorities
#define PRIORITY_CRITICAL           (configMAX_PRIORITIES - 1)
#define PRIORITY_HIGH               (configMAX_PRIORITIES - 2)
#define PRIORITY_NORMAL             (configMAX_PRIORITIES - 3)
#define PRIORITY_LOW                (configMAX_PRIORITIES - 4)

// Task Stack Sizes
#define STACK_SIZE_LARGE            1024
#define STACK_SIZE_MEDIUM           512
#define STACK_SIZE_SMALL            256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart3;

OPT4048 opt;

TMP117 tmp;

static char debugBuffer[DEBUG_BUFFER_SIZE];

static TaskHandle_t sensorTaskHandle = NULL;
static TaskHandle_t systemControlTaskHandle = NULL;
static TaskHandle_t debugTaskHandle = NULL;

static SystemState_t systemState = {0};

static PowerManagerConfig_t powerConfig;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for OPT4048_Init */
osThreadId_t OPT4048_InitHandle;
const osThreadAttr_t OPT4048_Init_attributes = {
  .name = "OPT4048_Init",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for FaultManagerTas */
osThreadId_t FaultManagerTasHandle;
const osThreadAttr_t FaultManagerTas_attributes = {
  .name = "FaultManagerTas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* USER CODE BEGIN PV */
uint8_t tx_buffer[27] = "Testing\n\r";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_RTC_Init(void);
void StartDefaultTask(void *argument);
void opt4048_init(void *argument);
void FaultManager(void *argument);

/* USER CODE BEGIN PFP */
// Task Functions
static void SensorTask(void *pvParameters);
static void SystemControlTask(void *pvParameters);
static void DebugTask(void *pvParameters);

// Callback Functions
static PowerResult_t SystemHealthCheck(void);
static PowerResult_t PowerModeChangeCallback(PowerMode_t fromMode, PowerMode_t toMode);
static void WakeupCallback(WakeupSource_t source);
static void PeripheralCallback(Subsystem_t subsystem, bool enabled);

// Debug Functions
static void DebugPrint(const char* format, ...);
static void PrintSystemStatus(void);

// Subsystem Management Functions
static void I2C_Enable(void);
static void I2C_Disable(void);
static void UART_Enable(void);
static void UART_Disable(void);
static PowerResult_t I2C_HealthCheck(void);
static PowerResult_t UART_HealthCheck(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  SemaphoreHandle_t I2C1_Mutex = xSemaphoreCreateMutex();
  registerI2CHandleWithMutex(&hi2c1, I2C1_Mutex);

  // Initialize system state
   systemState.currentMode = POWERMODE_NORMAL;
   systemState.modeStartTime = HAL_GetTick();

      // Configure Power Manager
   static SubsystemConfig_t subsystems[] = {
          {
              .id = SUBSYSTEM_I2C_LUX,
              .isCritical = true,
              .enableHardware = I2C_Enable,
              .disableHardware = I2C_Disable,
              .healthCheck = I2C_HealthCheck
          },
          {
              .id = SUBSYSTEM_I2C_TMP,
              .isCritical = false,
              .enableHardware = I2C_Enable,
              .disableHardware = I2C_Disable,
              .healthCheck = I2C_HealthCheck
          },
          {
              .id = SUBSYSTEM_UART_DEBUG,
              .isCritical = false,
              .enableHardware = UART_Enable,
              .disableHardware = UART_Disable,
              .healthCheck = UART_HealthCheck
          }
      };

   powerConfig.subsystems = subsystems;
   powerConfig.numSubsystems = sizeof(subsystems) / sizeof(SubsystemConfig_t);

      // Clock configuration for different power modes
   powerConfig.clockConfig.operating.pllM = 4;
   powerConfig.clockConfig.operating.pllN = 280;
   powerConfig.clockConfig.operating.pllP = 4;
   powerConfig.clockConfig.operating.pllQ = 4;
   powerConfig.clockConfig.operating.pllR = 2;
   powerConfig.clockConfig.operating.ahbPrescaler = RCC_SYSCLK_DIV1;
   powerConfig.clockConfig.operating.apb1Prescaler = RCC_HCLK_DIV2;
   powerConfig.clockConfig.operating.apb2Prescaler = RCC_HCLK_DIV2;
   powerConfig.clockConfig.operating.apb3Prescaler = RCC_HCLK_DIV2;
   powerConfig.clockConfig.operating.apb4Prescaler = RCC_HCLK_DIV2;

   powerConfig.clockConfig.standby.usePLL = false;
   powerConfig.clockConfig.standby.ahbPrescaler = RCC_SYSCLK_DIV4;
   powerConfig.clockConfig.standby.apb1Prescaler = RCC_HCLK_DIV2;
   powerConfig.clockConfig.standby.apb2Prescaler = RCC_HCLK_DIV2;
   powerConfig.clockConfig.standby.apb3Prescaler = RCC_HCLK_DIV2;
   powerConfig.clockConfig.standby.apb4Prescaler = RCC_HCLK_DIV2;

      // Wakeup configuration
   powerConfig.wakeupConfig.enableTimer = true;
   powerConfig.wakeupConfig.timerWakeupMs = 30000; // 30 second intervals

      // Task manager function pointers
   powerConfig.suspendAllTasks = TaskManager_SuspendAllTasks;
   powerConfig.suspendNonCriticalTasks = TaskManager_SuspendAllNonCriticalTasks;
   powerConfig.resumeAllTasks = TaskManager_ResumeAllTasks;
   powerConfig.manageSubsystemTasks = TaskManager_ManageSubsystemTasks;

   TaskManager_Init();

      if (PowerManager_Init(&powerConfig) != POWER_OK) {
          DebugPrint("ERROR: Power Manager initialization failed\r\n");
          Error_Handler();
      }

      // Set callbacks
      PowerManager_SetHealthCheckCallback(SystemHealthCheck);
      PowerManager_EnableRaceToSleep(true);

      // Handle wakeup from STANDBY mode
      PowerResult_t wakeupResult = PowerManager_HandleWakeup();
      if (wakeupResult != POWER_OK) {
          DebugPrint("WARNING: Wakeup handling returned error: %d\r\n", wakeupResult);
      }

      // Initialize sensors
      if (OPT4048_Init(&opt, &hi2c1) != 0) {
          DebugPrint("ERROR: LUX sensor initialization failed\r\n");
          Error_Handler();
      }

      if (TMP117_Initialization(&tmp, &hi2c1) != 0) {
          DebugPrint("ERROR: Temperature sensor initialization failed\r\n");
          Error_Handler();
      }

      DebugPrint("System initialized successfully\r\n");

      if (xTaskCreate(SensorTask, "Sensor", STACK_SIZE_MEDIUM, NULL,
                         PRIORITY_HIGH, &sensorTaskHandle) != pdPASS) {
              Error_Handler();
          }

       if (xTaskCreate(SystemControlTask, "SysCtrl", STACK_SIZE_MEDIUM, NULL,
                         PRIORITY_CRITICAL, &systemControlTaskHandle) != pdPASS) {
              Error_Handler();
          }

       if (xTaskCreate(DebugTask, "Debug", STACK_SIZE_SMALL, NULL,
                         PRIORITY_LOW, &debugTaskHandle) != pdPASS) {
              Error_Handler();
          }

          // Register tasks with Task Manager
       TaskManager_Register(sensorTaskHandle, "Sensor", PRIORITY_HIGH,
                              TASK_TYPE_CRITICAL, SUBSYSTEM_I2C_LUX);
       TaskManager_Register(systemControlTaskHandle, "SysCtrl", PRIORITY_CRITICAL,
                              TASK_TYPE_CRITICAL, SUBSYSTEM_COUNT);
       TaskManager_Register(debugTaskHandle, "Debug", PRIORITY_LOW,
                              TASK_TYPE_SUBSYSTEM, SUBSYSTEM_UART_DEBUG);

       DebugPrint("Starting FreeRTOS scheduler\r\n");
       PrintSystemStatus();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  //QueueHandle_t xFaultQueue;
  //xFaultQueue = xQueueCreate(10, sizeof(fault_event_t));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of OPT4048_Init */
  OPT4048_InitHandle = osThreadNew(opt4048_init, NULL, &OPT4048_Init_attributes);

  /* creation of FaultManagerTas */
  FaultManagerTasHandle = osThreadNew(FaultManager, NULL, &FaultManagerTas_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief Sensor monitoring task - reads lux and temperature
 */
static void SensorTask(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        TaskManager_SendHeartbeat(sensorTaskHandle);

        // Read lux sensor
        if (OPT4048_GetCIE(&opt) == 0) {
            systemState.currentLux = opt.lux;
            systemState.luxConditionMet = (systemState.currentLux > LUX_WAKE_THRESHOLD);
        } else {
            //DebugPrint("ERROR: Failed to read lux sensor\r\n");
        }

        // Read temperature sensor
        if (TMP117_GetTemperatureData(&tmp) == 0) {
            systemState.currentTemp = tmp.temp_Celcius;
            systemState.tempCritical = (systemState.currentTemp > TEMP_CRITICAL_THRESHOLD);
        } else {
            DebugPrint("ERROR: Failed to read temperature sensor\r\n");
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SENSOR_READ_INTERVAL_MS));
    }
}

/**
 * @brief System control task - manages power mode transitions
 */
static void SystemControlTask(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        TaskManager_SendHeartbeat(systemControlTaskHandle);

        PowerMode_t currentMode = systemState.currentMode;
        uint32_t timeInMode = HAL_GetTick() - systemState.modeStartTime;
        bool shouldTransition = false;
        PowerMode_t targetMode = currentMode;

        switch (currentMode) {
            case POWERMODE_NORMAL:
                // Transition to sleep if lux is low and timeout reached
                if (systemState.currentLux > 800) {
                    targetMode = POWERMODE_LOW_POWER;
                    shouldTransition = true;
                    DebugPrint("Normal -> Low Power: High Lux\r\n");
                }
                // Emergency sleep if temperature critical
                else if (systemState.tempCritical) {
                    targetMode = POWERMODE_SLEEP;
                    shouldTransition = true;
                    DebugPrint("Normal -> Sleep: Critical temperature\r\n");
                }
                break;

            case POWERMODE_LOW_POWER:
                // Transition to normal if lux condition is met
                if (systemState.currentLux  < 800) {
                    targetMode = POWERMODE_NORMAL;
                    shouldTransition = true;
                    DebugPrint("Low Power -> Normal: Lux condition met\r\n");
                }
                // Transition to sleep after timeout
                else if (timeInMode > LOW_POWER_TIMEOUT_MS) {
                    //targetMode = POWERMODE_SLEEP;
                    //shouldTransition = true;
                    //DebugPrint("Low Power -> Sleep: Timeout reached\r\n");
                }
                break;

            case POWERMODE_SLEEP:
                // This should not happen during normal operation
                // Sleep mode transitions are handled by PowerManager_HandleWakeup()
                break;
        }

        if (shouldTransition) {
            PowerResult_t result = PowerManager_SetMode(targetMode);
            if (result == POWER_OK) {
                systemState.currentMode = targetMode;
                systemState.modeStartTime = HAL_GetTick();
                DebugPrint("Power mode changed to: %s\r\n",
                          PowerManager_GetModeString(targetMode));
            } else {
                DebugPrint("ERROR: Power mode transition failed: %d\r\n", result);
            }
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000)); // Check every second
    }
}

/**
 * @brief Debug task - prints system status periodically
 */
static void DebugTask(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        TaskManager_SendHeartbeat(debugTaskHandle);
        PrintSystemStatus();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5000)); // Print every 5 seconds
    }
}

/**
 * @brief System health check callback
 */
static PowerResult_t SystemHealthCheck(void)
{
    // Check if critical sensors are responding
    if (OPT4048_GetCIE(&opt) != 0) {
        DebugPrint("Health Check: LUX sensor failed\r\n");
        return POWER_ERROR_PERIPHERAL_CONFIG;
    }

    if (TMP117_GetTemperatureData(&tmp) != 0) {
        DebugPrint("Health Check: Temperature sensor failed\r\n");
        return POWER_ERROR_PERIPHERAL_CONFIG;
    }

    // Check for critical temperature
    if (systemState.tempCritical) {
        DebugPrint("Health Check: Critical temperature detected\r\n");
        return POWER_ERROR_SUBSYSTEM_FAIL;
    }

    return POWER_OK;
}

/**
 * @brief Power mode change callback
 */
static PowerResult_t PowerModeChangeCallback(PowerMode_t fromMode, PowerMode_t toMode)
{
    DebugPrint("Power mode transition: %s -> %s\r\n",
               PowerManager_GetModeString(fromMode),
               PowerManager_GetModeString(toMode));

    // Perform any additional mode-specific setup
    switch (toMode) {
        case POWERMODE_NORMAL:
            break;

        case POWERMODE_LOW_POWER:
            break;

        case POWERMODE_SLEEP:
            DebugPrint("Entering STANDBY mode...\r\n");
            HAL_Delay(100); // Ensure debug message is sent
            break;
    }

    return TaskManager_PowerModeCallback(fromMode, toMode);
}

/**
 * @brief Wakeup callback
 */
static void WakeupCallback(WakeupSource_t source)
{
    switch (source) {
        case WAKEUP_SOURCE_RTC:
            DebugPrint("Wakeup: RTC timer\r\n");
            break;
        case WAKEUP_SOURCE_RESET:
            DebugPrint("Wakeup: System reset\r\n");
            break;
        default:
            DebugPrint("Wakeup: Unknown source\r\n");
            break;
    }
}

/**
 * @brief Peripheral state change callback
 */
static void PeripheralCallback(Subsystem_t subsystem, bool enabled)
{
    DebugPrint("Subsystem %s: %s\r\n",
               PowerManager_GetSubsystemString(subsystem),
               enabled ? "Enabled" : "Disabled");
}

/**
 * @brief Debug print function
 */
static void DebugPrint(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    vsnprintf(debugBuffer, DEBUG_BUFFER_SIZE, format, args);
    va_end(args);

    HAL_UART_Transmit(&huart3, (uint8_t*)debugBuffer, strlen(debugBuffer), 100);
}

/**
 * @brief Print system status
 */
static void PrintSystemStatus(void)
{

	uint8_t numArray[5];
	uint32_t lux_num, tmp_num;
	lux_num = opt.lux;
	tmp_num = tmp.temp_Celcius;

    DebugPrint("=== System Status ===\r\n");
    DebugPrint("Mode: %s (Time: %lu ms)\r\n",
               PowerManager_GetModeString(systemState.currentMode),
               HAL_GetTick() - systemState.modeStartTime);
    sprintf(numArray, "%d\r\n", lux_num);
	HAL_UART_Transmit(&huart3,"Lux: ", 5, 100);
	HAL_UART_Transmit(&huart3, numArray, 6, 100);

	sprintf(numArray, "%d\r\n", tmp_num);

	HAL_UART_Transmit(&huart3,"Temp: ", 5, 100);
	HAL_UART_Transmit(&huart3, numArray, 5, 100);

    DebugPrint("Unhealthy tasks: %lu\r\n", TaskManager_GetUnhealthyTaskCount());
    DebugPrint("Free heap: %lu bytes\r\n", xPortGetFreeHeapSize());
    DebugPrint("===================\r\n");
}

// Subsystem management functions
static void I2C_Enable(void)
{
    __HAL_RCC_I2C1_CLK_ENABLE();
    MX_I2C1_Init();

}

static void I2C_Disable(void)
{
    HAL_I2C_DeInit(&hi2c1);
    __HAL_RCC_I2C1_CLK_DISABLE();
}

static void UART_Enable(void)
{
    __HAL_RCC_USART3_CLK_ENABLE();
    MX_USART3_UART_Init();
}

static void UART_Disable(void)
{
    HAL_UART_DeInit(&huart3);
    __HAL_RCC_USART3_CLK_DISABLE();
}

static PowerResult_t I2C_HealthCheck(void)
{
    if (HAL_I2C_IsDeviceReady(&hi2c1, OPT4048_I2C_ADDR << 1, 1, 100) != HAL_OK) {
        return POWER_ERROR_PERIPHERAL_CONFIG;
    }
    return POWER_OK;
}

static PowerResult_t UART_HealthCheck(void)
{
    // Simple health check - ensure UART is initialized
    if (huart3.Instance == NULL) {
        return POWER_ERROR_PERIPHERAL_CONFIG;
    }
    return POWER_OK;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /*AXI clock gating */
  RCC->CKGAENR = 0xE003FFFF;

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0x500B, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    TaskManager_SendHeartbeat(defaultTaskHandle);

    //OPT4048_Init(&opt, &hi2c1);

    //OPT4048_GetCIE(&opt);

    //HAL_Delay(10);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_opt4048_init */
/**
* @brief Function implementing the OPT4048_Init thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_opt4048_init */
void opt4048_init(void *argument)
{
  /* USER CODE BEGIN opt4048_init */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END opt4048_init */
}

/* USER CODE BEGIN Header_FaultManager */
/**
* @brief Function implementing the FaultManagerTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_FaultManager */
void FaultManager(void *argument)
{
  /* USER CODE BEGIN FaultManager */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END FaultManager */
}

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
