/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU6500.h"
#include "PID.h"
#include "vl6180x.h"
#include "timerEncoder.h"
#include "Timer_Time.h"
#include "drive_motor.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "tx.pb.h"
#include "rx.pb.h"
#include "pb_common.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "pb.h"
#include "cobs.h"
#include "semphr.h"
#include "vacuum_esc.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
		.name = "defaultTask",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for vPIDCotrol */
osThreadId_t vPIDCotrolHandle;
const osThreadAttr_t vPIDCotrol_attributes = {
		.name = "vPIDCotrol",
		.stack_size = 1024 * 4,
		.priority = (osPriority_t) osPriorityHigh3,
};
/* Definitions for vSensorUpdate */
osThreadId_t vSensorUpdateHandle;
const osThreadAttr_t vSensorUpdate_attributes = {
		.name = "vSensorUpdate",
		.stack_size = 1024 * 4,
		.priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for vRosTx */
osThreadId_t vRosTxHandle;
const osThreadAttr_t vRosTx_attributes = {
		.name = "vRosTx",
		.stack_size = 1280 * 4,
		.priority = (osPriority_t) osPriorityAboveNormal7,
};
/* Definitions for vRosRx */
osThreadId_t vRosRxHandle;
const osThreadAttr_t vRosRx_attributes = {
		.name = "vRosRx",
		.stack_size = 1280 * 4,
		.priority = (osPriority_t) osPriorityAboveNormal5,
};
/* Definitions for vActuatorCtrl */
osThreadId_t vActuatorCtrlHandle;
const osThreadAttr_t vActuatorCtrl_attributes = {
		.name = "vActuatorCtrl",
		.stack_size = 512 * 4,
		.priority = (osPriority_t) osPriorityNormal7,
};
/* Definitions for vBattery */
osThreadId_t vBatteryHandle;
const osThreadAttr_t vBattery_attributes = {
		.name = "vBattery",
		.stack_size = 512 * 4,
		.priority = (osPriority_t) osPriorityNormal6,
};
/* Definitions for sensorMailboxQueue */
osMessageQueueId_t sensorMailboxQueueHandle;
const osMessageQueueAttr_t sensorMailboxQueue_attributes = {
		.name = "sensorMailboxQueue"
};
/* Definitions for odomMailboxQueue */
osMessageQueueId_t odomMailboxQueueHandle;
const osMessageQueueAttr_t odomMailboxQueue_attributes = {
		.name = "odomMailboxQueue"
};
/* Definitions for batterySocMailboxQueue */
osMessageQueueId_t batterySocMailboxQueueHandle;
const osMessageQueueAttr_t batterySocMailboxQueue_attributes = {
		.name = "batterySocMailboxQueue"
};
/* Definitions for rpmCmdMailboxQueue */
osMessageQueueId_t rpmCmdMailboxQueueHandle;
const osMessageQueueAttr_t rpmCmdMailboxQueue_attributes = {
		.name = "rpmCmdMailboxQueue"
};
/* Definitions for actuatorCmdMailboxQueue */
osMessageQueueId_t actuatorCmdMailboxQueueHandle;
const osMessageQueueAttr_t actuatorCmdMailboxQueue_attributes = {
		.name = "actuatorCmdMailboxQueue"
};
/* Definitions for coulombCntIntgMailboxQueue */
osMessageQueueId_t coulombCntIntgMailboxQueueHandle;
const osMessageQueueAttr_t coulombCntIntgMailboxQueue_attributes = {
		.name = "coulombCntIntgMailboxQueue"
};
/* Definitions for syncMsgMailbox */
osMessageQueueId_t syncMsgMailboxHandle;
const osMessageQueueAttr_t syncMsgMailbox_attributes = {
		.name = "syncMsgMailbox"
};
/* Definitions for CRCMutex */
osMutexId_t CRCMutexHandle;
const osMutexAttr_t CRCMutex_attributes = {
		.name = "CRCMutex"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void *argument);
void PIDTask(void *argument);
void SensorTask(void *argument);
void RosTx(void *argument);
void RosRx(void *argument);
void ActuatorTask(void *argument);
void BatteryTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define COBS_ENCODE_DST_BUF_LEN_MAX(SRC_LEN)            (((SRC_LEN) == 0u) ? 1u : ((SRC_LEN) + (((SRC_LEN) + 253u) / 254u)))
#define COBS_DECODE_DST_BUF_LEN_MAX(SRC_LEN)            (((SRC_LEN) == 0u) ? 0u : ((SRC_LEN) - 1u))

/*
 * For in-place encoding, the source data must be offset in the buffer by
 * the following amount (or more).
 */
#define COBS_ENCODE_SRC_OFFSET(SRC_LEN)                 (((SRC_LEN) + 253u)/254u)
#define MAX_TX_MSG_LEN 254
#define RX_BUF_SIZE 254


typedef struct {		//MPU6500
	float Ax;  // g
	float Ay;
	float Az;
	float Gx;  // deg/s
	float Gy;
	float Gz;
	uint32_t mpu6500TimeStamp_us;
	//VL6180x
	cliffs_status_t cliffs_status;
	uint32_t vl6180xTimeStamp_us;
	//ultrasonic
	//float distance;
	//uint32_t ultrasonicTimeStamp_us;;
}SensorsData_t;
typedef struct {		//odom
	float x;
	float y;
	float theta_rad;
	float linear_x;
	float angular_z;
	uint32_t odomTimeStamp_us;
}odomData_t;
typedef struct{
	float  leftRPMsetpoint;
	float  rightRPMsetpoint;
}setpointRPM_t;

typedef struct{
	float measuredLeftRPM;
	float measuredRightRPM;
}measuredRPM_t;

typedef struct {
	uint8_t brushState;
	uint8_t vacuumState;
}ActuatorCmd_t;
typedef enum {
	Act_off = 0,
	Act_on = 1,
}ActState;
typedef enum {
	vacuum_off = 0,
	vacuum_low= 1,
	vacuum_medium= 2,
	vacuum_high = 3,
}vacuumState;

typedef struct {
	float intialTime;
	float rxTime;
	float txTime;
}syncMsg_t;

typedef struct {
	float linear_x;
	float angular_z;
}cmdVel_t;

typedef struct {
	uint8_t vacuumState;
	uint8_t SideBrushCmd;
}hardwareCmd_t;


// Assuming you trigger this on UART Idle Line Detection or DMA Half/Full Transfer
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

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
	MX_DMA_Init();
	MX_I2C1_Init();
	MX_TIM1_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_USART2_UART_Init();
	MX_ADC1_Init();
	MX_USART6_UART_Init();
	MX_CRC_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
	Time_Init();
	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();
	/* Create the mutex(es) */
	/* creation of CRCMutex */
	CRCMutexHandle = osMutexNew(&CRCMutex_attributes);

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the queue(s) */
	/* creation of sensorMailboxQueue */
	sensorMailboxQueueHandle = osMessageQueueNew (1, sizeof(SensorsData_t), &sensorMailboxQueue_attributes);

	/* creation of odomMailboxQueue */
	odomMailboxQueueHandle = osMessageQueueNew (1, sizeof(odomData_t), &odomMailboxQueue_attributes);

	/* creation of batterySocMailboxQueue */
	batterySocMailboxQueueHandle = osMessageQueueNew (1, sizeof(float), &batterySocMailboxQueue_attributes);

	/* creation of rpmCmdMailboxQueue */
	rpmCmdMailboxQueueHandle = osMessageQueueNew (1, sizeof(setpointRPM_t), &rpmCmdMailboxQueue_attributes);

	/* creation of actuatorCmdMailboxQueue */
	actuatorCmdMailboxQueueHandle = osMessageQueueNew (1, sizeof(ActuatorCmd_t), &actuatorCmdMailboxQueue_attributes);

	/* creation of coulombCntIntgMailboxQueue */
	coulombCntIntgMailboxQueueHandle = osMessageQueueNew (1, sizeof(float), &coulombCntIntgMailboxQueue_attributes);

	/* creation of syncMsgMailbox */
	syncMsgMailboxHandle = osMessageQueueNew (1, sizeof(syncMsg_t), &syncMsgMailbox_attributes);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of defaultTask */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

	/* creation of vPIDCotrol */
	vPIDCotrolHandle = osThreadNew(PIDTask, NULL, &vPIDCotrol_attributes);

	/* creation of vSensorUpdate */
	vSensorUpdateHandle = osThreadNew(SensorTask, NULL, &vSensorUpdate_attributes);

	/* creation of vRosTx */
	vRosTxHandle = osThreadNew(RosTx, NULL, &vRosTx_attributes);

	/* creation of vRosRx */
	vRosRxHandle = osThreadNew(RosRx, NULL, &vRosRx_attributes);

	/* creation of vActuatorCtrl */
	vActuatorCtrlHandle = osThreadNew(ActuatorTask, NULL, &vActuatorCtrl_attributes);

	/* creation of vBattery */
	vBatteryHandle = osThreadNew(BatteryTask, NULL, &vBattery_attributes);

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
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 90;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void)
{

	/* USER CODE BEGIN CRC_Init 0 */

	/* USER CODE END CRC_Init 0 */

	/* USER CODE BEGIN CRC_Init 1 */

	/* USER CODE END CRC_Init 1 */
	hcrc.Instance = CRC;
	if (HAL_CRC_Init(&hcrc) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN CRC_Init 2 */

	/* USER CODE END CRC_Init 2 */

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
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 4;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 999;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 44;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 4294967295;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 449;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 1999;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 65535;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief USART6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART6_UART_Init(void)
{

	/* USER CODE BEGIN USART6_Init 0 */

	/* USER CODE END USART6_Init 0 */

	/* USER CODE BEGIN USART6_Init 1 */

	/* USER CODE END USART6_Init 1 */
	huart6.Instance = USART6;
	huart6.Init.BaudRate = 115200;
	huart6.Init.WordLength = UART_WORDLENGTH_8B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_NONE;
	huart6.Init.Mode = UART_MODE_TX_RX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart6) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART6_Init 2 */

	/* USER CODE END USART6_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
	/* DMA1_Stream6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, ULTRA_EN_Pin|XSHUT_2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, XSHUT_1_Pin|DIR_L_Pin|DIR_R_Pin|S_BRUSH_RELAY1_Pin
			|S_BRUSH_RELAY2_Pin|XSHUT_3_Pin|XSHUT_4_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(VAC_EN_GPIO_Port, VAC_EN_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : ULTRA_EN_Pin XSHUT_2_Pin */
	GPIO_InitStruct.Pin = ULTRA_EN_Pin|XSHUT_2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : XSHUT_1_Pin DIR_L_Pin DIR_R_Pin S_BRUSH_RELAY1_Pin
                           S_BRUSH_RELAY2_Pin XSHUT_3_Pin XSHUT_4_Pin */
	GPIO_InitStruct.Pin = XSHUT_1_Pin|DIR_L_Pin|DIR_R_Pin|S_BRUSH_RELAY1_Pin
			|S_BRUSH_RELAY2_Pin|XSHUT_3_Pin|XSHUT_4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : VAC_EN_Pin */
	GPIO_InitStruct.Pin = VAC_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(VAC_EN_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;

		vTaskNotifyGiveFromISR(vRosTxHandle, &xHigherPriorityTaskWoken);

		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART2)
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;

		vTaskNotifyGiveFromISR(vRosRxHandle, &xHigherPriorityTaskWoken);

		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}


///* Ensure you have this to handle the "Serial Flood" safely */
//void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
//{
//    if (huart->Instance == USART2)
//    {
//        // 1. Stop the current broken DMA transfer
//        HAL_UART_AbortReceive(huart);
//
//        // 2. Clear buffers/indices if necessary
//        // ... (Optional: reset your read_ptr and msg_idx here) ...
//
//        // 3. Restart the DMA to be ready for the next fresh frame
//        HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t*)rx_dma_buffer, RX_BUF_SIZE);
//    }
//}



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
		osDelay(1);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_PIDTask */
/**
 * @brief Function implementing the vPIDCotrol thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_PIDTask */
void PIDTask(void *argument)
{
	/* USER CODE BEGIN PIDTask */
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xFrequency = pdMS_TO_TICKS(13);
	uint8_t u8CycleCounter = 0;
	//	static uint32_t pid_last_micros = 0;
	float dt = 0.013f;
	//	pid_last_micros = __HAL_TIM_GET_COUNTER(&htim2);

	volatile bool isEmergencyStopped = false;
	static setpointRPM_t setpointRPM = {0};
	static setpointRPM_t tempSetpoint = {0};
	static measuredRPM_t measuredRPM = {0};
	static odomData_t xCurrentOdometry;
	static PIDController leftPIDController;
	static PIDController rightPIDController;
	//odom init
	Odometry_t odom;
	Odom_Init(&odom, &htim4, &htim3);
	//left pid
	PIDController_Init(&leftPIDController);
	leftPIDController.Kd = 0;
	leftPIDController.Ki = 0.03;
	leftPIDController.Kp = 1.5;
	leftPIDController.kff = 6;
	leftPIDController.T = 0;
	leftPIDController.limMax = 1000;
	leftPIDController.limMin = -1000;
	leftPIDController.limMaxInt= 200;
	leftPIDController.limMinInt = -200;

	//right pid
	PIDController_Init(&rightPIDController);
	rightPIDController.Kd = 0;
	rightPIDController.Ki = 0.03;
	rightPIDController.Kp = 1.5;
	rightPIDController.kff = 6;
	rightPIDController.T = 0;
	rightPIDController.limMax = 1000;
	rightPIDController.limMin = -1000;
	rightPIDController.limMaxInt= 200;
	rightPIDController.limMinInt = -200;
	//drive motor init
	DriveMotor_Init();

	for(;;) {

		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		//		float dt = Time_GetDeltaTime(&pid_last_micros);
		Odom_Update(&odom , dt);
		// Encoders (RPM)
		xCurrentOdometry.x = odom.x;
		xCurrentOdometry.y = odom.y;
		xCurrentOdometry.theta_rad = odom.theta_rad;
		xCurrentOdometry.linear_x= odom.linear_x;
		xCurrentOdometry.angular_z= odom.angular_z;
		xCurrentOdometry.odomTimeStamp_us = Time_GetStatusMicros();
		measuredRPM.measuredLeftRPM = odom.measured_RPM_left;
		measuredRPM.measuredRightRPM = odom.measured_RPM_right;
		//recieve RPM setpoint for each  wheel
		if ((xQueueReceive(rpmCmdMailboxQueueHandle, &tempSetpoint, pdMS_TO_TICKS(0)))==pdPASS) {
			setpointRPM.leftRPMsetpoint = tempSetpoint.leftRPMsetpoint;
			setpointRPM.rightRPMsetpoint = tempSetpoint.rightRPMsetpoint;
		}

		if (ulTaskNotifyTake(pdTRUE, 0)) {
			isEmergencyStopped = false;
		}
//		if (isEmergencyStopped == true) {
//			setpointRPM.leftRPMsetpoint = 0;
//			setpointRPM.rightRPMsetpoint = 0;
//			isEmergencyStopped = false;
//		}
		// PID
		leftPIDController.T  = dt;
		rightPIDController.T = dt;
		float fPwmLeft = PIDController_UpdateWithFeedForward(&leftPIDController, setpointRPM.leftRPMsetpoint, measuredRPM.measuredLeftRPM);
		float fPwmRight =PIDController_UpdateWithFeedForward(&rightPIDController, setpointRPM.rightRPMsetpoint, measuredRPM.measuredRightRPM);

		DriveMotor_SetPWM(MOTOR_LEFT, fPwmLeft);
		DriveMotor_SetPWM(MOTOR_RIGHT,fPwmRight);
		u8CycleCounter++;
		if (u8CycleCounter >= 2) {
			xQueueOverwrite(odomMailboxQueueHandle, &xCurrentOdometry);
			u8CycleCounter = 0;
		}
	}
	// send to RosTx each 25ms

	/* USER CODE END PIDTask */
}

/* USER CODE BEGIN Header_SensorTask */
/**
 * @brief Function implementing the vSensorUpdate thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_SensorTask */
void SensorTask(void *argument)
{
	/* USER CODE BEGIN SensorTask */
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xFrequency = pdMS_TO_TICKS(50);

	static uint32_t sensors_last_micros = 0;

	sensors_last_micros = __HAL_TIM_GET_COUNTER(&htim2);
	uint32_t readMpu6500TimeStamp_us = 0;
	uint32_t readVl6180xTimeStamp_us = 0;
	static SensorsData_t SensorsData = {0};
	uint8_t tofStatus = 0;
	uint8_t cliffStatus = 0;
	//float fCurrentAmps = 0.0f;
	static MPU6500_t mpu6500 = {0};
	static allDistanceTofData_t tofAllData = {0};
	static cliffs_status_t allCliffStatus = {0};
		MPU6500_Init(&hi2c1, &mpu6500);
		tofStatus = vl6180x_init(&hi2c1);

	for(;;) {
		// Wait until the next execution cycle (50 ms periodic)
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		// 1. Read raw Ax, Ay, Az, Gx, Gy, Gz from MPU6500 via I2C
		float dt = Time_GetDeltaTime(&sensors_last_micros);
		readMpu6500TimeStamp_us = Time_GetStatusMicros();
		MPU6500_Read_All(&hi2c1, &mpu6500, dt);
		// 2. Read cliff detection status from VL680x ToF sensors via I2C
		if (tofStatus == init_success) {
			readVl6180xTimeStamp_us= Time_GetStatusMicros();
			readAllvl6180xData(&tofAllData);
			cliffStatus =updateCliffStatus(&tofAllData, &allCliffStatus);
			if (cliffStatus == cliff_detected) {
				xTaskNotifyGive(vPIDCotrolHandle);
			}
		}
		SensorsData.Ax = mpu6500.Ax;
		SensorsData.Ay = mpu6500.Ay;
		SensorsData.Az = mpu6500.Az;
		SensorsData.Gx = mpu6500.Gx;
		SensorsData.Gy = mpu6500.Gy;
		SensorsData.Gz = mpu6500.Gz;
		SensorsData.cliffs_status.cliff1= allCliffStatus.cliff1;
		SensorsData.cliffs_status.cliff2= allCliffStatus.cliff2;
		SensorsData.cliffs_status.cliff3= allCliffStatus.cliff3;
		SensorsData.cliffs_status.cliff4= allCliffStatus.cliff4;
		SensorsData.mpu6500TimeStamp_us = readMpu6500TimeStamp_us;
		SensorsData.vl6180xTimeStamp_us = readVl6180xTimeStamp_us;
		//		// 3. Read obstacle distance from SR05 Ultrasonic sensor via USART (Pseudo-code)
		//		xRawSensors.distance = sudo_sr05_get_distance();

		// 4. Sample the current sensor output via ADC (STM32 HAL)
		//		HAL_ADC_Start(&hadc1);
		//		if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
		//			fCurrentAmps = sudo_convert_adc_to_amps(HAL_ADC_GetValue(&hadc1));
		//		}
		//HAL_ADC_Stop(&hadc1);

		// 5. Update shared queues using overwrite mode
		xQueueOverwrite(sensorMailboxQueueHandle, &SensorsData);
		//		xQueueOverwrite(xqCoulombIntegrator, &fCurrentAmps);

	}
	/* USER CODE END SensorTask */
}

/* USER CODE BEGIN Header_RosTx */
/**
 * @brief Function implementing the vRosTx thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_RosTx */
void RosTx(void *argument)
{
	/* USER CODE BEGIN RosTx */
	// Structures for FreeRTOS Queues
	uint8_t battarySoc = 70;
	uint8_t chargingState = 0;
	static syncMsg_t syncMsgSend = {0};
	static SensorsData_t sensorDataSend= {0};
	static odomData_t odomDataSend= {0};

	static uint8_t cobs_buffer[COBS_ENCODE_DST_BUF_LEN_MAX(MAX_TX_MSG_LEN)+1];
	static uint8_t txproto_buf[180];
	memset(txproto_buf, 0, sizeof(txproto_buf));

	for(;;)
	{
		// 1. Block until a message is received from vPIDControl
		if (xQueueReceive(odomMailboxQueueHandle, &odomDataSend, portMAX_DELAY) == pdTRUE)
		{
			memset(txproto_buf, 0, sizeof(txproto_buf));
			TxMsg txMessage = TxMsg_init_zero; // Nanopb struct
			// 2. Read the latest Battery SoC (Overwrite Queue)
			// xQueuePeek is used so the value remains in the queue for others,
			// or xQueueReceive with wait=0 if it's a size-1 overwrite queue.
			xQueueReceive(sensorMailboxQueueHandle, &sensorDataSend, (TickType_t)0);
			xQueueReceive(syncMsgMailboxHandle, &syncMsgSend, (TickType_t)0);

			// 3. Populate Protobuf Message
			// Add other fields...
			txMessage.has_odom = true;
			txMessage.has_imu= true;
			txMessage.has_sync_msg_tx= true;
			txMessage.sync_msg_tx.stm32_tx_time_stamp_us= Time_GetStatusMicros();
			txMessage.sync_msg_tx.pi_initial_time_stamp_us= syncMsgSend.intialTime;
			txMessage.sync_msg_tx.stm32_rx_time_stamp_us =syncMsgSend.rxTime;
			txMessage.has_tofArray= true;
			txMessage.has_battery= true;
			txMessage.odom.has_x = true;
			txMessage.odom.has_y = true;
			txMessage.odom.has_yaw= true;
			txMessage.odom.has_linear_x = true;
			txMessage.odom.has_angular_z = true;
			txMessage.odom.has_odom_time_stamp_us= true;
			txMessage.odom.x = odomDataSend.x;
			txMessage.odom.y = odomDataSend.y;
			txMessage.odom.yaw = odomDataSend.theta_rad;
			txMessage.odom.linear_x = odomDataSend.linear_x;
			txMessage.odom.angular_z = odomDataSend.angular_z;
			txMessage.odom.odom_time_stamp_us = odomDataSend.odomTimeStamp_us;
			txMessage.imu.has_ax = true;
			txMessage.imu.has_ay = true;
			txMessage.imu.has_az = true;
			txMessage.imu.has_gx = true;
			txMessage.imu.has_gy = true;
			txMessage.imu.has_gz = true;
			txMessage.imu.ax=sensorDataSend.Ax;
			txMessage.imu.ay=sensorDataSend.Ay;
			txMessage.imu.az=sensorDataSend.Az;
			txMessage.imu.gx=sensorDataSend.Gx;
			txMessage.imu.gy=sensorDataSend.Gy;
			txMessage.imu.gz=sensorDataSend.Gz;
			txMessage.imu.has_mpu6500_time_stamp_us= true;
			txMessage.imu.mpu6500_time_stamp_us=sensorDataSend.mpu6500TimeStamp_us;
			txMessage.battery.has_StateOfCharge = true;
			txMessage.battery.has_ChargingState= true;
			txMessage.battery.StateOfCharge = battarySoc;
			txMessage.battery.ChargingState= chargingState;
			txMessage.tofArray.has_cliff1 =true;
			txMessage.tofArray.has_cliff2 =true;
			txMessage.tofArray.has_cliff3 =true;
			txMessage.tofArray.has_cliff4 =true;
			txMessage.tofArray.cliff1 =sensorDataSend.cliffs_status.cliff1;
			txMessage.tofArray.cliff2=sensorDataSend.cliffs_status.cliff2;
			txMessage.tofArray.cliff3=sensorDataSend.cliffs_status.cliff3;
			txMessage.tofArray.cliff4=sensorDataSend.cliffs_status.cliff4;
			txMessage.tofArray.has_tof_time_stamp_us=true;
			txMessage.tofArray.tof_time_stamp_us=sensorDataSend.vl6180xTimeStamp_us;

			// 4. Encode with Nanopb
			pb_ostream_t txStream = pb_ostream_from_buffer(txproto_buf, sizeof(txproto_buf));
			if (pb_encode(&txStream, TxMsg_fields, &txMessage))
			{
				size_t encoded_len = txStream.bytes_written;

				// 5. Calculate and Append CRC32
				xSemaphoreTake(CRCMutexHandle,portMAX_DELAY);
				uint32_t crc_val = HAL_CRC_Calculate(&hcrc, (uint32_t*)txproto_buf, (encoded_len + 3) / 4);
				xSemaphoreGive(CRCMutexHandle);

				memcpy(&txproto_buf[encoded_len], &crc_val, 4);
				encoded_len += 4;

				// 6. COBS Encoding
				cobs_encode_result cobs_res = cobs_encode(cobs_buffer, sizeof(cobs_buffer), txproto_buf, encoded_len);

				if (cobs_res.status == COBS_ENCODE_OK)
				{
					// Add standard COBS delimiter
					cobs_buffer[cobs_res.out_len] = 0x00;

					// 7. Execute Data Transmission via UART DMA (Normal Mode)
					// We assume a mechanism (like a semaphore) exists to ensure UART is ready before sending
					HAL_UART_Transmit_DMA(&huart2, cobs_buffer, cobs_res.out_len + 1);
					ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
				}
			}
		}
	}
	/* USER CODE END RosTx */
}

/* USER CODE BEGIN Header_RosRx */
/**
 * @brief Function implementing the vRosRx thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_RosRx */
void RosRx(void *argument)
{
	/* USER CODE BEGIN RosRx */
	static uint32_t received_crc, calculated_crc;
	static volatile uint8_t rx_dma_buffer[RX_BUF_SIZE]; /* Added volatile for hardware updates */
	static uint8_t msg_extract_buffer[RX_BUF_SIZE];
	static uint8_t rx_decoded_buffer[RX_BUF_SIZE];
	static uint16_t msg_idx = 0;
	static uint16_t read_ptr = 0;
	static hardwareCmd_t hwCmd = {0};
	static cmdVel_t cmdVel = {0};
	static setpointRPM_t tempsetpointRPM= {0};
	static setpointRPM_t setpointRPM= {0};
	static syncMsg_t syncMsg = {0};
	// Start Circular DMA Reception (Runs continuously in background)
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t*)rx_dma_buffer, sizeof(rx_dma_buffer));
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
	/* Infinite loop */
	for(;;)
	{
		// 1. Block and wait for Scheduler Interrupt from DMA callback
		if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) {

			syncMsg.rxTime = Time_GetStatusMicros();
			uint16_t write_ptr = RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);

			/* Stream through newly written bytes from the DMA peripheral */
			while (read_ptr != write_ptr)
			{
				uint8_t byte = rx_dma_buffer[read_ptr];

				if (byte == 0x00)
				{
					/* Frame delimiter spotted! Process the buffered message fragment */
					if (msg_idx > 0)
					{
						RxMsg rxMessage = RxMsg_init_zero;
						/* 1. Undo COBS Framing */
						cobs_decode_result decode_rslt = cobs_decode(rx_decoded_buffer, sizeof(rx_decoded_buffer), msg_extract_buffer, msg_idx);

						if (decode_rslt.status == COBS_DECODE_OK)
						{
							size_t payload_len = decode_rslt.out_len;

							/* 2. Validate Framing Content and Integrity via Checksum */
							if (payload_len > 4)
							{
								size_t pb_data_len = payload_len - 4;

								/* Extract appended hardware checksum byte package */
								// Removed unneeded Mutex here
								memcpy(&received_crc, &rx_decoded_buffer[pb_data_len], 4);

								/* Calculate Expected CRC32 matching transmitter specifications */
								// Moved Mutex here: Protect the shared hardware CRC peripheral
								xSemaphoreTake(CRCMutexHandle, portMAX_DELAY);
								calculated_crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)rx_decoded_buffer, (pb_data_len + 3) / 4);
								xSemaphoreGive(CRCMutexHandle);

								if (calculated_crc == received_crc)
								{
									/* 3. Safe Parsing of Internal Proto Messages */

									pb_istream_t rx_stream = pb_istream_from_buffer(rx_decoded_buffer, pb_data_len);

									if (pb_decode(&rx_stream, RxMsg_fields, &rxMessage))
									{
										/* Packet successfully verified! Process rxMessagePayload attributes here */
										if (rxMessage.has_cmd_vel) {
											cmdVel.linear_x = rxMessage.cmd_vel.linear_x;
											cmdVel.angular_z = rxMessage.cmd_vel.angular_z;
											tempsetpointRPM.leftRPMsetpoint=(15.0f * ((2.0f * cmdVel.linear_x) - (cmdVel.angular_z* WHEELBASE))) /PI_f ;
											tempsetpointRPM.rightRPMsetpoint=(15.0f * ((2.0f * cmdVel.linear_x) + (cmdVel.angular_z* WHEELBASE))) /PI_f ;
											setpointRPM.leftRPMsetpoint = tempsetpointRPM.leftRPMsetpoint;
											setpointRPM.rightRPMsetpoint = tempsetpointRPM.rightRPMsetpoint;
											xQueueOverwrite(rpmCmdMailboxQueueHandle, &setpointRPM);
										}
										if (rxMessage.has_sync_msg_rx) {
											syncMsg.intialTime = rxMessage.sync_msg_rx.pi_initial_time_stamp_us;
											syncMsg.rxTime= Time_GetStatusMicros();
											xQueueOverwrite(syncMsgMailboxHandle, &syncMsg);
										}
										if (rxMessage.has_hardware_cmd) {
											hwCmd.vacuumState =rxMessage.hardware_cmd.vacuum_state;
											hwCmd.SideBrushCmd =rxMessage.hardware_cmd.side_brushcmd;
											xQueueOverwrite(actuatorCmdMailboxQueueHandle, &hwCmd);
										}

									}
								}
							}
						}
					}
					msg_idx = 0; /* Clear index to sync for next frame */
				}
				else
				{
					/* Push bytes into processing frame array */
					if (msg_idx < RX_BUF_SIZE)
					{
						msg_extract_buffer[msg_idx++] = byte;
					}
					else
					{
						/* Overflow protection: frame exceeded limits without zero-delimiter. Flush. */
						msg_idx = 0;
					}
				}

				/* Shift software reading pointer circularly */
				read_ptr = (read_ptr + 1) % RX_BUF_SIZE;
			}
		}
	}

	/* USER CODE END RosRx */
}

/* USER CODE BEGIN Header_ActuatorTask */
/**
 * @brief Function implementing the vActuatorCtrl thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_ActuatorTask */
void ActuatorTask(void *argument)
{
	/* USER CODE BEGIN ActuatorTask */
	//#define S_BRUSH_RELAY1_Pin GPIO_PIN_14
	//#define S_BRUSH_RELAY1_GPIO_Port GPIOB
	//#define S_BRUSH_RELAY2_Pin GPIO_PIN_15
	//#define S_BRUSH_RELAY2_GPIO_Port GPIOB

	hardwareCmd_t tempHwCmd = {0};
	hardwareCmd_t HwCmd = {0};
	Vacuum_ESC_Init();
	HAL_GPIO_WritePin(S_BRUSH_RELAY1_GPIO_Port, S_BRUSH_RELAY1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(S_BRUSH_RELAY2_GPIO_Port, S_BRUSH_RELAY2_Pin, GPIO_PIN_SET);

	for(;;) {
		// Remain in a low-power blocked state until a configuration command is dispatched
		if (xQueueReceive(actuatorCmdMailboxQueueHandle, &tempHwCmd, portMAX_DELAY) == pdPASS) {
			HwCmd.vacuumState = tempHwCmd.vacuumState;
			HwCmd.SideBrushCmd = tempHwCmd.SideBrushCmd;
			Vacuum_ESC_SetState(HwCmd.vacuumState);
			if (HwCmd.SideBrushCmd == Act_on) {
				HAL_GPIO_WritePin(S_BRUSH_RELAY1_GPIO_Port, S_BRUSH_RELAY1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(S_BRUSH_RELAY2_GPIO_Port, S_BRUSH_RELAY2_Pin, GPIO_PIN_RESET);
			}
			else {
				HAL_GPIO_WritePin(S_BRUSH_RELAY1_GPIO_Port, S_BRUSH_RELAY1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(S_BRUSH_RELAY2_GPIO_Port, S_BRUSH_RELAY2_Pin, GPIO_PIN_SET);

			}

		}
	}

	/* USER CODE END ActuatorTask */
}

/* USER CODE BEGIN Header_BatteryTask */
/**
 * @brief Function implementing the vBattery thread.
 * @param argument: Not used
 * @retval None
 */

/* USER CODE END Header_BatteryTask */
void BatteryTask(void *argument)
{
	/* USER CODE BEGIN BatteryTask */
	/* Infinite loop */
	//	float fCurrentReading;
	//	uint8_t u8SocPercent;

	for(;;) {
		//		// Block until the sensor task pushes an updated current integration reading
		//		if (xQueueReceive(xqCoulombIntegrator, &fCurrentReading, portMAX_DELAY) == pdPASS) {
		//
		//			// 1. Evaluate State of Charge using an OCV Lookup Table and current integration (Pseudo-code)
		//			u8SocPercent = sudo_calculate_soc(fCurrentReading, sudo_get_bus_voltage());
		//
		//			// 2. Export updated SoC percentage to the telemetry stream queue
		//			xQueueOverwrite(xqBatterySoc, &u8SocPercent);
		//		}

	}
}
/* USER CODE END BatteryTask */


/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM10 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM10)
	{
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
