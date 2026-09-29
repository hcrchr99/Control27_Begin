/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId App_Remote_TaskHandle;
osThreadId App_ChassisTaskHandle;
osThreadId App_Grab_TaskHandle;
osThreadId App_Sense_TaskHandle;
osThreadId App_Daemon_TaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartApp_Remote_Task(void const * argument);
void StartApp_Chassis_Task(void const * argument);
void StartApp_Grab_Task(void const * argument);
void StartApp_Sense_Task(void const * argument);
void StartApp_Daemon_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of App_Remote_Task */
  osThreadDef(App_Remote_Task, StartApp_Remote_Task, osPriorityHigh, 0, 512);
  App_Remote_TaskHandle = osThreadCreate(osThread(App_Remote_Task), NULL);

  /* definition and creation of App_ChassisTask */
  osThreadDef(App_ChassisTask, StartApp_Chassis_Task, osPriorityAboveNormal, 0, 1024);
  App_ChassisTaskHandle = osThreadCreate(osThread(App_ChassisTask), NULL);

  /* definition and creation of App_Grab_Task */
  osThreadDef(App_Grab_Task, StartApp_Grab_Task, osPriorityNormal, 0, 768);
  App_Grab_TaskHandle = osThreadCreate(osThread(App_Grab_Task), NULL);

  /* definition and creation of App_Sense_Task */
  osThreadDef(App_Sense_Task, StartApp_Sense_Task, osPriorityBelowNormal, 0, 256);
  App_Sense_TaskHandle = osThreadCreate(osThread(App_Sense_Task), NULL);

  /* definition and creation of App_Daemon_Task */
  osThreadDef(App_Daemon_Task, StartApp_Daemon_Task, osPriorityLow, 0, 768);
  App_Daemon_TaskHandle = osThreadCreate(osThread(App_Daemon_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartApp_Remote_Task */
/**
  * @brief  Function implementing the App_Remote_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartApp_Remote_Task */
__weak void StartApp_Remote_Task(void const * argument)
{
  /* USER CODE BEGIN StartApp_Remote_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartApp_Remote_Task */
}

/* USER CODE BEGIN Header_StartApp_Chassis_Task */
/**
* @brief Function implementing the App_Chassis_Tas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartApp_Chassis_Task */
__weak void StartApp_Chassis_Task(void const * argument)
{
  /* USER CODE BEGIN StartApp_Chassis_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartApp_Chassis_Task */
}

/* USER CODE BEGIN Header_StartApp_Grab_Task */
/**
* @brief Function implementing the App_Grab_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartApp_Grab_Task */
__weak void StartApp_Grab_Task(void const * argument)
{
  /* USER CODE BEGIN StartApp_Grab_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartApp_Grab_Task */
}

/* USER CODE BEGIN Header_StartApp_Sense_Task */
/**
* @brief Function implementing the App_Sense_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartApp_Sense_Task */
__weak void StartApp_Sense_Task(void const * argument)
{
  /* USER CODE BEGIN StartApp_Sense_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartApp_Sense_Task */
}

/* USER CODE BEGIN Header_StartApp_Daemon_Task */
/**
* @brief Function implementing the App_Daemon_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartApp_Daemon_Task */
__weak void StartApp_Daemon_Task(void const * argument)
{
  /* USER CODE BEGIN StartApp_Daemon_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartApp_Daemon_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

