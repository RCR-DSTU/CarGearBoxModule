/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int8.h>
#include <diagnostic_msgs/msg/diagnostic_array.h>

#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	float P_k;
	float I_k;
	float D_k;
	float current;
	float target;
	char pid_on;
	float max_sum_error;
	float error_end;
	float output;
	float min_output;
	float max_output;
}PID;
PID Regulator[2];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
  std_msgs__msg__Int8 msgTransmission;
  diagnostic_msgs__msg__DiagnosticArray msgDiagnostic;

/* USER CODE END Variables */
/* Definitions for xuROS */
osThreadId_t xuROSHandle;
const osThreadAttr_t xuROS_attributes = {
  .name = "xuROS",
  .stack_size = 2000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for xTransmission */
osThreadId_t xTransmissionHandle;
const osThreadAttr_t xTransmission_attributes = {
  .name = "xTransmission",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for xRestart */
osThreadId_t xRestartHandle;
const osThreadAttr_t xRestart_attributes = {
  .name = "xRestart",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for xRestartQueue */
osMessageQueueId_t xRestartQueueHandle;
const osMessageQueueAttr_t xRestartQueue_attributes = {
  .name = "xRestartQueue"
};
/* Definitions for xTransmissionQueue */
osMessageQueueId_t xTransmissionQueueHandle;
const osMessageQueueAttr_t xTransmissionQueue_attributes = {
  .name = "xTransmissionQueue"
};
/* Definitions for xStateQueue */
osMessageQueueId_t xStateQueueHandle;
const osMessageQueueAttr_t xStateQueue_attributes = {
  .name = "xStateQueue"
};
/* Definitions for xRegulator */
osTimerId_t xRegulatorHandle;
const osTimerAttr_t xRegulator_attributes = {
  .name = "xRegulator"
};


/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void uROSTask(void *argument);
void TransmissionTask(void *argument);
void RestartTask(void *argument);
void Regulator_Callback(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

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

  /* Create the semaphores(s) */
  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of xRegulator */
  xRegulatorHandle = osTimerNew(Regulator_Callback, osTimerPeriodic, NULL, &xRegulator_attributes);
  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of xRestartQueue */
  xRestartQueueHandle = osMessageQueueNew (1, sizeof(bool), &xRestartQueue_attributes);

  /* creation of xTransmissionQueue */
  xTransmissionQueueHandle = osMessageQueueNew (1, sizeof(uint8_t), &xTransmissionQueue_attributes);

  /* creation of xStateQueue */
  xStateQueueHandle = osMessageQueueNew (1, sizeof(uint8_t), &xStateQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of xuROS */
  xuROSHandle = osThreadNew(uROSTask, NULL, &xuROS_attributes);

  /* creation of xTransmission */
  xTransmissionHandle = osThreadNew(TransmissionTask, NULL, &xTransmission_attributes);

  /* creation of xRestart */
  xRestartHandle = osThreadNew(RestartTask, NULL, &xRestart_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_uROSTask */
/**
  * @brief  Function implementing the xuROS thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_uROSTask */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void   microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

void uROSTask(void *argument)
{
  /* USER CODE BEGIN uROSTask */
	 rmw_uros_set_custom_transport(
	    true,
	    (void *) &huart1,
	    cubemx_transport_open,
	    cubemx_transport_close,
	    cubemx_transport_write,
	    cubemx_transport_read);

	  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
	  freeRTOS_allocator.allocate = microros_allocate;
	  freeRTOS_allocator.deallocate = microros_deallocate;
	  freeRTOS_allocator.reallocate = microros_reallocate;
	  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

	  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
	      printf("Error on default allocators (line %d)\n", __LINE__);
	  }

	  // micro-ROS app

	  rcl_publisher_t publisher;
	  std_msgs__msg__Int8 msg;
	  rclc_support_t support;
	  rcl_allocator_t allocator;
	  rcl_node_t node;

	  allocator = rcl_get_default_allocator();

	  //create init_options
	  rclc_support_init(&support, 0, NULL, &allocator);

	  // create node
	  rclc_node_init_default(&node, "cubemx_node", "", &support);

	  rclc_publisher_init_default(
	    &publisher,
	    &node,
	    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
	    "cubemx_publisher");
	  // 100 Hz timer
	  osTimerStart(xRegulatorHandle, (10/portTICK_RATE_MS));

  /* Infinite loop */
  for(;;)
  {
	    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
	    if (ret != RCL_RET_OK)
	    {
	      printf("Error publishing (line %d)\n", __LINE__);
	    }

	    //msg.data++;
	    osDelay(10);
  }
  /* USER CODE END uROSTask */
}

/* USER CODE BEGIN Header_TransmissionTask */
/**
* @brief Function implementing the xTransmission thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TransmissionTask */
void TransmissionTask(void *argument)
{
  /* USER CODE BEGIN TransmissionTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END TransmissionTask */
}

/* USER CODE BEGIN Header_RestartTask */
/**
* @brief Function implementing the xRestart thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RestartTask */
void RestartTask(void *argument)
{
  /* USER CODE BEGIN RestartTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END RestartTask */
}

void ParseEncoderData(void) {

}
/* Regulator_Callback function */
void Regulator_Callback(void *argument)
{
  /* USER CODE BEGIN Regulator_Callback */




  /* USER CODE END Regulator_Callback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */




/* USER CODE END Application */

