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
#include "transmission.h"
#include "pathCoordination.h"
#include "regulator.h"
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
PID Regulator[2];
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TEST	1
const char *subscriber_name = "TransmissionPosition";
const char *publisher_name = "CarStateResponse";
const int timeout_ms = 250;
const uint8_t attempts = 5;
#define uROS_delayMs				100
#define Regulator_periodMs		    20
#define Tracking_periodMs			40
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
  std_msgs__msg__Int8 msgTransmission;
  diagnostic_msgs__msg__DiagnosticArray msgDiagnostic;
  rcl_ret_t ret;
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
  .name = "xRegulator",
};

osTimerId_t xTrackRegulatorHandle;
const osTimerAttr_t xTrack_attributes = {
		.name = "xTracking",
};
/* Definitions for xRestarthandle */
osSemaphoreId_t xRestartSemaphore;
const osSemaphoreAttr_t xRestartSemaphore_attributes = {
  .name = "xRestartSemaphore",
};

#if(TEST == 1)
osThreadId_t xTestHandle;
const osThreadAttr_t xTest_attributes = {
		.name = "xTest",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityNormal,
};
#endif
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void uROSTask(void *argument);
void TransmissionTask(void *argument);
void RestartTask(void *argument);
void TestTask(void *argument);
void Regulator_Callback(void *argument);
void Track_Callback(void *argument);

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
  xTrackRegulatorHandle = osTimerNew(Track_Callback, osTimerPeriodic, NULL, &xTrack_attributes);
  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of xRestartSemaphore */

  xRestartHandle = osSemaphoreNew(1, 1, &xRestartSemaphore_attributes);

  /* creation of xTransmissionQueue */
  xTransmissionQueueHandle = osMessageQueueNew (1, sizeof(uint8_t), &xTransmissionQueue_attributes);

  /* creation of xStateQueue */
  xStateQueueHandle = osMessageQueueNew (1, sizeof(uint8_t), &xStateQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of xuROS */
#if(TEST == 0)
  xuROSHandle = osThreadNew(uROSTask, NULL, &xuROS_attributes);
#endif /*TEST*/
  /* creation of xTransmission */
#if(TEST == 0)
  xTransmissionHandle = osThreadNew(TransmissionTask, NULL, &xTransmission_attributes);
#endif /*TEST*/
  /* creation of xRestart */
#if(TEST == 0)
  xRestartHandle = osThreadNew(RestartTask, NULL, &xRestart_attributes);
#endif /*TEST*/
#if (TEST == 1)
  xTestHandle = osThreadNew(TestTask, NULL, &xTest_attributes);
#endif /*TEST*/
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

void ConstructAnswer(char* message, uint32_t millisec, uint32_t nanosec)
{
	msgDiagnostic.header.stamp.nanosec = nanosec;
	msgDiagnostic.header.stamp.sec = millisec / 1000;
	msgDiagnostic.header.frame_id.data = message;
}

/*!
 * subscription callback function
 * @param msgin new value data from TransmissionPosition subscriber
 */
void subscription_callback(const void* msgin)
{
	const std_msgs__msg__Int8 * msg = (std_msgs__msg__Int8 *)msgin;
	msgTransmission = *msg;
	printf("Received msg from topic \r\n");
}

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
	  rcl_subscription_t subscriber;

	  rclc_support_t support;
	  rcl_allocator_t allocator;
	  rcl_node_t node;

	  allocator = rcl_get_default_allocator();

	  //create init_options
	  rclc_support_init(&support, 0, NULL, &allocator);

	  // create node
	  rclc_node_init_default(&node, "TransmissionModule", "", &support);

	  ret = rclc_publisher_init_default(
	    &publisher,
	    &node,
	    ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticArray),
	    publisher_name);

	  if(RCL_RET_OK != ret) {
		  osSemaphoreRelease(xRestartSemaphore);
	  }

	  ret = rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
		subscriber_name);

	  rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	  ret = rclc_executor_init(&executor, &support.context, 1, &allocator);

	  ret = rclc_executor_add_subscription(&executor, &subscriber, &msgDiagnostic,
			  &subscription_callback, ON_NEW_DATA);
	  if(RCL_RET_OK != ret) { osSemaphoreRelease(xRestartSemaphore); }

	  int64_t time_ns, time_ms = 0;

  /* Infinite loop */
  for(;;)
  {
	  /*!
	   * 1) check ping agent result
	   * 2) check time sync result
	   * 3) get new value from subscriber
	   * 4) publish current value
	   * 5) if new data is new position of transmission -> Start performer task
	   */
	  	rmw_ret_t ping_result = rmw_uros_ping_agent(timeout_ms,attempts);
	  	if(RMW_RET_OK != ping_result) { osSemaphoreRelease(xRestartSemaphore); }

	  	rmw_uros_sync_session(timeout_ms);
	  	if(!rmw_uros_epoch_synchronized()) { osSemaphoreRelease(xRestartSemaphore); }
	  	time_ms = rmw_uros_epoch_millis();
	  	time_ns = rmw_uros_epoch_nanos();

	  	if(RCL_RET_OK != ret) { osSemaphoreRelease(xRestartSemaphore); }
	  	rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));

	  	/* Send success message with publisher */
	  	ConstructAnswer("[Transmission Module] OK",time_ms, time_ns);
	  	ret = rcl_publish(&publisher, &msgDiagnostic, NULL);

	  	/* Delay after uros-app cycle*/
	  	osDelay( uROS_delayMs / portTICK_RATE_MS);
  }

  ret = rcl_publisher_fini(&publisher,&node);
  ret = rcl_subscription_fini(&subscriber,&node);
  osThreadExit();
  /* USER CODE END uROSTask */
}

#if(TEST == 1)
/* USER CODE BEGIN Header_TestTask */
void TestTask(void *argument)
{
	PID_init();
	for(;;)
	{



	}
	osThreadExit();
}
/* USER CODE END Header_TestTask */
#endif
/* USER CODE BEGIN Header_TransmissionTask */
/**
* @brief Function implementing the xTransmission thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TransmissionTask */
void TransmissionTask(void *argument)
{
	int8_t *target;
  /* USER CODE BEGIN TransmissionTask */
  /* Infinite loop */
  for(;;)
  {
	  osMessageQueueGet(xTransmissionQueueHandle, &target, NULL, portMAX_DELAY);

//	  if(*target != Transmission.Current_flag)
//	  {
//		  /* Start track manager*/
//		  osTimerStart(xTrackRegulatorHandle,Tracking_periodMs / portTICK_RATE_MS);
//	  }
  }
  osThreadExit();
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
	  osSemaphoreAcquire(xRestartSemaphore, portMAX_DELAY);
		  /* Stop current tasks */
		  osThreadSuspend(xuROSHandle);
		  osThreadSuspend(xTransmissionHandle);

		  __NOP();

		  /* delete tasks */
		  osThreadTerminate(xuROSHandle);
		  osThreadTerminate(xTransmissionHandle);

		  /* reset current queues */
		  osMessageQueueReset(xTransmissionQueueHandle);
		  osMessageQueueReset(xStateQueueHandle);

		  /* creating tasks again */
		  xTransmissionHandle = osThreadNew(TransmissionTask, NULL, &xTransmission_attributes);

		  xuROSHandle = osThreadNew(uROSTask, NULL, &xuROS_attributes);
  }
  osThreadExit();
  /* USER CODE END RestartTask */
}


/* Regulator_Callback function */
/*!
 * Timer for control PID regulator
 * @param argument
 */
void Regulator_Callback(void *argument)
{
  /* USER CODE BEGIN Regulator_Callback */
	PID_calc(0);
	PID_calc(1);
  /* USER CODE END Regulator_Callback */
}

/* Track_Callback function */
/*!
 * Timer for tracking position
 * @param argument
 */
void Track_Callback(void *argument)
{
	/* USER CODE BEGIN Track_Callback */

	/* USER CODE END Track_Callback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE END Application */

