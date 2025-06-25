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
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <time.h>
#include "robot.h"

// Basic micro-ROS includes
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <uxr/client/transport.h>
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

I2C_HandleTypeDef hi2c1;

osThreadId ethernetTaskHandle;
osThreadId robotTaskHandle;
osThreadId rosTaskHandle;
/* USER CODE BEGIN PV */
extern struct netif gnetif;

// Simple micro-ROS variables
rcl_node_t node;
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
rclc_support_t support;
rclc_executor_t executor;
std_msgs__msg__Int32 pub_msg;
std_msgs__msg__String sub_msg;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
void ethernetStatusCheck(void const * argument);
void robotControl(void const * argument);
void ros(void const * argument);

/* USER CODE BEGIN PFP */
void debugNetworkStatus(void);

// Simple micro-ROS functions
void subscription_callback(const void * msgin);

// Transport function prototypes (from udp_transport.c)
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _gettimeofday(struct timeval *tv, void *tzvp)
{
  (void)tzvp;
  uint64_t t = HAL_GetTick();
  tv->tv_sec = t / 1000;
  tv->tv_usec = (t % 1000) * 1000;
  return 0;
}

// Simple subscription callback
void subscription_callback(const void * msgin)
{
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  if (msg != NULL && msg->data.data != NULL) {
    
    // Parse servo command: "servoX:Y" where X=servo number, Y=angle
    char* command = (char*)msg->data.data;
    
    // Look for "servo" followed by a number and colon
    if (strncmp(command, "servo", 5) == 0) {
      // Extract servo number
      int servo_num = -1;
      int angle = -1;
      
      // Find the colon separator
      char* colon_pos = strchr(command + 5, ':');
      if (colon_pos != NULL) {
        // Parse servo number (between "servo" and ":")
        servo_num = atoi(command + 5);
        
        // Parse angle (after ":")
        angle = atoi(colon_pos + 1);
        
        // Validate ranges
        if (servo_num >= 0 && servo_num <= 15 && angle >= 0 && angle <= 180) {
          // Valid command - control the servo
          set_servo(servo_num, angle);
          
          // Toggle blue LED to indicate successful command
          HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
        }
      }
    }
  }
}
void debugNetworkStatus(void)
{  
  if (netif_is_up(&gnetif) && netif_is_link_up(&gnetif)) {
    // Network is fully working - solid green
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);  // Red OFF
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);     // Green ON
  } else {
    // No physical link - solid red
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);   // Green OFF
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);    // Red ON
  }
}
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(100);
  /* USER CODE END 2 */

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
  /* definition and creation of ethernetTask */
  osThreadDef(ethernetTask, ethernetStatusCheck, osPriorityAboveNormal, 0, 256);
  ethernetTaskHandle = osThreadCreate(osThread(ethernetTask), NULL);

  /* definition and creation of robotTask */
  osThreadDef(robotTask, robotControl, osPriorityIdle, 0, 128);
  robotTaskHandle = osThreadCreate(osThread(robotTask), NULL);

  /* definition and creation of rosTask */
  osThreadDef(rosTask, ros, osPriorityNormal, 0, 3840);
  rosTaskHandle = osThreadCreate(osThread(rosTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

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
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
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
  hi2c1.Init.Timing = 0x20404768;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_14|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB14 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_14|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_ethernetStatusCheck */
/**
  * @brief  Function implementing the ethernetTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ethernetStatusCheck */
void ethernetStatusCheck(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN 5 */

  /* Infinite loop */
  for(;;)
  {
	debugNetworkStatus();
    osDelay(2000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_robotControl */
/**
* @brief Function implementing the robotControlTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_robotControl */
void robotControl(void const * argument)
{
  /* USER CODE BEGIN robotControl */
  
  // Initialize robot hardware
  pca9685_init();
  
  // Initialize servos to safe starting positions
  set_servo(0, 90);   // Servo 0 to center
  set_servo(1, 90);   // Servo 1 to center
  set_servo(2, 90);   // Servo 2 to center
  
  for (;;) {
  }
  /* USER CODE END robotControl */
}

/* USER CODE BEGIN Header_ros */
/**
* @brief Function implementing the rosTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ros */
void ros(void const * argument)
{
  /* USER CODE BEGIN ros */
  
  // Wait for network to be ready - use longer delays to give Ethernet task time
  uint32_t network_wait_count = 0;
  while (!netif_is_up(&gnetif) || !netif_is_link_up(&gnetif)) {
    network_wait_count++;
    
    // Blink blue LED to show we're waiting for network
    if (network_wait_count % 10 == 0) {
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
    }
    
    osDelay(1000);  // 1 second delay - gives plenty of time for Ethernet task
  }
  
  // Network is ready - turn off blue LED
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
  
  // Setup micro-ROS transport
  rmw_uros_set_custom_transport(
    false,
    "192.168.10.1",  // Change to your micro-ROS agent IP
    cubemx_transport_open,
    cubemx_transport_close,
    cubemx_transport_write,
    cubemx_transport_read
  );

  // Initialize micro-ROS
  rcl_allocator_t allocator = rcl_get_default_allocator();
  
  // Create support
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    // Error - blink red LED fast
    for(;;) {
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
      osDelay(100);
    }
  }

  // Create node
  if (rclc_node_init_default(&node, "stm32_simple_node", "", &support) != RCL_RET_OK) {
    // Error - blink red LED fast
    for(;;) {
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
      osDelay(100);
    }
  }

  // Create publisher
  if (rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "counter") != RCL_RET_OK) {
    // Error - blink red LED medium
    for(;;) {
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
      osDelay(400);
    }
  }

  // Create subscriber
  if (rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "servo_cmd") != RCL_RET_OK) {
    // Error - blink red LED medium
    for(;;) {
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
      osDelay(400);
    }
  }

  // Create executor
  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) {
    // Error - blink blue LED  slow
    for(;;) {
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
      osDelay(1000);
    }
  }

  // Add subscription to executor
  if (rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, 
    &subscription_callback, ON_NEW_DATA) != RCL_RET_OK) {
    // Error - blink red LED  slow
    for(;;) {
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
      osDelay(1000);
    }
  }

  // Initialize messages
  pub_msg.data = 0;
  sub_msg.data.data = (char*)malloc(50);
  sub_msg.data.size = 0;
  sub_msg.data.capacity = 50;

  // Success - solid green LED
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  
  uint32_t publish_timer = 0;
  
  /* Infinite loop */
  for(;;)
  {
    // Handle subscriptions
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    
    // Publish counter every 1 second
    if (publish_timer >= 100) { // 100 * 10ms = 1 second
      pub_msg.data++;
      rcl_publish(&publisher, &pub_msg, NULL);
      publish_timer = 0;
    } else {
      publish_timer++;
    }
    
    osDelay(10);
  }
  /* USER CODE END ros */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM9 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM9)
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
