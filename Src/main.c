/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "dma.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ws2812.h"
#include "led_animation.h"
#include "nrf24l01.h"
#include "nb_delay.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void stm_sleep()
{
	HAL_PWR_EnterSLEEPMode(PWR_REGULATOR_VOLTAGE_SCALE1, PWR_SLEEPENTRY_WFI);
}
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
  MX_SPI1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  nrf_init_multicast(&hspi2);
  ws2812_init(8, &hspi1);

  struct ws2812_color color = {
		  .red = 10,
		  .green = 0,
		  .blue = 0,
  };

  ws2812_set_color_all(&color);

  nb_delay button_delay = nb_delay_create(7000, NULL);
  nb_delay ws_2812_transmit_delay = nb_delay_create(50, ws2812_transmit);
  nb_delay nrf_receive_delay = nb_delay_create(100, NULL);

  animation power_up_animation1 = animation_create(0, 8, 200, ws2812_on);
  animation_add_step(&power_up_animation1, 0, 1, 1000, animation_nop);
  animation_add_step(&power_up_animation1, 0, 8, 0, ws2812_off);

//  animation power_up_animation2 = animation_create(15, 7, 200, ws2812_nop);
//  animation_add_step(&power_up_animation2, 0, 1, 2000, animation_nop);
//  animation_add_step(&power_up_animation2, 15, 7, 0, ws2812_off);


  animation power_down_animation1 = animation_create(0, 8, 0, ws2812_on);
    animation_add_step(&power_down_animation1, 0, 1, 400, animation_nop);
  animation_add_step(&power_down_animation1, 7, -1, 200, ws2812_off);

//  animation power_down_animation2 = animation_create(8, 16, 0, ws2812_on);
//  animation_add_step(&power_down_animation2, 8, 16, 200, ws2812_off);


  animation main_animation = animation_create(0, 8, 200, ws2812_on);
  animation_add_step(&main_animation, 0, 1, 1000, animation_nop);
  animation_add_step(&main_animation, 8, -1, 100, ws2812_off);


  enum power_state {
	  stm_to_sleep,
	  stm_to_running,
	  stm_sleeping,
	  stm_running,
  };

  enum button_state {
	  pressed,
	  holding,
	  released,
  };


  static uint8_t current_power_state = stm_sleeping;
  static uint8_t button_state = released;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin)){
		  button_state = released;
	  } else {
		  if (button_state == pressed)
			  button_state = holding;
		  else if (button_state == released)
			  button_state = pressed;
	  }


	  if (nb_delay_check(button_delay)) {
		  switch (current_power_state) {
			case stm_sleeping:
				current_power_state = stm_to_running;
				animation_start(&power_up_animation1);
				break;
			case stm_running:
				current_power_state = stm_to_sleep;
				animation_start(&power_down_animation1);
				nrf_standby();
				nrf_power_down();
				break;
		}
	  }

	  if (current_power_state == stm_to_running) {
		  if (animation_ended(power_up_animation1)) {
	  		  current_power_state = stm_running;
	  		  nrf_power_up();
	  		  nrf_send_command(FLUSH_RX);
	  		  nrf_send_command(FLUSH_TX);
	  		  nrf_rx_mode();
	  	  }
	  }

	  if (current_power_state == stm_to_sleep) {
		  if (animation_ended(power_down_animation1)) {
	  		  current_power_state = stm_sleeping;
	  	  }
	  }


	  if (current_power_state == stm_running) {
		  if (button_state == pressed && animation_ended(main_animation)){
			  	static uint8_t data = 'A';
			  	nrf_write_payload(&data, 1);
			  	nrf_tx_mode();
			  	while(!nrf_payload_transmitted() || !nrf_tx_fifo_empty());
			  	animation_start(&main_animation);
			  	nrf_rx_mode();
		  }
	  	  if (nb_delay_check(nrf_receive_delay)){
	  		  if (nrf_payload_received()) {
	  			  while (!nrf_rx_fifo_empty()) {
	  				  static uint8_t data = 0;
	  				  nrf_read_payload(&data, 1);
	  				  if (data == 'A'){
	  					  LD2_GPIO_Port-> ODR ^= LD2_Pin;
	  					  animation_start(&main_animation);
	  				  }
	  			  }
	  		  }
	  	  }
	  }

	  if (button_state == released){
		  nb_delay_restart(button_delay);
	  	  if (current_power_state == stm_sleeping)
	  		  stm_sleep();
	  }

	  animation_animate(&main_animation);
	  animation_animate(&power_down_animation1);
	  animation_animate(&power_up_animation1);

	  nb_delay_check(ws_2812_transmit_delay);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV4;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_24;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
