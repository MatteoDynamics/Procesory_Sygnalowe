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

#include <adc.h>
#include <cmsis_gcc.h>
#include <crc.h>
#include <dcmi.h>
#include <dma2d.h>
#include <eth.h>
#include <fatfs.h>
#include <fmc.h>
#include <gpio.h>
#include <i2c.h>
#include <ltdc.h>
#include <quadspi.h>
#include <rtc.h>
#include <sai.h>
#include <sdmmc.h>
#include <spdifrx.h>
#include <spi.h>
#include <stm32746g_discovery.h>
#include <stm32746g_discovery_audio.h>
#include <stm32746g_discovery_lcd.h>
#include <stm32f746xx.h>
#include <stm32f7xx_hal_def.h>
#include <stm32f7xx_hal_flash_ex.h>
#include <stm32f7xx_hal_pwr.h>
#include <stm32f7xx_hal_pwr_ex.h>
#include <stm32f7xx_hal_rcc.h>
#include <stm32f7xx_hal_rcc_ex.h>
#include <stm32f7xx_hal_tim.h>
#include <sys/_stdint.h>
#include <tim.h>
#include <usart.h>
#include <wm8994/wm8994.h>
#include "main.h"

#define ARM_MATH_CM7
#include "arm_math.h"
#include "logo.h"
#include "lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#include "main.h"
#include <stdio.h>
#include <string.h>


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Definicje stalych
#define SAMPLE_RATE 48000  // Czestotliwosc pr�bkowania [Hz]
#define SINE_FREQ 1000    // Czestotliwosc sygnalu [Hz]
#define TABLE_SIZE 256     // Rozmiar tablicy sinusow
#define AUDIO_BUFFER_SIZE 480  // Rozmiar bufora audio
#define CORDIC_ITERATIONS 6  // Liczba iteracji CORDIC
#define TAYLOR_TERMS 8        // Liczba wyrazow szeregu Taylora
#define SIGNAL_LENGTH 10000 //DLUGOSC SYGNALOW DO MNOZENIA
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

int16_t curr_sig = 0; // zmienna do wyswietlania w stm32 cube monitor
static float global_phase = 0.0f;

// Bufory sygnałów do mnozenia
int16_t signalA_int[SIGNAL_LENGTH];
int16_t signalB_int[SIGNAL_LENGTH];
int16_t result_int[SIGNAL_LENGTH];

float signalA_float[SIGNAL_LENGTH];
float signalB_float[SIGNAL_LENGTH];
float result_float[SIGNAL_LENGTH];

const int16_t lookup_table[TABLE_SIZE] = {
     0,   807,  1614,  2420,  3224,  4027,  4827,
     5624,  6417,  7207,  7992,  8773,  9548, 10317,
    11080, 11837, 12586, 13328, 14061, 14786, 15502,
    16208, 16905, 17592, 18267, 18932, 19585, 20226,
    20855, 21472, 22075, 22665, 23241, 23803, 24351,
    24883, 25401, 25903, 26390, 26860, 27315, 27752,
    28173, 28577, 28963, 29332, 29683, 30016, 30330,
    30627, 30904, 31163, 31403, 31624, 31826, 32008,
    32171, 32315, 32439, 32543, 32627, 32692, 32737,
    32761, 32766, 32751, 32717, 32662, 32587, 32493,
    32379, 32246, 32092, 31920, 31728, 31516, 31286,
    31036, 30768, 30481, 30175, 29851, 29510, 29150,
    28772, 28377, 27965, 27536, 27090, 26627, 26149,
    25654, 25144, 24619, 24079, 23524, 22955, 22372,
    21775, 21165, 20542, 19907, 19260, 18601, 17931,
    17250, 16558, 15856, 15145, 14425, 13695, 12958,
    12212, 11459, 10700,  9933,  9161,  8383,  7600,
     6813,  6021,  5226,  4427,  3626,  2822,  2017,
     1211,   404,  -404, -1211, -2017, -2822, -3626,
    -4427, -5226, -6021, -6813, -7600, -8383, -9161,
    -9933,-10700,-11459,-12212,-12958,-13695,-14425,
   -15145,-15856,-16558,-17250,-17931,-18601,-19260,
   -19907,-20542,-21165,-21775,-22372,-22955,-23548,
   -24079,-24619,-25144,-25654,-26149,-26627,-27090,
   -27536,-27965,-28377,-28772,-29150,-29510,-29851,
   -30175,-30481,-30768,-31036,-31286,-31516,-31728,
   -31920,-32092,-32246,-32379,-32493,-32587,-32662,
   -32717,-32751,-32766,-32761,-32737,-32692,-32627,
   -32543,-32439,-32315,-32171,-32008,-31826,-31624,
   -31403,-31163,-30904,-30627,-30330,-30016,-29683,
   -29332,-28963,-28577,-28173,-27752,-27315,-26860,
   -26390,-25903,-25401,-24883,-24351,-23803,-23241,
   -22665,-22075,-21472,-20855,-20226,-19585,-18932,
   -18267,-17592,-16905,-16208,-15502,-14786,-14061,
   -13328,-12586,-11837,-11080,-10317, -9548, -8773,
    -7992, -7207, -6417, -5624, -4827, -4027, -3224,
    -2420, -1614,  -807
};
int16_t audio_buffer[AUDIO_BUFFER_SIZE];
// Tablica kat�w dla CORDIC
const float cordic_angles[CORDIC_ITERATIONS] = {
    0.785398163, 0.463647609, 0.244978663, 0.124354994, 0.06241881, 0.031239833,
    0.015623729, 0.007812341, 0.003906230, 0.001953123, 0.000976562, 0.000488281,
    0.000244141, 0.00012207, 0.000061035, 0.000030518
};
const float cordic_gain = 0.607252935; // Stala skalujaca CORDIC

// 🌀 Returns value from LUT using angle in radians
int16_t Sine_LookupTable(float angle) {
    float normalized = angle / (2.0f * PI);        // Map to [0, 1)
    int index = (int)(normalized * TABLE_SIZE) % TABLE_SIZE;
    if (index < 0) index += TABLE_SIZE;
    return lookup_table[index];
}


// Generowanie sinusa za pomoca tablicy stan�w

// Generowanie sinusa za pomoca szeregu Taylora
int16_t Sine_Taylor(float angle) {
    float x = fmodf(angle, 2.0f * PI);
    if (x > PI) x -= 2.0f * PI;
    float result = x;
    float term = x;
    float x2 = x * x;
    for (int i = 1; i < TAYLOR_TERMS; i++) {
        term *= -x2 / ((2 * i) * (2 * i + 1));
        result += term;
    }
    return (int16_t)(32767.0f * result);
}

int16_t Sine_CORDIC(float theta) {
    // Normalize angle to [0, 2π)
    while (theta >= 2.0f * PI) theta -= 2.0f * PI;
    while (theta < 0.0f) theta += 2.0f * PI;

    float angle;
    int8_t sign_sin = 1, sign_cos = 1;

    // Determine quadrant
    if (theta >= 0.0f && theta < PI / 2.0f) {
        angle = theta;
        sign_sin = 1;
        sign_cos = 1;
    } else if (theta >= PI / 2.0f && theta < PI) {
        angle = PI - theta;
        sign_sin = 1;
        sign_cos = -1;
    } else if (theta >= PI && theta < 3.0f * PI / 2.0f) {
        angle = theta - PI;
        sign_sin = -1;
        sign_cos = -1;
    } else {
        angle = 2.0f * PI - theta;
        sign_sin = -1;
        sign_cos = 1;
    }

    float x = cordic_gain; // Initial x = K (CORDIC gain)
    float y = 0.0f;
    float z = angle;

    // CORDIC iterations
    for (uint8_t i = 0; i < CORDIC_ITERATIONS; i++) {
        float dx = x * (1.0f / (1 << i)); // 2^(-i)
        float dy = y * (1.0f / (1 << i)); // 2^(-i)
        if (z >= 0.0f) {
            x -= dy;
            y += dx;
            z -= cordic_angles[i];
        } else {
            x += dy;
            y -= dx;
            z += cordic_angles[i];
        }
    }

    // Scale to 16-bit integer (assuming max amplitude fits in int16_t)
    return (int16_t)(sign_sin * y * 32767.0f); // Scale to [-32767, 32767]
}


void GenerateSineWave(int16_t* buffer, uint32_t size, int16_t (*sine_func)(float)) {
    float phase_step = 2.0f * PI * SINE_FREQ / SAMPLE_RATE;
    float phase = 0.0f;

    for (uint32_t i = 0; i < size; i++) {
        buffer[i] = sine_func(phase); // Use raw value directly
        phase += phase_step;
        if (phase >= 2.0f * PI) phase -= 2.0f * PI;
    }
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */
void mac_int16(const int16_t *a, const int16_t *b, int16_t *result, uint32_t length)
void mac_float(const float *a, const float *b, float *result, uint32_t length);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void BSP_AUDIO_OUT_TransferComplete_CallBack()
{
	BSP_LED_Off(LED1);
}


int __io_putchar(int ch)
{
    if (ch == '\n') {
        uint8_t ch2 = '\r';
        HAL_UART_Transmit(&huart2, &ch2, 1, HAL_MAX_DELAY);
    }
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return 1;
}

void mac_int16(const int16_t *a, const int16_t *b, int16_t *result, uint32_t length)
{
    for (uint32_t i = 0; i < length; ++i)
    {
        result[i] += a[i] * b[i];
    }
}

// Funkcja wykonująca operację MAC dla tablic typu float
void mac_float(const float *a, const float *b, float *result, uint32_t length)
{
    for (uint32_t i = 0; i < length; ++i)
    {
        result[i] += a[i] * b[i];
    }
}

uint32_t get_pixel_color(uint8_t index) {
    uint8_t r = (index >> 5) & 0x07; // 3 bits for red
    uint8_t g = (index >> 2) & 0x07; // 3 bits for green
    uint8_t b = index & 0x03;        // 2 bits for blue

    // Scale 3-3-2 to 8-8-8
    uint8_t r8 = (r << 5) | (r << 2) | (r >> 1);
    uint8_t g8 = (g << 5) | (g << 2) | (g >> 1);
    uint8_t b8 = (b << 6) | (b << 4) | (b << 2);

    return (0xFF << 24) | (r8 << 16) | (g8 << 8) | b8; // ARGB8888
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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC3_Init();
  MX_CRC_Init();
  MX_DCMI_Init();
  MX_DMA2D_Init();
  MX_ETH_Init();
  MX_FMC_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_LTDC_Init();
  MX_QUADSPI_Init();
  MX_RTC_Init();
  MX_SAI2_Init();
  MX_SDMMC1_SD_Init();
  MX_SPDIFRX_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  BSP_LED_Init(LED1);
  //NVIC_DisableIRQ(TIM6_DAC_IRQn);
  init_screen();



      int16_t audio_buffer[AUDIO_BUFFER_SIZE] = {0};
          BSP_LED_Init(LED1);

          // Initialize WM8994 audio codec for input and output
          if (BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_HEADPHONE, 80, 16000) != AUDIO_OK) {
        	  BSP_LED_Toggle(LED1);
        	  HAL_Delay(100);
        	  BSP_LED_Toggle(LED1);
                  while (1); // Blad inicjalizacji
              }

          //BSP_LED_Toggle(LED1); // Toggle to confirm audio init
          //HAL_Delay(100);
          GenerateSineWave(audio_buffer, AUDIO_BUFFER_SIZE, Sine_LookupTable);
              GenerateSineWave(audio_buffer, AUDIO_BUFFER_SIZE, Sine_Taylor);
              GenerateSineWave(audio_buffer, AUDIO_BUFFER_SIZE, Sine_CORDIC);;
              if(BSP_AUDIO_OUT_Play(audio_buffer, 512 * sizeof(uint16_t)) != AUDIO_OK) {
                      	  BSP_LED_Off(LED1);
                      	  while (1);
                        }
              int16_t audio_buffer2[AUDIO_BUFFER_SIZE] = {0};
              if(BSP_AUDIO_IN_Init(16000, DEFAULT_AUDIO_IN_BIT_RESOLUTION, DEFAULT_AUDIO_IN_CHANNEL_NBR != AUDIO_OK)) {
              	  BSP_LED_Off(LED1);
              	  while (1);
                }
              BSP_AUDIO_IN_Record(&audio_buffer2, sizeof(audio_buffer2));

              printf("Start programu...\r\n");

                 // Inicjalizacja sygnałów
                 for (uint32_t i = 0; i < SIGNAL_LENGTH; ++i)
                 {
                     signalA_int[i] = i % 256;
                     signalB_int[i] = (255 - i) % 256;

                     signalA_float[i] = (float)(i % 256);
                     signalB_float[i] = (float)((255 - i) % 256);
                 }

                 // Pomiar czasu - int16_t
                 uint32_t start = HAL_GetTick();
                 multiply_int16(signalA_int, signalB_int, result_int, SIGNAL_LENGTH);
                 uint32_t stop = HAL_GetTick();
                 printf("Czas wykonania multiply_int16: %lu ms\r\n", (stop - start));

                 // Pomiar czasu - float
                 start = HAL_GetTick();
                 multiply_float(signalA_float, signalB_float, result_float, SIGNAL_LENGTH);
                 stop = HAL_GetTick();
                 printf("Czas wykonania multiply_float: %lu ms\r\n", (stop - start));

                      //  BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);
//    BSP_LCD_DisplayStrint)"BSP Example", CENTER_MODE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
             //    GenerateSineWave(audio_buffer,AUDIO_BUFFER_SIZE,Sine_LookupTable);
                 GenerateSineWave(audio_buffer,AUDIO_BUFFER_SIZE,Sine_CORDIC);
                 int i = 0;
  while (1)
  {
    /* USER CODE END WHILE */
	  curr_sig = audio_buffer[i];
	  i++;

	  //BSP_LED_Toggle(LED1);
	  HAL_Delay(1);
	  i = i%AUDIO_BUFFER_SIZE;
	  //curr_sig = curr_sig +1

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_SAI2
                              |RCC_PERIPHCLK_SDMMC1|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 5;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_8;
  PeriphClkInitStruct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLSAI;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
  PeriphClkInitStruct.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
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
