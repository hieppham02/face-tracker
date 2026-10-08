/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
    uint16_t pan;
    uint16_t tilt;
} ServoPosition;

typedef struct
{
    GPIO_PinState raw;
    GPIO_PinState stable;
    uint32_t changed_at;
} ButtonState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define JOY_X_MIN      3
#define JOY_X_CENTER   2000
#define JOY_X_MAX      4095

#define JOY_Y_MIN      3
#define JOY_Y_CENTER   2000
#define JOY_Y_MAX      4095

#define DEAD_ZONE       250
#define SERVO_UPDATE_MS 15

#define MAX_STEP_X10    30

#define PAN_POS_GAIN    100
#define PAN_NEG_GAIN    100

#define TILT_POS_GAIN   100
#define TILT_NEG_GAIN   100

#define MAX_SAVED_POSITIONS 20U
#define BUTTON_DEBOUNCE_MS  30U
#define PLAYBACK_WAIT_MS    750U

#define PLAYBACK_SPEED_DPS  200U
#define PLAYBACK_UPDATE_MS  20U
#if PLAYBACK_SPEED_DPS < 1 || PLAYBACK_SPEED_DPS > 360
#error "PLAYBACK_SPEED_DPS must be between 1 and 360"
#endif
#define SAVE_BTN_PORT       GPIOB
#define SAVE_BTN_PIN        GPIO_PIN_15
#define RUN_BTN_PORT        GPIOB
#define RUN_BTN_PIN         GPIO_PIN_14
/* Last 1 KB page reserved by the Keil IROM size (63 KB). */
#define WAYPOINT_FLASH_ADDRESS 0x0800FC00U
#define WAYPOINT_FLASH_MAGIC   0x5750U
#define WAYPOINT_FLASH_VERSION 1U
#define WAYPOINT_FLASH_WORDS   (4U + MAX_SAVED_POSITIONS * 2U)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
volatile uint16_t adc_value[2];
static uint8_t rx_byte;
static char rx_buffer[20];
static uint8_t rx_index;
static uint8_t rx_overflow;
/* ISR publishes angle + 1; main owns all servo positions and PWM writes. */
static volatile uint16_t uart_pan_pending;
static volatile uint8_t playback_active;
static int32_t pan_angle = 90;
static int32_t tilt_angle = 90;
static int32_t pan_accum;
static int32_t tilt_accum;
static int8_t pan_direction;
static int8_t tilt_direction;
static uint32_t last_update;
static ServoPosition saved_positions[MAX_SAVED_POSITIONS];
static uint8_t saved_count;
/* First SAVE after boot or RUN starts a new teaching list. */
static uint8_t teach_new_session = 1;
static uint8_t playback_index;
static uint32_t playback_since;
static uint8_t playback_moving;
static int32_t playback_start_pan;
static int32_t playback_start_tilt;
static int32_t playback_delta_pan;
static int32_t playback_delta_tilt;
static uint32_t playback_duration;
static uint32_t playback_update_at;
static ButtonState save_button;
static ButtonState run_button;
static uint8_t joystick_moved;
static uint8_t playback_cancelled;
/* Watch: 0=not attempted, 1=save OK, 2=write/verify failed. */
static volatile uint8_t waypoint_flash_status;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void Joystick_Control_Task(void);
void Servo_Memory_Task(void);
void Servo_Demo_Task(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Record: magic, version, count, CRC16, then 20 pan/tilt pairs.
   Magic is programmed last so interrupted writes are not accepted at boot. */
static uint16_t Waypoint_CRC(const uint16_t *words)
{
    uint16_t crc = 0xFFFFU;
    for (uint32_t i = 1; i < WAYPOINT_FLASH_WORDS; ++i)
    {
        if (i == 3U) continue;
        for (uint32_t byte = 0; byte < 2U; ++byte)
        {
            crc ^= (uint16_t)(((words[i] >> (byte * 8U)) & 0xFFU) << 8);
            for (uint32_t bit = 0; bit < 8U; ++bit)
                crc = (uint16_t)((crc & 0x8000U) ? ((uint32_t)crc << 1) ^ 0x1021U : (uint32_t)crc << 1);
        }
    }
    return crc;
}

static uint8_t Waypoint_ReadRecord(uint16_t *words)
{
    const volatile uint16_t *flash = (const volatile uint16_t *)WAYPOINT_FLASH_ADDRESS;
    for (uint32_t i = 0; i < WAYPOINT_FLASH_WORDS; ++i) words[i] = flash[i];
    if (words[0] != WAYPOINT_FLASH_MAGIC || words[1] != WAYPOINT_FLASH_VERSION ||
        words[2] > MAX_SAVED_POSITIONS || words[3] != Waypoint_CRC(words)) return 0;
    for (uint32_t i = 0; i < words[2]; ++i)
        if (words[4U + i * 2U] > 180U || words[5U + i * 2U] > 180U) return 0;
    return 1;
}

static void Waypoint_FlashLoad(void)
{
    uint16_t words[WAYPOINT_FLASH_WORDS];
    saved_count = 0;
    if (!Waypoint_ReadRecord(words)) return;
    saved_count = (uint8_t)words[2];
    for (uint32_t i = 0; i < saved_count; ++i)
    {
        saved_positions[i].pan = words[4U + i * 2U];
        saved_positions[i].tilt = words[5U + i * 2U];
    }
    /* Restore the list only; never start playback or move to a saved angle at boot. */
}

static void Waypoint_FlashSave(void)
{
    uint16_t words[WAYPOINT_FLASH_WORDS] = {0};
    uint16_t verify[WAYPOINT_FLASH_WORDS];
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t page_error;
    HAL_StatusTypeDef status;
    words[0] = WAYPOINT_FLASH_MAGIC;
    words[1] = WAYPOINT_FLASH_VERSION;
    words[2] = saved_count;
    for (uint32_t i = 0; i < saved_count; ++i)
    {
        words[4U + i * 2U] = saved_positions[i].pan;
        words[5U + i * 2U] = saved_positions[i].tilt;
    }
    words[3] = Waypoint_CRC(words);
    waypoint_flash_status = 2;
    if (HAL_FLASH_Unlock() != HAL_OK) return;
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = WAYPOINT_FLASH_ADDRESS;
    erase.NbPages = 1;
    status = HAL_FLASHEx_Erase(&erase, &page_error);
    for (uint32_t i = 1; status == HAL_OK && i < WAYPOINT_FLASH_WORDS; ++i)
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                                  WAYPOINT_FLASH_ADDRESS + i * 2U, words[i]);
    if (status == HAL_OK)
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, WAYPOINT_FLASH_ADDRESS, words[0]);
    HAL_FLASH_Lock();
    if (status == HAL_OK && Waypoint_ReadRecord(verify) &&
        memcmp(words, verify, sizeof(words)) == 0) waypoint_flash_status = 1;
}

static void Servo_ApplyPosition(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 500U + ((uint32_t)pan_angle * 2000U) / 180U);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 500U + ((uint32_t)tilt_angle * 2000U) / 180U);
}

static void Servo_ResetAccumulators(void)
{
    pan_accum = 0;
    tilt_accum = 0;
    pan_direction = 0;
    tilt_direction = 0;
}

static uint16_t UART_TakePanCommand(void)
{
    uint32_t primask = __get_PRIMASK();
    uint16_t command;
    __disable_irq();
    command = uart_pan_pending;
    uart_pan_pending = 0;
    __set_PRIMASK(primask);
    return command;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;
    if (rx_byte == '\n')
    {
        uint16_t angle = 0;
        uint8_t valid = (rx_index > 0U && !rx_overflow);
        /* Preserve the existing decimal pan-angle + newline protocol. */
        for (uint8_t i = 0; valid && i < rx_index; ++i)
        {
            if (rx_buffer[i] < '0' || rx_buffer[i] > '9') valid = 0;
            else
            {
                angle = (uint16_t)(angle * 10U + (uint16_t)(rx_buffer[i] - '0'));
                if (angle > 180U) valid = 0;
            }
        }
        if (valid && !playback_active) uart_pan_pending = angle + 1U;
        rx_index = 0;
        rx_overflow = 0;
    }
    else if (rx_byte != '\r')
    {
        if (rx_index < sizeof(rx_buffer) - 1U) rx_buffer[rx_index++] = (char)rx_byte;
        else rx_overflow = 1;
    }
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

static uint8_t Button_Pressed(ButtonState *button, GPIO_PinState raw, GPIO_PinState active, uint32_t now)
{
    if (raw != button->raw)
    {
        button->raw = raw;
        button->changed_at = now;
    }
    if (raw != button->stable &&
        (uint32_t)(now - button->changed_at) >= BUTTON_DEBOUNCE_MS)
    {
        button->stable = raw;
        return raw == active;
    }
    return 0;
}

static void Joystick_UpdateAxis(int32_t sample, int32_t minimum, int32_t center, int32_t maximum, int32_t positive_gain, int32_t negative_gain, int32_t *angle, int32_t *accum, int8_t *direction)
{
    int32_t error = sample - center;
    int8_t next_direction = error > DEAD_ZONE ? 1 : (error < -DEAD_ZONE ? -1 : 0);
    int32_t range, speed10, step;
    if (next_direction != *direction) *accum = 0;
    *direction = next_direction;
    if (!next_direction)
    {
        *accum = 0;
        return;
    }
    range = next_direction > 0 ? maximum - center - DEAD_ZONE : center - minimum - DEAD_ZONE;
    speed10 = 10 + (((next_direction > 0 ? error : -error) - DEAD_ZONE) * (MAX_STEP_X10 - 10)) / range;
    speed10 = speed10 * (next_direction > 0 ? positive_gain : negative_gain) / 100;
    *accum += speed10;
    step = *accum / 10;
    *accum %= 10;
    *angle += next_direction * step;
    if (*angle > 180) 
		{ 
			*angle = 180; 
			*accum = 0; 
		}
    if (*angle < 0) 
		{ 
			*angle = 0; 
			*accum = 0; 
		}
}

void Joystick_Control_Task(void)
{
    uint32_t now = HAL_GetTick();
    int32_t joy_x = adc_value[0];
    int32_t joy_y = adc_value[1];
    int32_t error_x = joy_x - JOY_X_CENTER;
    int32_t error_y = joy_y - JOY_Y_CENTER;
    uint16_t uart_command;
    playback_cancelled = 0;
    joystick_moved = (error_x > DEAD_ZONE || error_x < -DEAD_ZONE || error_y > DEAD_ZONE || error_y < -DEAD_ZONE);
    if (playback_active)
    {
        if (!joystick_moved) return;
        playback_active = 0;
        playback_cancelled = 1;
        playback_moving = 0;
        Servo_ResetAccumulators();
        (void)UART_TakePanCommand();
        last_update = now - SERVO_UPDATE_MS;
    }
    uart_command = UART_TakePanCommand();
    if (uart_command)
    {
        pan_angle = uart_command - 1U;
        pan_accum = 0;
        pan_direction = 0;
        Servo_ApplyPosition();
    }
    if ((uint32_t)(now - last_update) < SERVO_UPDATE_MS) return;
    last_update = now;
    Joystick_UpdateAxis(joy_x, JOY_X_MIN, JOY_X_CENTER, JOY_X_MAX, PAN_POS_GAIN, PAN_NEG_GAIN, &pan_angle, &pan_accum, &pan_direction);
    Joystick_UpdateAxis(joy_y, JOY_Y_MIN, JOY_Y_CENTER, JOY_Y_MAX, TILT_POS_GAIN, TILT_NEG_GAIN, &tilt_angle, &tilt_accum, &tilt_direction);
    Servo_ApplyPosition();
}

static void Playback_ApplyWaypoint(uint32_t now)
{
    int32_t pan_distance, tilt_distance;
    uint32_t distance;
    playback_start_pan = pan_angle;
    playback_start_tilt = tilt_angle;
    playback_delta_pan = (int32_t)saved_positions[playback_index].pan - pan_angle;
    playback_delta_tilt = (int32_t)saved_positions[playback_index].tilt - tilt_angle;
    pan_distance = playback_delta_pan < 0 ? -playback_delta_pan : playback_delta_pan;
    tilt_distance = playback_delta_tilt < 0 ? -playback_delta_tilt : playback_delta_tilt;
    distance = (uint32_t)(pan_distance > tilt_distance ? pan_distance : tilt_distance);
    /* Common duration synchronizes the axes; round up to honor the speed limit. */
    playback_duration = (distance * 1000U + PLAYBACK_SPEED_DPS - 1U) / PLAYBACK_SPEED_DPS;
    playback_moving = distance != 0U;
    playback_since = now;
    playback_update_at = now;
    Servo_ResetAccumulators();
}

static void Playback_MoveTask(uint32_t now)
{
    uint32_t elapsed = (uint32_t)(now - playback_since);
    if (elapsed >= playback_duration)
    {
        pan_angle = saved_positions[playback_index].pan;
        tilt_angle = saved_positions[playback_index].tilt;
        Servo_ApplyPosition();
        playback_moving = 0;
        playback_since = now; /* Dwell starts after the command reaches the target. */
        return;
    }
    if ((uint32_t)(now - playback_update_at) < PLAYBACK_UPDATE_MS) return;
    playback_update_at = now;
    /* Integer interpolation preserves current commanded angles for cancellation. */
    pan_angle = playback_start_pan +
        (playback_delta_pan * (int32_t)elapsed) / (int32_t)playback_duration;
    tilt_angle = playback_start_tilt +
        (playback_delta_tilt * (int32_t)elapsed) / (int32_t)playback_duration;
    Servo_ApplyPosition();
}

void Servo_Memory_Task(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t save_pressed = Button_Pressed(&save_button, HAL_GPIO_ReadPin(SAVE_BTN_PORT, SAVE_BTN_PIN), 1, now);
    uint8_t run_pressed = Button_Pressed(&run_button, HAL_GPIO_ReadPin(RUN_BTN_PORT, RUN_BTN_PIN), 1, now);
    if (playback_active)
    {
        if (playback_moving)
        {
            Playback_MoveTask(now);
            return;
        }
        if ((uint32_t)(now - playback_since) >= PLAYBACK_WAIT_MS)
        {
            ++playback_index;
            if (playback_index < saved_count) Playback_ApplyWaypoint(now);
            else
            {
                playback_active = 0;
                Servo_ResetAccumulators();
                last_update = now;
            }
        }
        return;
    }
    if (playback_cancelled) return;
    if (save_pressed)
    {
        if (teach_new_session)
        {
            saved_count = 0;
            playback_index = 0;
            teach_new_session = 0;
        }
        if (saved_count < MAX_SAVED_POSITIONS)
        {
            saved_positions[saved_count].pan = (uint16_t)pan_angle;
            saved_positions[saved_count].tilt = (uint16_t)tilt_angle;
            ++saved_count;
            Waypoint_FlashSave();
        }
    }
    if (run_pressed && saved_count && !joystick_moved)
    {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        playback_active = 1;
        teach_new_session = 1;
        uart_pan_pending = 0;
        __set_PRIMASK(primask);
        playback_index = 0;
        Playback_ApplyWaypoint(now);
    }
}

void Servo_Demo_Task(void)
{
    pan_angle = 180;
    tilt_angle = 180;
    Servo_ResetAccumulators();
    Servo_ApplyPosition();
    HAL_Delay(1000);

    pan_angle = 90;
    tilt_angle = 90;
    Servo_ApplyPosition();
    HAL_Delay(1000);
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
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
    Waypoint_FlashLoad();
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_value, 2);
	
	Servo_ApplyPosition();
	
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	
	save_button.raw = save_button.stable = HAL_GPIO_ReadPin(SAVE_BTN_PORT, SAVE_BTN_PIN);
	run_button.raw = run_button.stable = HAL_GPIO_ReadPin(RUN_BTN_PORT, RUN_BTN_PIN);
	
	last_update = HAL_GetTick();
	
	save_button.changed_at = run_button.changed_at = last_update;
	
	HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
while (1)
  {
    Joystick_Control_Task();
    Servo_Memory_Task();
    //Servo_Demo_Task();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
/* Joystick SW on PB15: active LOW. PB14 RUN retains generated pulldown. */
  GPIO_InitStruct.Pin = SAVE_BTN_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SAVE_BTN_PORT, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
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
