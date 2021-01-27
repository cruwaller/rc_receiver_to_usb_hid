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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "uart.h"
#include "helpers.h"
#if PROTO_CRSF
#include "crsf.h"
#elif PROTO_SBUS
#include "sbus.h"
#endif
#include "usb_device.h"   // hUsbDeviceFS
#include "usbd_hid.h"     // USBD_HID_SendReport


/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
#define PIN_IO_TEST_OUT GPIO('B', 10)
#define PIN_IO_TEST_IN  GPIO('B', 9)

/* Private variables ---------------------------------------------------------*/
#if defined(PIN_IO_TEST_OUT)
struct gpio_pin test_io_out;
struct gpio_pin test_io_in;
uint32_t test_triggered;
#endif


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);


/* Private user code ---------------------------------------------------------*/
extern uint32_t SystemCoreClock;
#define clockCyclesPerMicrosecond()  ( SystemCoreClock / 1000000U )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
#define microsecondsToClockCycles(a) ( (a) * clockCyclesPerMicrosecond() )

uint32_t timer_read_time(void)
{
  return DWT->CYCCNT;
}
uint32_t micros(void)
{
  return clockCyclesToMicroseconds(timer_read_time());
}
#ifdef DWT_BASE
static void dwt_access(uint8_t ena)
{
#if (__CORTEX_M == 0x07U)
    /*
    * Define DWT LSR mask which is (currentuly) not defined by the CMSIS.
    * Same as ITM LSR one.
    */
#if !defined DWT_LSR_Present_Msk
#define DWT_LSR_Present_Msk ITM_LSR_Present_Msk
#endif
#if !defined DWT_LSR_Access_Msk
#define DWT_LSR_Access_Msk ITM_LSR_Access_Msk
#endif
    uint32_t lsr = DWT->LSR;

    if ((lsr & DWT_LSR_Present_Msk) != 0) {
        if (ena) {
        if ((lsr & DWT_LSR_Access_Msk) != 0) { //locked
            DWT->LAR = 0xC5ACCE55;
        }
        } else {
        if ((lsr & DWT_LSR_Access_Msk) == 0) { //unlocked
            DWT->LAR = 0;
        }
        }
    }
#else /* __CORTEX_M */
    UNUSED(ena);
#endif /* __CORTEX_M */
}
static uint32_t dwt_init(void)
{

    /* Enable use of DWT */
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }

    /* Unlock */
    dwt_access(1);

    /* Reset the clock cycle counter value */
    DWT->CYCCNT = 0;

    /* Enable  clock cycle counter */
    DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk;

    /* 3 NO OPERATION instructions */
    __asm volatile(" nop      \n\t"
                   " nop      \n\t"
                   " nop      \n\t");

    /* Check if clock cycle counter has started */
    return (DWT->CYCCNT) ? 0 : 1;
}
#endif /* DWT_BASE */


void gpio_port_pin_get(uint32_t io, void ** port, uint32_t * pin)
{
  *pin = IO_GET_PIN(io);
  io = IO_GET_PORT(io);
  *port = (void*)((uintptr_t)GPIOA_BASE + (io * 0x0400UL));
}

void gpio_port_clock(uint32_t port)
{
  // Enable gpio clock
  switch (port) {
    case GPIOA_BASE:
      __HAL_RCC_GPIOA_CLK_ENABLE();
      break;
    case GPIOB_BASE:
      __HAL_RCC_GPIOB_CLK_ENABLE();
      break;
#ifdef __HAL_RCC_GPIOC_CLK_ENABLE
    case GPIOC_BASE:
      __HAL_RCC_GPIOC_CLK_ENABLE();
      break;
#endif
#ifdef __HAL_RCC_GPIOD_CLK_ENABLE
    case GPIOD_BASE:
      __HAL_RCC_GPIOD_CLK_ENABLE();
      break;
#endif
#ifdef __HAL_RCC_GPIOE_CLK_ENABLE
    case GPIOE_BASE:
      __HAL_RCC_GPIOE_CLK_ENABLE();
      break;
#endif
#ifdef __HAL_RCC_GPIOF_CLK_ENABLE
    case GPIOF_BASE:
      __HAL_RCC_GPIOF_CLK_ENABLE();
      break;
#endif
    default:
      break;
    }
}

void GPIO_SetupPin(GPIO_TypeDef *regs, uint32_t pos, uint32_t mode, int pullup)
{
  gpio_port_clock((uint32_t)regs);

#if defined(STM32F1)
  // Configure GPIO
  uint32_t shift = (pos % 8) * 4, msk = 0xf << shift, cfg;

  if (mode == GPIO_INPUT) {
    cfg = pullup ? 0x8 : 0x4;
  } else if (mode == GPIO_OUTPUT) {
    cfg = 0x1; // push-pull, 0b00 | max speed 2 MHz, 0b01
  } else if (mode == (GPIO_OUTPUT | GPIO_OPEN_DRAIN)) {
    cfg = 0x5; // Open-drain, 0b01 | max speed 2 MHz, 0b01
  } else if (mode == GPIO_ANALOG) {
    cfg = 0x0;
  } else {
    // Alternate function
    if (mode & GPIO_OPEN_DRAIN)
      // output open-drain mode, 10MHz
      cfg = 0xd;
      // output open-drain mode, 50MHz
      //cfg = 0xF;
    else if (pullup > 0)
      // input pins use GPIO_INPUT mode on the stm32f1
      cfg = 0x8;
    else
      // output push-pull mode, 10MHz
      cfg = 0x9;
      // output push-pull mode, 50MHz
      //cfg = 0xB;
  }
  if (pos & 0x8)
    regs->CRH = (regs->CRH & ~msk) | (cfg << shift);
  else
    regs->CRL = (regs->CRL & ~msk) | (cfg << shift);

  if (pullup > 0)
    regs->BSRR = 1 << pos;
  else if (pullup < 0)
    regs->BSRR = 1 << (pos + 16);
#else
  uint32_t const bit_pos = 0x1 << pos;
  if (mode == GPIO_INPUT) {
    LL_GPIO_SetPinMode(regs, bit_pos, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(regs, bit_pos, (0 < pullup ? LL_GPIO_PULL_UP : LL_GPIO_PULL_DOWN));
  } else if (mode == GPIO_OUTPUT) {
    LL_GPIO_SetPinMode(regs, bit_pos, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(regs, bit_pos, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinSpeed(regs, bit_pos, LL_GPIO_SPEED_FREQ_LOW);
    LL_GPIO_ResetOutputPin(regs, bit_pos);
  } else {
    mode = (mode >> 4) & 0xff;
    LL_GPIO_SetPinMode(regs, bit_pos, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinPull(regs, bit_pos, (0 < pullup ? LL_GPIO_PULL_UP : LL_GPIO_PULL_DOWN));
    if (pos & 0x8) {
      LL_GPIO_SetAFPin_8_15(regs, bit_pos, mode);
    } else {
      LL_GPIO_SetAFPin_0_7(regs, bit_pos, mode);
    }
  }
#endif
}

struct gpio_pin
GPIO_Setup(uint32_t io, uint32_t mode, int pullup)
{
  uint32_t pin = IO_GET_PIN(io);
  io = IO_GET_PORT(io);
  GPIO_TypeDef *port = (void*)((uintptr_t)GPIOA_BASE + (io * 0x0400UL));

  GPIO_SetupPin(port, pin, mode, pullup);

  return (struct gpio_pin){.reg = port, .bit = (1 << pin)};
}

#if NEGATIVE_ALLOWED
#define MAP_x16 MAP_I16
#else
#define MAP_x16 MAP_U16
#endif

struct hid_report {
  int16_t analogs[NUM_ANALOGS];
  uint8_t buttons[((NUM_BUTTONS + 7) / 8)];
} PACKED;

void send_to_usb(uint16_t * rc_data, uint8_t len)
{
  uint8_t iter, btns;
  struct hid_report report;

#if defined(PIN_IO_TEST_OUT)
  if ((2 * CHANNEL_OUT_VALUE_MIN) <= rc_data[5]) {
    GPIO_Write(test_io_in, 1);
    uint32_t diff = micros() - test_triggered;
    (void)diff;
  }
#endif

  // analog channels
  for (iter = 0; iter < NUM_ANALOGS; iter++) {
    report.analogs[iter] = MAP_x16(rc_data[iter],
      CHANNEL_OUT_VALUE_MIN, CHANNEL_OUT_VALUE_MAX,
      ANALOG_MIN, ANALOG_MAX);
    if (iter == 1) /* Y is inverted */
      report.analogs[1] = ANALOG_MAX - report.analogs[1];
  }

  // buttons are bit
  for (btns = iter; btns < len; btns++) {
    uint8_t idx = (btns - NUM_ANALOGS);
    if (CHANNEL_OUT_VALUE_MID <= rc_data[btns])
      report.buttons[idx/8] |= (1 << idx);
  }
  USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&report, sizeof(report));
}


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_AFIO_REMAP_SWJ_DISABLE();

    /* Init DWT if present */
#ifdef DWT_BASE
    if (dwt_init()) {
        Error_Handler();
    }
#endif

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
#ifdef GPIOD_BASE
  gpio_port_clock(GPIOD_BASE); // USB ??
#endif

#if defined(PIN_IO_TEST_OUT)
  test_io_out = GPIO_Setup(PIN_IO_TEST_OUT, GPIO_OUTPUT, -1);
  test_io_in = GPIO_Setup(PIN_IO_TEST_IN, GPIO_OUTPUT, -1);
#endif

  // UART init
  uart_init(RX_BAUDRATE, RECEIVER_UART_RX, DBG_UART_TX);
  // USB HID init
  MX_USB_DEVICE_Init();

  /* Infinite loop */
  uint32_t last = HAL_GetTick();
  uint16_t channels[NUM_ANALOGS + NUM_BUTTONS];
  uint8_t data;
  while (1) {
    if (uart_receive_timeout(&data, 1, 20) == UART_OK) {
#if defined(PIN_IO_TEST_OUT)
        GPIO_Write(test_io_out, 0);
#endif
#if PROTO_CRSF
      if (crsf_parse_byte(data)) {
        crsf_get_rc_data(channels, ARRAY_SIZE(channels));
        send_to_usb(channels, ARRAY_SIZE(channels));
      }
#elif PROTO_SBUS
      if (sbus_parse_byte(data)) {
        sbus_get_rc_data(channels, ARRAY_SIZE(channels));
        send_to_usb(channels, ARRAY_SIZE(channels));
      }
#endif

#if defined(PIN_IO_TEST_OUT)
      if (200 <= (HAL_GetTick() - last)) {
        last = HAL_GetTick();
        // TODO: random timing!!
        test_triggered = micros();
        GPIO_Write(test_io_in, 0);
        GPIO_Write(test_io_out, 1);
      }
#endif
    }
  }
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
