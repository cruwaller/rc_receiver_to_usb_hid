/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_usart.h"

/* Private includes ----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/
#define DBG_UART_TX GPIO('A', 9)
//#define DBG_UART_RX GPIO('A', 10)

//#define RECEIVER_UART_TX GPIO('A', 2)
// next line is for default Bluepill RX pin
#define RECEIVER_UART_RX GPIO('A', 3)
// next line is for CC3D Flexi port RX pin
// #define RECEIVER_UART_RX GPIO('C', 11)




enum {
  IO_PORT_A = 'A',
  IO_PORT_B = 'B',
  IO_PORT_C = 'C',
  IO_PORT_D = 'D',
  IO_PORT_E = 'E',
  IO_PORT_F = 'F'
};
#define IO_CONCAT(_P) IO_PORT_ ## _P
#define IO_CREATE_IMPL(port, pin) ((IO_CONCAT(port) * 16) + pin)
#define IO_CREATE(...)  IO_CREATE_IMPL(__VA_ARGS__)
#define IO_GET_PORT(_p) (((_p) / 16) - 'A')
#define IO_GET_PIN(_p)  ((_p) % 16)
#define GPIO(R, P)      (((R) * 16) + (P))

#define GPIO_INPUT        0
#define GPIO_OUTPUT       1
#define GPIO_AF           2
#define GPIO_ANALOG       3
#define GPIO_OPEN_DRAIN   0x100
#define GPIO_FUNCTION(fn) (GPIO_AF | ((fn) << 4))

struct gpio_pin {
  GPIO_TypeDef *reg;
  uint32_t bit;
};
struct gpio_pin GPIO_Setup(uint32_t pin, uint32_t mode, int pullup);
static inline void GPIO_Write(struct gpio_pin gpio, uint32_t state) {
  if (state)
    gpio.reg->BSRR = gpio.bit;
  else
    gpio.reg->BSRR = (gpio.bit << 16);
}
static inline uint8_t GPIO_Read(struct gpio_pin gpio) {
  return !!(gpio.reg->IDR & gpio.bit);
}

void GPIO_SetupPin(GPIO_TypeDef *regs, uint32_t pos, uint32_t mode, int pullup);

#define barrier() __asm__ __volatile__("": : :"memory")

static inline void write_u8(void *addr, uint8_t val) {
    barrier();
    *(volatile uint8_t *)addr = val;
}
static inline uint8_t read_u8(const void *addr) {
    uint8_t val = *(volatile const uint8_t *)addr;
    barrier();
    return val;
}


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
