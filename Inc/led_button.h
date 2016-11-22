/*
 * led_button.h
 *
 *  Created on: 07.03.2016
 *      Author: revelo
 */

#ifndef LED_BUTTON_H_
#define LED_BUTTON_H_

#include "stm32f4xx_hal.h"

/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_Exported_Types
 * @{
 */
typedef enum {
	LED4 = 0, LED3 = 1, LED5 = 2, LED6 = 3
} Led_TypeDef;

typedef enum {
	BUTTON_KEY = 0,
} Button_TypeDef;

typedef enum {
	BUTTON_MODE_GPIO = 0, BUTTON_MODE_EXTI = 1
} ButtonMode_TypeDef;
/**
 * @}
 */

/** @addtogroup STM32F4_DISCOVERY_LOW_LEVEL_LED
 * @{
 */
#define LEDn                             4

#define LED4_PIN                         GPIO_PIN_12
#define LED4_GPIO_PORT                   GPIOD
#define LED4_GPIO_CLK_ENABLE()           __GPIOD_CLK_ENABLE()
#define LED4_GPIO_CLK_DISABLE()          __GPIOD_CLK_DISABLE()

#define LED3_PIN                         GPIO_PIN_13
#define LED3_GPIO_PORT                   GPIOD
#define LED3_GPIO_CLK_ENABLE()           __GPIOD_CLK_ENABLE()
#define LED3_GPIO_CLK_DISABLE()          __GPIOD_CLK_DISABLE()

#define LED5_PIN                         GPIO_PIN_14
#define LED5_GPIO_PORT                   GPIOD
#define LED5_GPIO_CLK_ENABLE()           __GPIOD_CLK_ENABLE()
#define LED5_GPIO_CLK_DISABLE()          __GPIOD_CLK_DISABLE()

#define LED6_PIN                         GPIO_PIN_15
#define LED6_GPIO_PORT                   GPIOD
#define LED6_GPIO_CLK_ENABLE()           __GPIOD_CLK_ENABLE()
#define LED6_GPIO_CLK_DISABLE()          __GPIOD_CLK_DISABLE()

#define LEDx_GPIO_CLK_ENABLE(__INDEX__) do{if((__INDEX__) == 0) LED4_GPIO_CLK_ENABLE(); else \
                                           if((__INDEX__) == 1) LED3_GPIO_CLK_ENABLE(); else \
                                           if((__INDEX__) == 2) LED5_GPIO_CLK_ENABLE(); else \
                                           if((__INDEX__) == 3) LED6_GPIO_CLK_ENABLE(); \
                                           }while(0)

#define LEDx_GPIO_CLK_DISABLE(__INDEX__) do{if((__INDEX__) == 0) LED4_GPIO_CLK_DISABLE(); else \
                                            if((__INDEX__) == 1) LED3_GPIO_CLK_DISABLE(); else \
                                            if((__INDEX__) == 2) LED5_GPIO_CLK_DISABLE(); else \
                                            if((__INDEX__) == 3) LED6_GPIO_CLK_DISABLE(); \
                                            }while(0)
/**
 * @}
 */

/** @addtogroup STM32F4_DISCOVERY_LOW_LEVEL_BUTTON
 * @{
 */
#define BUTTONn                          1

/**
 * @brief Wakeup push-button
 */
#define KEY_BUTTON_PIN                GPIO_PIN_0
#define KEY_BUTTON_GPIO_PORT          GPIOA
#define KEY_BUTTON_GPIO_CLK_ENABLE()  __GPIOA_CLK_ENABLE()
#define KEY_BUTTON_GPIO_CLK_DISABLE() __GPIOA_CLK_DISABLE()
#define KEY_BUTTON_EXTI_IRQn          EXTI0_IRQn

#define BUTTONx_GPIO_CLK_ENABLE(__INDEX__)    do{if((__INDEX__) == 0) KEY_BUTTON_GPIO_CLK_ENABLE(); \
                                                }while(0)

#define BUTTONx_GPIO_CLK_DISABLE(__INDEX__)    do{if((__INDEX__) == 0) KEY_BUTTON_GPIO_CLK_DISABLE(); \
                                                 }while(0)
/**
 * @}
 */

void BSP_LED_Toggle(Led_TypeDef Led);
void BSP_LED_Init(Led_TypeDef Led);
void BSP_LED_On(Led_TypeDef Led);
void BSP_LED_Off(Led_TypeDef Led);
void BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef Mode);
uint8_t get_button_state(void);
void set_button_state(void);
void reset_button_state(void);

#endif /* LED_BUTTON_H_ */
