#ifndef __KEY_H__
#define	__KEY_H__

#include "stm32f10x.h"

void Key_GPIO_Config(void);
uint8_t Key_Scan(GPIO_TypeDef* GPIOx,u16 GPIO_Pin);
#endif 
