/*
 * motor.h
 *
 *  Created on: Dec 16, 2021
 *      Author: ermoz
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

typedef struct {
	TIM_HandleTypeDef *htim;
	uint32_t channel;

	GPIO_TypeDef *engA_port;
	GPIO_TypeDef *engB_port;
	uint16_t engA_pin;
	uint16_t engB_pin;

	float speed;
	uint32_t compare;
} Motor;

void Motor_Init(Motor *, TIM_HandleTypeDef *, uint32_t, GPIO_TypeDef *, uint16_t, GPIO_TypeDef *, uint16_t);
void Motor_SetSpeed(Motor *, float);
void Motor_SetCompare(Motor *, uint32_t);

#endif /* INC_MOTOR_H_ */
