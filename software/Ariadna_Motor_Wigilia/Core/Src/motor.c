/*
 * motor.c
 *
 *  Created on: Dec 16, 2021
 *      Author: ermoz
 */

#include "motor.h"

void Motor_Init(Motor *motor, TIM_HandleTypeDef *htim, uint32_t channel, GPIO_TypeDef *engA_port, uint16_t engA_pin, GPIO_TypeDef *engB_port, uint16_t engB_pin) {
	motor->htim = htim;
	motor->channel = channel;
	motor->engA_port = engA_port;
	motor->engA_pin = engA_pin;
	motor->engB_port = engB_port;
	motor->engB_pin = engB_pin;

	motor->speed = 0;
	motor->compare = 0;

	HAL_TIM_PWM_Start(motor->htim, motor->channel);
}

// speed in range [-1; 1]
void Motor_SetSpeed(Motor *motor, float speed) {
	motor->speed = speed;

	if(motor->speed>0) {
		HAL_GPIO_WritePin(motor->engA_port, motor->engA_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(motor->engB_port, motor->engB_pin, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(motor->engA_port, motor->engA_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(motor->engB_port, motor->engB_pin, GPIO_PIN_SET);
	}

	motor->compare = fabs(speed)*__HAL_TIM_GET_AUTORELOAD(motor->htim);

	__HAL_TIM_SET_COMPARE(motor->htim, motor->channel, motor->compare);
}

void Motor_SetCompare(Motor *motor, uint32_t compare) {
	motor->compare = compare;

	__HAL_TIM_SET_COMPARE(motor->htim, motor->channel, motor->compare);
}
