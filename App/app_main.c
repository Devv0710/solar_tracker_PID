/*
 * app_main.c
 *
 *  Created on: Oct 25, 2025
 *      Author: Devv
 */
#include "main.h"

// Secuencia half-step (8 pasos)
const uint8_t step_sequence[8][4] = { { 1, 0, 0, 0 }, { 1, 1, 0, 0 }, { 0, 1, 0,
		0 }, { 0, 1, 1, 0 }, { 0, 0, 1, 0 }, { 0, 0, 1, 1 }, { 0, 0, 0, 1 }, {
		1, 0, 0, 1 } };

void stepMotor(int step);
void stepForward(int steps, uint32_t delay_ms);
void stepBackward(int steps, uint32_t delay_ms);

void app_main() {
	while(1)
	{
		// Gira en una dirección usando el delay 'tp' del PDF [cite: 159]
		stepForward(20000, 4);  // 512 pasos con 4 ms entre pasos
//		HAL_Delay(1000);

		// Gira en la dirección contraria
//		stepBackward(512, 5);
//		HAL_Delay(1000);
	}
}


void stepMotor(int step ) {
	HAL_GPIO_WritePin(M_1_GPIO_Port, M_1_Pin,
			step_sequence[step][0] ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(M_2_GPIO_Port, M_2_Pin,
			step_sequence[step][1] ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(M_3_GPIO_Port, M_3_Pin,
			step_sequence[step][2] ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(M_4_GPIO_Port, M_4_Pin,
			step_sequence[step][3] ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void stepForward(int steps, uint32_t delay_ms) {
	for (int i = 0; i < steps; i++) {
		stepMotor(i % 8);
		HAL_Delay(delay_ms);
	}
}

void stepBackward(int steps, uint32_t delay_ms) {
	for (int i = steps; i > 0; i--) {
		stepMotor(i % 8);
		HAL_Delay(delay_ms);
	}
}

